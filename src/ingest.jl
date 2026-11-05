# ─────────────────────────────────────────────────────────────────────────────
# Record ingest: from device records to predetection integrations, per channel.
# On a fixed channel table: the primary code period grid, record continuity,
# the secondary code wipe-off and the coherent integration. An integration is
# *not* cut where the NCO command changed, because `calc_mean_nco_command`
# already weights a change inside an integration.
# ─────────────────────────────────────────────────────────────────────────────

# ── Records to correlators ───────────────────────────────────────────────────

# The record's meaningful taps as the template correlator's accumulator vector:
# latest first, one `ComplexF64` per tap for a single antenna and an
# `SVector{M}` per tap for `M`. Zero-cost reshaping of the fixed tuple.
@inline function _record_accumulators(template::AbstractCorrelator{1}, record::DeviceRecord)
    N = length(get_accumulators(template))
    SVector{N,ComplexF64}(ntuple(i -> record.taps[i], Val(N)))
end
@inline function _record_accumulators(
    template::AbstractCorrelator{M},
    record::DeviceRecord,
) where {M}
    N = length(get_accumulators(template))
    SVector{N,SVector{M,ComplexF64}}(
        ntuple(i -> SVector{M,ComplexF64}(ntuple(a -> record.taps[(a-1)*N+i], Val(M))), Val(N)),
    )
end

@inline _record_correlator(template::AbstractCorrelator, record::DeviceRecord) =
    with_accumulators(template, _record_accumulators(template, record))

# Sum two correlators' accumulators, keeping everything else from the first.
@inline _add_accumulators(a::AbstractCorrelator, b::AbstractCorrelator) =
    with_accumulators(a, get_accumulators(a) .+ get_accumulators(b))

@inline _negate_accumulators(c::AbstractCorrelator) =
    with_accumulators(c, -get_accumulators(c))

@inline _scale_accumulators(c::AbstractCorrelator, scale::Float64) =
    with_accumulators(c, get_accumulators(c) ./ scale)

# ── The primary code period grid ─────────────────────────────────────────────

struct CodePeriodSpan
    periods::Float64
    wraps::Int
    code_period_phase::Float64
end

@inline _chip_rate_hz(signal::AbstractGNSSSignal) = ustrip(Hz, uconvert(Hz, get_chip_rate(signal)))

# The channel's code frequency: the chip rate plus its code Doppler.
@inline _code_frequency_hz(core::LoopCore, ch::Int, signal::AbstractGNSSSignal) =
    _chip_rate_hz(signal) + core.channels.code_doppler[ch]

@inline _code_period_seconds(signal::AbstractGNSSSignal) = get_code_length(signal) / _chip_rate_hz(signal)

# Primary code periods a record spans — exactly, as a fraction, Doppler-adjusted.
@inline function _record_code_periods(core::LoopCore, ch::Int, signal, record::DeviceRecord)
    record.num_integrated_samples * _code_frequency_hz(core, ch, signal) /
    (get_code_length(signal) * core.channels.sampling_frequency[ch])
end

# How close to a code period boundary still counts as being on it: one sample,
# and never less than a chip and a half, as a fraction of a code period.
@inline _code_period_boundary_tolerance(core::LoopCore, ch::Int, signal) =
    max(1.5, _chip_rate_hz(signal) / core.channels.sampling_frequency[ch]) / get_code_length(signal)

@inline _snap_code_period_fraction(fraction, tolerance) =
    (fraction < tolerance || fraction > 1 - tolerance) ? 0.0 : fraction

@inline function _record_start_fraction(core::LoopCore, ch::Int, record, periods, reported, tolerance)
    T = core.channels
    previous_end = T.last_record_end[ch]
    standing = T.code_period_phase[ch]
    if previous_end != typemin(Int64) && !isnan(standing)
        record_start = record.sample_index - record.num_integrated_samples
        periods_per_sample = periods / max(1, record.num_integrated_samples)
        hole = (record_start - previous_end) * periods_per_sample
        return _snap_code_period_fraction(mod(standing + hole, 1.0), tolerance)
    end
    ends_at = isnan(reported) ? 0.0 : reported
    _snap_code_period_fraction(mod(ends_at - periods, 1.0), tolerance)
end

@inline function _record_code_period_span(core::LoopCore, ch::Int, signal, record::DeviceRecord)
    code_length = get_code_length(signal)
    tolerance = _code_period_boundary_tolerance(core, ch, signal)
    periods = _record_code_periods(core, ch, signal, record)
    reported = isnan(record.code_phase) ? NaN : mod(record.code_phase / code_length, 1.0)
    start = _record_start_fraction(core, ch, record, periods, reported, tolerance)
    wraps = max(0, floor(Int, start + periods + tolerance))
    code_period_phase = _snap_code_period_fraction(
        isnan(reported) ? clamp(start + periods - wraps, 0.0, 1.0) : reported,
        tolerance,
    )
    CodePeriodSpan(periods, wraps, code_period_phase)
end

@inline _allows_partial_primary_records(core::LoopCore, signal) =
    _code_period_seconds(signal) > core.config.max_integration_time * (1 + 1e-9)

@inline _max_record_periods(core::LoopCore, signal) =
    core.config.max_integration_time / _code_period_seconds(signal)

# How long one of this channel's records nominally is, in device samples.
@inline function _nominal_record_samples(core::LoopCore, ch::Int, signal)
    period = floor(
        Int64,
        get_code_length(signal) * core.channels.sampling_frequency[ch] / _code_frequency_hz(core, ch, signal),
    )
    seen = core.channels.nominal_record_samples[ch]
    seen <= 0 ? period : min(period, seen)
end

# ── Continuity ───────────────────────────────────────────────────────────────

# A channel's records tile the sample axis. A hole shorter than a record after
# which a short record ends on the next boundary is a re-arm; a hole of a whole
# record or more is a record that never arrived, which cuts the navigation bit
# clock: the integration before it is closed, the hole is declared, and the
# satellite's bit synchronizer is restarted (`_restart_lost_bit_clock!`) rather
# than compensated.
function _account_record_continuity!(
    core::LoopCore,
    bank::ChannelBank,
    ch::Int,
    record::DeviceRecord,
    span::CodePeriodSpan,
)
    T = core.channels
    expected_start = T.last_record_end[ch]
    record_start = record.sample_index - record.num_integrated_samples
    if expected_start != typemin(Int64)
        gap = record_start - expected_start
        if gap > 0
            _emit_integration!(core, bank, ch)
            nominal = _nominal_record_samples(core, ch, bank.signal)
            if gap < nominal && record.num_integrated_samples < nominal
                T.rearm_dead_samples[ch] += gap
                core.rearm_gaps += 1
            else
                T.lost_record_samples[ch] += gap
                core.lost_record_gaps += 1
                T.bit_clock_lost[ch] = true
            end
        end
    end
    T.last_record_end[ch] = record.sample_index
    T.last_record_samples[ch] = record.num_integrated_samples
    T.nominal_record_samples[ch] = max(T.nominal_record_samples[ch], Int64(record.num_integrated_samples))
    T.code_period_phase[ch] = span.code_period_phase
    T.primary_wraps[ch] += span.wraps
    nothing
end

# ── Secondary code wipe-off ──────────────────────────────────────────────────

@inline _is_secondary_code_removed(core::LoopCore, ch::Int) =
    core.channels.secondary_wipe[ch] && core.channels.secondary_code_phase[ch] >= 0

@inline function _forget_secondary_code_phase!(core::LoopCore, ch::Int)
    core.channels.secondary_code_phase[ch] = -1
    core.channels.secondary_code_phase_sample[ch] = typemin(Int64)
    nothing
end

# Seed the secondary code phase from the bit synchronizer on the fold that
# finds sync, and drop it wherever sync no longer stands. Runs once per epoch,
# after the fold.
function _anchor_secondary_code_phase!(core::LoopCore, bank::ChannelBank, ch::Int)
    T = core.channels
    T.secondary_wipe[ch] || return nothing
    state = bank.states[ch]
    if !is_bit_or_secondary_code_synced(state)
        _forget_secondary_code_phase!(core, ch)
        return nothing
    end
    T.secondary_code_phase[ch] >= 0 && return nothing
    last_end = T.last_record_end[ch]
    last_end == typemin(Int64) && return nothing
    T.secondary_code_phase[ch] =
        mod(get_secondary_code_phase(state), get_secondary_code_length(bank.signal))
    T.secondary_code_phase_sample[ch] = last_end
    nothing
end

# Take the secondary code chip off one record and advance the phase over it,
# or — where the record does not start where the phase stands, or crosses a
# code period boundary inside itself — cut the integration and stop wiping
# until the next sync re-seeds the phase. Nothing is ever wiped at a guessed
# phase.
function _wipe_secondary_code!(
    core::LoopCore,
    bank::ChannelBank,
    ch::Int,
    correlator,
    record::DeviceRecord,
    span::CodePeriodSpan,
)
    T = core.channels
    _is_secondary_code_removed(core, ch) || return correlator
    signal = bank.signal
    within_one_code_period = span.wraps == 0 || (span.wraps == 1 && span.code_period_phase == 0.0)
    if record.sample_index - record.num_integrated_samples != T.secondary_code_phase_sample[ch] ||
       !within_one_code_period
        _emit_integration!(core, bank, ch)
        _forget_secondary_code_phase!(core, ch)
        return correlator
    end
    chip = GNSSSignals.secondary_value(get_secondary_code(signal), T.prn[ch], T.secondary_code_phase[ch])
    T.secondary_code_phase[ch] = mod(T.secondary_code_phase[ch] + span.wraps, get_secondary_code_length(signal))
    T.secondary_code_phase_sample[ch] = record.sample_index
    chip > 0 ? correlator : _negate_accumulators(correlator)
end

# ── Coherent integration ─────────────────────────────────────────────────────

# How many primary code periods to integrate into one predetection integration
# for this channel now: one before bit/secondary sync (the detectors take one
# prompt per code period), one for a secondary-coded signal whose phase is not
# yet known, and otherwise up to one bit, ending on the bit boundary the bit
# synchronizer is counting toward.
function _num_code_periods_to_integrate(core::LoopCore, bank::ChannelBank, ch::Int)
    state = bank.states[ch]
    is_bit_or_secondary_code_synced(state) || return 1
    core.channels.bit_clock_lost[ch] && return 1
    signal = bank.signal
    get_secondary_code_length(signal) == 1 || _is_secondary_code_removed(core, ch) || return 1
    max_num_code_periods = max_num_code_periods_to_integrate(signal)
    max_num_code_periods <= 1 && return 1
    requested =
        core.config.num_code_periods_to_integrate == 0 ? max_num_code_periods :
        min(core.config.num_code_periods_to_integrate, max_num_code_periods)
    # Integrations are stepped as they are emitted here, so the bit
    # synchronizer's own count is current; nothing is queued against it.
    counted = get_num_code_periods_into_bit(state)
    remaining = max_num_code_periods - mod(counted, max_num_code_periods)
    max(1, min(requested, remaining))
end

function _code_periods_to_integrate(core::LoopCore, bank::ChannelBank, ch::Int)
    num_code_periods = _num_code_periods_to_integrate(core, bank, ch)
    _allows_partial_primary_records(core, bank.signal) || return Float64(num_code_periods)
    min(Float64(num_code_periods), _max_record_periods(core, bank.signal))
end

@inline function _record_is_emittable(core::LoopCore, signal, ch::Int)
    T = core.channels
    T.partial_samples[ch] > 0 || return false
    _allows_partial_primary_records(core, signal) && return true
    phase = T.code_period_phase[ch]
    isnan(phase) || phase == 0.0
end

# Add one record to the channel's open predetection integration and, once it
# spans the coherent integration length, hand the sum to the loop.
function _accumulate!(
    core::LoopCore,
    bank::ChannelBank,
    ch::Int,
    correlator,
    record::DeviceRecord,
    span::CodePeriodSpan,
)
    T = core.channels
    signal = bank.signal
    tolerance = _code_period_boundary_tolerance(core, ch, signal)
    target = _code_periods_to_integrate(core, bank, ch)
    partial_primary = _allows_partial_primary_records(core, signal)
    if T.partial_samples[ch] > 0 &&
       !partial_primary &&
       T.partial_wraps[ch] >= max(1, floor(Int, target + tolerance)) &&
       _record_is_emittable(core, signal, ch)
        _emit_integration!(core, bank, ch)
        target = _code_periods_to_integrate(core, bank, ch)
    end
    if T.partial_samples[ch] == 0
        bank.partial[ch] = correlator
        T.partial_samples[ch] = record.num_integrated_samples
        T.partial_periods[ch] = span.periods
        T.partial_wraps[ch] = span.wraps
    else
        bank.partial[ch] = _add_accumulators(bank.partial[ch], correlator)
        T.partial_samples[ch] += record.num_integrated_samples
        T.partial_periods[ch] += span.periods
        T.partial_wraps[ch] += span.wraps
    end
    T.partial_end[ch] = record.sample_index
    T.partial_code_phase[ch] = record.code_phase
    long_enough =
        partial_primary ? T.partial_periods[ch] >= target - tolerance :
        T.partial_wraps[ch] >= max(1, floor(Int, target + tolerance))
    if long_enough && _record_is_emittable(core, signal, ch)
        _emit_integration!(core, bank, ch)
    elseif T.partial_periods[ch] >= target + 1
        # A whole code period past the target and no boundary to cut on: the
        # device's dump grid does not divide its code period. Hand over what
        # there is rather than accumulate for ever.
        _emit_integration!(core, bank, ch)
    end
    nothing
end

@inline function _discard_partial!(core::LoopCore, ch::Int)
    T = core.channels
    T.partial_samples[ch] = 0
    T.partial_periods[ch] = 0.0
    T.partial_wraps[ch] = 0
    T.partial_end[ch] = typemin(Int64)
    T.partial_first[ch] = false
    T.partial_code_phase[ch] = NaN
    nothing
end

# ── One record in ────────────────────────────────────────────────────────────

# Append one noise-reference record to its band's noise window: the record's
# taps, brought onto the satellites' scale first (the replica and code
# amplitudes the arm declared: a density left at the device's own amplitude
# would be off every C/N₀ by that factor squared), are the looks TrackingLoops'
# `append_noise_taps!` pools — one power sum for one antenna, the spatial
# covariance `Σ b·bᴴ` for several — over the record's own sample count.
function _append_noise_record!(bank::ChannelBank, core::LoopCore, ch::Int, record::DeviceRecord)
    T = core.channels
    b = core.bands[T.band[ch]]
    signal = bank.signal
    probe = TrackingLoops.NoiseProbe(
        signal,
        b.noise_prn,
        b.noise_code_phase,
        get_chip_rate(signal),
        (b.entry.intermediate_frequency_hz + b.noise_carrier_doppler) * Hz,
    )
    TrackingLoops.append_noise_taps!(
        b.noise_estimator,
        probe,
        _noise_taps(NumAntennas(b.noise_estimator), record, T.scale[ch]),
        Int(record.num_integrated_samples),
        T.sampling_frequency[ch] * Hz,
    )
    nothing
end

# A record's meaningful taps, scaled, as the looks `append_noise_taps!` pools:
# one `ComplexF64` per tap for a single antenna, an `SVector{M}` per tap for `M`
# (the record is antenna-major). Lazy, so nothing is allocated.
@inline function _noise_taps(::NumAntennas{1}, record::DeviceRecord, scale::Float64)
    taps = record.taps
    (taps[i] / scale for i = 1:min(Int(record.num_taps), MAX_RECORD_TAPS))
end
@inline function _noise_taps(::NumAntennas{M}, record::DeviceRecord, scale::Float64) where {M}
    taps = record.taps
    num_taps = min(Int(record.num_taps), MAX_RECORD_TAPS ÷ M)
    (SVector{M,ComplexF64}(ntuple(a -> taps[(a-1)*num_taps+i] / scale, Val(M))) for i = 1:num_taps)
end

function _ingest_in_bank!(bank::ChannelBank, core::LoopCore, ch::Int, record::DeviceRecord)
    T = core.channels
    if Int(record.num_taps) != get_num_correlators(bank.template)
        core.tap_layout_mismatches += 1
        return nothing
    end
    span = _record_code_period_span(core, ch, bank.signal, record)
    correlator = _record_correlator(bank.template, record)
    correlator = _wipe_secondary_code!(core, bank, ch, correlator, record, span)
    _account_record_continuity!(core, bank, ch, record, span)
    _accumulate!(core, bank, ch, correlator, record, span)
    T.records_folded[ch] += 1
    if T.signal_index[ch] == 1 && !isnan(record.code_phase)
        T.anchor_sample[ch] = record.sample_index
        T.anchor_code_phase[ch] = record.code_phase
    end
    nothing
end

"""
    ingest_record!(core, record)

Route one device record: drop it as stale when its channel is free, unconfirmed,
holds another PRN or its integration began before the confirmed arm; append it
to its band's noise window when the channel is the noise reference; otherwise add it to
the channel's open predetection integration, stepping the loop whenever one is
completed.
"""
function ingest_record!(core::LoopCore, record::DeviceRecord)
    ch = Int(record.channel)
    T = core.channels
    if !(1 <= ch <= core.num_channels) || !T.armed[ch]
        core.stale_dumps += 1
        return nothing
    end
    start = assignment_start(core.driver, ch)
    if start == typemax(Int64) || record.sample_index - record.num_integrated_samples < start
        core.stale_dumps += 1
        T.stale_records[ch] += 1
        return nothing
    end
    if Int(record.prn) != T.prn[ch]
        core.stale_dumps += 1
        T.stale_records[ch] += 1
        return nothing
    end
    if T.signal_index[ch] == 0
        with_bank(_append_noise_record!, core, ch, core, ch, record)
        return nothing
    end
    with_bank(_ingest_in_bank!, core, ch, core, ch, record)
    nothing
end
