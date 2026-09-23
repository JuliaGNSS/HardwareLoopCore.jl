# ─────────────────────────────────────────────────────────────────────────────
# Record ingest: from a device record to a folded loop record, per channel.
# The accounting is the receiver-side link's (GNSSReceiver.jl's
# `hardware_correlator.jl`), on a fixed channel table — the primary-code block
# grid, record continuity, the overlay removal and the coherent accumulation —
# minus the one rule the plan drops: a record is *not* cut where the NCO word
# changed, because `mean_nco_word` already weights a change inside a record.
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
    update_accumulator(template, _record_accumulators(template, record))

# Sum two correlators' accumulators, keeping everything else from the first.
@inline _add_accumulators(a::AbstractCorrelator, b::AbstractCorrelator) =
    update_accumulator(a, get_accumulators(a) .+ get_accumulators(b))

@inline _negate_accumulators(c::AbstractCorrelator) =
    update_accumulator(c, -get_accumulators(c))

@inline _scale_accumulators(c::AbstractCorrelator, scale::Float64) =
    update_accumulator(c, get_accumulators(c) ./ scale)

# ── The primary-code block grid ──────────────────────────────────────────────

struct RecordBlockSpan
    periods::Float64
    wraps::Int
    block_phase::Float64
end

@inline function _code_rate_hz(core::LoopCore, ch::Int, signal::AbstractGNSSSignal)
    ustrip(Hz, uconvert(Hz, get_code_frequency(signal))) + core.channels.code_doppler[ch]
end

@inline _code_period_seconds(signal::AbstractGNSSSignal) =
    get_code_length(signal) / ustrip(Hz, uconvert(Hz, get_code_frequency(signal)))

# Primary-code periods a record spans — exactly, as a fraction, Doppler-adjusted.
@inline function _record_code_periods(core::LoopCore, ch::Int, signal, record::DeviceRecord)
    record.integrated_samples * _code_rate_hz(core, ch, signal) /
    (get_code_length(signal) * core.channels.sampling_freq[ch])
end

# How close to a block boundary still counts as being on it: one sample, and
# never less than a chip and a half, as a fraction of a code period.
@inline _block_boundary_tolerance(core::LoopCore, ch::Int, signal) =
    max(
        1.5,
        ustrip(Hz, uconvert(Hz, get_code_frequency(signal))) / core.channels.sampling_freq[ch],
    ) / get_code_length(signal)

@inline _snap_block_fraction(fraction, tolerance) =
    (fraction < tolerance || fraction > 1 - tolerance) ? 0.0 : fraction

@inline function _record_start_fraction(core::LoopCore, ch::Int, record, periods, reported, tolerance)
    T = core.channels
    previous_end = T.last_record_end[ch]
    standing = T.block_phase[ch]
    if previous_end != typemin(Int64) && !isnan(standing)
        record_start = record.sample_index - record.integrated_samples
        periods_per_sample = periods / max(1, record.integrated_samples)
        hole = (record_start - previous_end) * periods_per_sample
        return _snap_block_fraction(mod(standing + hole, 1.0), tolerance)
    end
    ends_at = isnan(reported) ? 0.0 : reported
    _snap_block_fraction(mod(ends_at - periods, 1.0), tolerance)
end

@inline function _record_block_span(core::LoopCore, ch::Int, signal, record::DeviceRecord)
    code_length = get_code_length(signal)
    tolerance = _block_boundary_tolerance(core, ch, signal)
    periods = _record_code_periods(core, ch, signal, record)
    reported = isnan(record.code_phase) ? NaN : mod(record.code_phase / code_length, 1.0)
    start = _record_start_fraction(core, ch, record, periods, reported, tolerance)
    wraps = max(0, floor(Int, start + periods + tolerance))
    block_phase = _snap_block_fraction(
        isnan(reported) ? clamp(start + periods - wraps, 0.0, 1.0) : reported,
        tolerance,
    )
    RecordBlockSpan(periods, wraps, block_phase)
end

@inline _allows_partial_primary_records(core::LoopCore, signal) =
    _code_period_seconds(signal) > core.config.max_integration_time * (1 + 1e-9)

@inline _max_record_periods(core::LoopCore, signal) =
    core.config.max_integration_time / _code_period_seconds(signal)

# How long one of this channel's records nominally is, in device samples.
@inline function _nominal_record_samples(core::LoopCore, ch::Int, signal)
    period = floor(
        Int64,
        get_code_length(signal) * core.channels.sampling_freq[ch] / _code_rate_hz(core, ch, signal),
    )
    seen = core.channels.nominal_record_samples[ch]
    seen <= 0 ? period : min(period, seen)
end

# ── Continuity ───────────────────────────────────────────────────────────────

# A channel's records tile the sample axis. A hole shorter than a record after
# which a short record ends on the next boundary is a re-arm; a hole of a whole
# record or more is a record that never arrived, which cuts the navigation bit
# clock: the accumulation before it is closed, the hole is declared, and the
# satellite's bit clock is restarted (`_restart_bit_clock!`) rather than
# compensated.
function _account_record_continuity!(
    core::LoopCore,
    bank::ChannelBank,
    ch::Int,
    record::DeviceRecord,
    span::RecordBlockSpan,
)
    T = core.channels
    expected_start = T.last_record_end[ch]
    record_start = record.sample_index - record.integrated_samples
    if expected_start != typemin(Int64)
        gap = record_start - expected_start
        if gap > 0
            _emit_partial!(core, bank, ch)
            nominal = _nominal_record_samples(core, ch, bank.signal)
            if gap < nominal && record.integrated_samples < nominal
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
    T.last_record_samples[ch] = record.integrated_samples
    T.nominal_record_samples[ch] = max(T.nominal_record_samples[ch], Int64(record.integrated_samples))
    T.block_phase[ch] = span.block_phase
    T.primary_wraps[ch] += span.wraps
    nothing
end

# ── Secondary (overlay) code removal ─────────────────────────────────────────

@inline _is_secondary_code_removed(core::LoopCore, ch::Int) =
    core.channels.secondary_wipe[ch] && core.channels.secondary_phase[ch] >= 0

@inline function _forget_secondary_phase!(core::LoopCore, ch::Int)
    core.channels.secondary_phase[ch] = -1
    core.channels.secondary_phase_sample[ch] = typemin(Int64)
    nothing
end

# Seed the overlay counter from the bit buffer on the fold that finds sync, and
# drop it wherever sync no longer stands. Runs once per epoch, after the fold.
function _anchor_secondary_phase!(core::LoopCore, bank::ChannelBank, ch::Int)
    T = core.channels
    T.secondary_wipe[ch] || return nothing
    state = bank.states[ch]
    if !has_bit_or_secondary_code_been_found(state)
        _forget_secondary_phase!(core, ch)
        return nothing
    end
    T.secondary_phase[ch] >= 0 && return nothing
    last_end = T.last_record_end[ch]
    last_end == typemin(Int64) && return nothing
    T.secondary_phase[ch] =
        mod(state.bit_buffer.secondary_phase, get_secondary_code_length(bank.signal))
    T.secondary_phase_sample[ch] = last_end
    nothing
end

# Take the overlay chip off one record and advance the counter over it, or —
# where the record does not start where the counter stands, or crosses a block
# boundary inside itself — cut the accumulation and stop removing until the
# next sync re-seeds the phase. Nothing is ever wiped at a guessed phase.
function _wipe_secondary_code!(
    core::LoopCore,
    bank::ChannelBank,
    ch::Int,
    correlator,
    record::DeviceRecord,
    span::RecordBlockSpan,
)
    T = core.channels
    _is_secondary_code_removed(core, ch) || return correlator
    signal = bank.signal
    within_one_block = span.wraps == 0 || (span.wraps == 1 && span.block_phase == 0.0)
    if record.sample_index - record.integrated_samples != T.secondary_phase_sample[ch] ||
       !within_one_block
        _emit_partial!(core, bank, ch)
        _forget_secondary_phase!(core, ch)
        return correlator
    end
    chip = GNSSSignals.secondary_value(get_secondary_code(signal), T.prn[ch], T.secondary_phase[ch])
    T.secondary_phase[ch] = mod(T.secondary_phase[ch] + span.wraps, get_secondary_code_length(signal))
    T.secondary_phase_sample[ch] = record.sample_index
    chip > 0 ? correlator : _negate_accumulators(correlator)
end

# ── Coherent accumulation ────────────────────────────────────────────────────

# How many primary-code blocks to sum into one record for this channel now: one
# before bit/secondary sync (the detectors take one prompt per block), one for
# an overlaid signal whose phase is not yet known, and otherwise up to one
# symbol, landing on the symbol boundary the bit buffer is counting toward.
function _coherent_integration_blocks(core::LoopCore, bank::ChannelBank, ch::Int)
    state = bank.states[ch]
    has_bit_or_secondary_code_been_found(state) || return 1
    core.channels.bit_clock_lost[ch] && return 1
    signal = bank.signal
    get_secondary_code_length(signal) == 1 || _is_secondary_code_removed(core, ch) || return 1
    blocks_per_symbol = max_num_code_blocks_to_integrate(signal)
    blocks_per_symbol <= 1 && return 1
    requested =
        core.config.coherent_code_blocks == 0 ? blocks_per_symbol :
        min(core.config.coherent_code_blocks, blocks_per_symbol)
    # Records are stepped as they are emitted here, so the bit buffer's own
    # count is current; nothing is queued against it.
    counted = state.bit_buffer.prompt_accumulator_integrated_code_blocks
    remaining = blocks_per_symbol - mod(counted, blocks_per_symbol)
    max(1, min(requested, remaining))
end

function _coherent_integration_periods(core::LoopCore, bank::ChannelBank, ch::Int)
    blocks = _coherent_integration_blocks(core, bank, ch)
    _allows_partial_primary_records(core, bank.signal) || return Float64(blocks)
    min(Float64(blocks), _max_record_periods(core, bank.signal))
end

@inline function _record_is_emittable(core::LoopCore, signal, ch::Int)
    T = core.channels
    T.partial_samples[ch] > 0 || return false
    _allows_partial_primary_records(core, signal) && return true
    phase = T.block_phase[ch]
    isnan(phase) || phase == 0.0
end

# Add one record to the channel's open accumulation and, once it spans the
# coherent integration length, hand the sum to the loop as a single record.
function _accumulate!(
    core::LoopCore,
    bank::ChannelBank,
    ch::Int,
    correlator,
    record::DeviceRecord,
    span::RecordBlockSpan,
)
    T = core.channels
    signal = bank.signal
    tolerance = _block_boundary_tolerance(core, ch, signal)
    target = _coherent_integration_periods(core, bank, ch)
    partial_primary = _allows_partial_primary_records(core, signal)
    if T.partial_samples[ch] > 0 &&
       !partial_primary &&
       T.partial_wraps[ch] >= max(1, floor(Int, target + tolerance)) &&
       _record_is_emittable(core, signal, ch)
        _emit_partial!(core, bank, ch)
        target = _coherent_integration_periods(core, bank, ch)
    end
    if T.partial_samples[ch] == 0
        bank.partial[ch] = correlator
        T.partial_samples[ch] = record.integrated_samples
        T.partial_periods[ch] = span.periods
        T.partial_wraps[ch] = span.wraps
    else
        bank.partial[ch] = _add_accumulators(bank.partial[ch], correlator)
        T.partial_samples[ch] += record.integrated_samples
        T.partial_periods[ch] += span.periods
        T.partial_wraps[ch] += span.wraps
    end
    T.partial_end[ch] = record.sample_index
    long_enough =
        partial_primary ? T.partial_periods[ch] >= target - tolerance :
        T.partial_wraps[ch] >= max(1, floor(Int, target + tolerance))
    if long_enough && _record_is_emittable(core, signal, ch)
        _emit_partial!(core, bank, ch)
    elseif T.partial_periods[ch] >= target + 1
        # A whole code period past the target and no boundary to cut on: the
        # device's dump grid does not divide its code period. Hand over what
        # there is rather than accumulate for ever.
        _emit_partial!(core, bank, ch)
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
    nothing
end

# ── One record in ────────────────────────────────────────────────────────────

# Pool one noise-reference record: every meaningful tap is an independent look
# at the floor, and `Σ|b|²` over them is what the density is measured from. The
# taps are brought onto the satellites' scale first (the replica and code
# amplitudes the arm declared, squared): a density left at the device's own
# amplitude would be off every C/N₀ by that factor squared.
function _pool_noise_record!(core::LoopCore, ch::Int, band::Int, record::DeviceRecord)
    b = core.bands[band]
    n = Int(record.num_taps) * max(1, Int(record.num_ants))
    power = 0.0
    @inbounds for i = 1:min(n, MAX_RECORD_TAPS)
        power += abs2(record.taps[i])
    end
    scale = core.channels.scale[ch]
    b.noise_power += power / (scale * scale)
    b.noise_looks += Int(record.num_taps)
    b.noise_samples_per_look = Int(record.integrated_samples)
    nothing
end

function _ingest_in_bank!(bank::ChannelBank, core::LoopCore, ch::Int, record::DeviceRecord)
    T = core.channels
    if Int(record.num_taps) != get_num_accumulators(bank.template)
        core.tap_layout_mismatches += 1
        return nothing
    end
    span = _record_block_span(core, ch, bank.signal, record)
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
holds another PRN or its integration began before the confirmed arm; pool it
into its band's noise reference when the channel is one; otherwise fold it into
the channel's coherent accumulation, stepping the loop whenever a record is
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
    if start == typemax(Int64) || record.sample_index - record.integrated_samples < start
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
        _pool_noise_record!(core, ch, T.band[ch], record)
        return nothing
    end
    with_bank(_ingest_in_bank!, core, ch, core, ch, record)
    nothing
end
