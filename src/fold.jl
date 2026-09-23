# ─────────────────────────────────────────────────────────────────────────────
# The epoch fold: emitting a completed record into the loop, closing epochs,
# scheduling and committing NCO words, keeping the absolute code phase, and
# publishing everything the receiver needs to mirror the channel.
# ─────────────────────────────────────────────────────────────────────────────

# The device sample the words computed in this pass are predicted to land at,
# on the reference counter: the word is written before the pass ends, so it is
# the counter read at the start of the pass plus the configured lead — never
# before the newest record seen. `commit_words!` corrects the timeline to the
# sample the word really landed at.
@inline function _landing_sample(core::LoopCore, boundary::Int64, now_reference::Int64)
    max(now_reference, core.latest_sample_index) + core.config.commit_lead_samples
end

# Hand the channel's part-accumulated record to the loop and start a fresh one.
# This is where the estimator steps, the events are published and the word to
# be scheduled is refreshed. A no-op when nothing is accumulated.
function _emit_partial!(core::LoopCore, bank::ChannelBank, ch::Int)
    T = core.channels
    T.partial_samples[ch] == 0 && return nothing
    signal = bank.signal
    band = core.bands[T.band[ch]]
    samples = T.partial_samples[ch]
    sample_end = T.partial_end[ch]
    correlator = _scale_accumulators(bank.partial[ch], T.scale[ch])
    output = CorrelatorOutput(correlator, Int(samples), Int(sample_end))
    fs = T.sampling_freq[ch] * Hz
    state = bank.states[ch]
    previous_prompt = state.last_filtered_prompt
    blocks_before = state.bit_buffer.prompt_accumulator_integrated_code_blocks
    bits_before = length(get_soft_bits(state))
    found_before = has_bit_or_secondary_code_been_found(state)
    noise_density = band.noise_density / Hz
    state, prompt, filtered, integrated_code_blocks = apply_record(
        state,
        signal,
        T.prn[ch],
        output,
        fs,
        noise_density,
        band.noise_density_ready,
    )
    bank.states[ch] = state
    # The loop: every record the driver component completes, unless this epoch
    # is being folded observation-only (a stale backlog), in which case the
    # bits and the C/N₀ are kept current and the filter is left alone.
    flags = UInt32(0)
    band.noise_density_ready && (flags |= HardwareLoopProtocol.RECORD_HAS_CN0)
    _is_secondary_code_removed(core, ch) && (flags |= HardwareLoopProtocol.RECORD_OVERLAY_WIPED)
    T.partial_first[ch] && (flags |= HardwareLoopProtocol.RECORD_FIRST_AFTER_ARM)
    timeline = T.timeline[ch]
    record_start = Int64(sample_end) - Int64(samples)
    applied_carrier, applied_code = mean_nco_word(timeline, record_start, Int64(sample_end))
    if T.signal_index[ch] == 1 && !core.observation_only
        record = LoopRecord(signal, filtered, previous_prompt, output, integrated_code_blocks, fs)
        landing = _to_band(core, T.band[ch], core.current_landing)
        est, carrier, code = step_loop(core.estimator, T.estimator[ch], record, timeline, landing)
        T.estimator[ch] = est
        T.carrier_doppler[ch] = ustrip(Hz, uconvert(Hz, carrier))
        T.code_doppler[ch] = ustrip(Hz, uconvert(Hz, code))
        T.word_dirty[ch] = true
    end
    # Publish the record, then any bits it completed.
    ring = event_ring(core.segment, ch)
    tag = EventTag(
        HardwareLoopProtocol.EVENT_RECORD,
        ch,
        sample_end;
        band = T.band[ch],
        prn = T.prn[ch],
        signal_index = T.signal_index[ch],
        num_taps = get_num_accumulators(bank.template),
    )
    cn0 = band.noise_density_ready ? ustrip(Hz, Unitful.linear(estimate_cn0(state, samples / fs))) : 0.0
    publish!(
        ring,
        tag,
        RecordEvent(prompt, Int64(samples), Int32(integrated_code_blocks), flags, cn0, applied_carrier, applied_code),
    )
    core.events_published += 1
    if core.config.publish_taps || T.want_taps[ch]
        taps = get_accumulators(filtered)
        publish!(
            ring,
            EventTag(HardwareLoopProtocol.EVENT_TAPS, ch, sample_end; band = T.band[ch], prn = T.prn[ch],
                     signal_index = T.signal_index[ch], num_taps = get_num_accumulators(bank.template)),
            TapsEvent(_pack_correlator_taps(correlator), Int64(samples)),
        )
        core.events_published += 1
    end
    soft_bits = get_soft_bits(state)
    @inbounds for i = (bits_before+1):length(soft_bits)
        T.bit_index[ch] += 1
        publish!(
            ring,
            EventTag(HardwareLoopProtocol.EVENT_BIT, ch, sample_end; band = T.band[ch], prn = T.prn[ch],
                     signal_index = T.signal_index[ch]),
            BitEvent(soft_bits[i], state.bit_buffer.polarity, T.bit_index[ch]),
        )
        core.events_published += 1
    end
    # The bits have been published; the vector is drained so it never grows.
    isempty(soft_bits) || empty!(soft_bits)
    # A post-sync record that overshot the bit boundary dropped sync; say so.
    if found_before && !has_bit_or_secondary_code_been_found(state)
        publish!(
            ring,
            EventTag(HardwareLoopProtocol.EVENT_STATUS, ch, sample_end; band = T.band[ch], prn = T.prn[ch]),
            StatusEvent(HardwareLoopProtocol.STATUS_BIT_CLOCK_RESTART, HardwareLoopProtocol.REJECT_NONE, sample_end, 0),
        )
        core.events_published += 1
        T.bit_phase_anchored[ch] = false
        _forget_secondary_phase!(core, ch)
    end
    T.partial_samples[ch] = 0
    T.partial_periods[ch] = 0.0
    T.partial_wraps[ch] = 0
    T.partial_end[ch] = typemin(Int64)
    T.partial_first[ch] = false
    nothing
end

# The correlator's taps as the fixed tuple a `TapsEvent` carries.
@inline function _pack_correlator_taps(correlator::AbstractCorrelator{1})
    acc = get_accumulators(correlator)
    ntuple(i -> i <= length(acc) ? ComplexF64(acc[i]) : complex(0.0, 0.0), Val(MAX_RECORD_TAPS))
end
@inline function _pack_correlator_taps(correlator::AbstractCorrelator{M}) where {M}
    acc = get_accumulators(correlator)
    N = length(acc)
    ntuple(Val(MAX_RECORD_TAPS)) do i
        a, tap = divrem(i - 1, N)
        (a < M && tap < N) ? ComplexF64(acc[tap+1][a+1]) : complex(0.0, 0.0)
    end
end

# A lost record cut the bit clock: rebuild it from the signal. Runs after the
# epoch's records so the records up to the hole were credited to the buffer
# that was counting them.
function _restart_lost_bit_clock!(bank::ChannelBank, core::LoopCore, ch::Int)
    T = core.channels
    bank.states[ch] = restart_bit_clock(bank.states[ch])
    T.bit_phase_anchored[ch] = false
    T.bit_index[ch] = 0
    _forget_secondary_phase!(core, ch)
    publish!(
        event_ring(core.segment, ch),
        EventTag(HardwareLoopProtocol.EVENT_STATUS, ch, T.last_record_end[ch]; band = T.band[ch], prn = T.prn[ch]),
        StatusEvent(HardwareLoopProtocol.STATUS_BIT_CLOCK_RESTART, HardwareLoopProtocol.REJECT_NONE, T.last_record_end[ch], 0),
    )
    core.events_published += 1
    nothing
end

# ── Absolute code phase ──────────────────────────────────────────────────────

# Advance the driver channel's absolute code phase to the fold boundary,
# absorbing this epoch's replica anchor, exactly as the link did: dead-reckon on
# the channel's band counter, take the wrapped difference to the reported
# replica phase, extrapolate the short hop to the boundary.
function _advance_code_phase!(bank::ChannelBank, core::LoopCore, ch::Int, epoch_boundary::Int64)
    T = core.channels
    signal = bank.signal
    code_length = get_code_length(signal)
    chips_per_sample = _code_rate_hz(core, ch, signal) / T.sampling_freq[ch]
    boundary = _to_band(core, T.band[ch], epoch_boundary)
    reference = T.phase_ref_sample[ch]
    anchor = T.anchor_sample[ch]
    code_phase = T.code_phase[ch]
    if anchor != typemin(Int64)
        predicted = reference == typemin(Int64) ? code_phase : code_phase + (anchor - reference) * chips_per_sample
        correction = rem(T.anchor_code_phase[ch] - mod(predicted, code_length), code_length, RoundNearest)
        code_phase = predicted + correction + (boundary - anchor) * chips_per_sample
        T.anchor_sample[ch] = typemin(Int64)
        T.anchor_code_phase[ch] = NaN
    elseif reference != typemin(Int64)
        code_phase += (boundary - reference) * chips_per_sample
    else
        return nothing
    end
    state = bank.states[ch]
    # Post-sync the phase wraps at the symbol (the data bit for a data signal,
    # the secondary code for a pilot); before that at the primary code.
    wrap = has_bit_or_secondary_code_been_found(state) ? _post_sync_code_length(signal) : code_length
    T.code_phase[ch] = mod(code_phase, wrap)
    T.phase_ref_sample[ch] = boundary
    nothing
end

@inline function _post_sync_code_length(signal::AbstractGNSSSignal)
    primary = get_code_length(signal)
    secondary = get_secondary_code_length(signal)
    data_frequency = get_data_frequency(signal)
    if iszero(data_frequency)
        primary * secondary
    else
        blocks_per_bit = Int(get_code_frequency(signal) / (primary * data_frequency))
        primary * max(secondary, blocks_per_bit)
    end
end

# Tie a newly synchronised data signal's integer code-period count to its bit
# buffer, once per assignment, after the epoch's records have been folded.
function _anchor_bit_phase!(bank::ChannelBank, core::LoopCore, ch::Int, epoch_boundary::Int64)
    T = core.channels
    T.bit_phase_anchored[ch] && return nothing
    signal = bank.signal
    get_secondary_code_length(signal) == 1 || return nothing
    iszero(get_data_frequency(signal)) && return nothing
    state = bank.states[ch]
    has_bit_or_secondary_code_been_found(state) || return nothing
    boundary = _to_band(core, T.band[ch], epoch_boundary)
    T.phase_ref_sample[ch] == boundary || return nothing
    last = T.last_record_end[ch]
    last == typemin(Int64) && return nothing
    primary = get_code_length(signal)
    rate = _code_rate_hz(core, ch, signal) / T.sampling_freq[ch]
    elapsed = (boundary - last) * rate
    residual = rem(T.code_phase[ch] - elapsed, primary, RoundNearest)
    T.code_phase[ch] =
        state.bit_buffer.prompt_accumulator_integrated_code_blocks * primary + residual + elapsed
    T.bit_phase_anchored[ch] = true
    nothing
end

# ── Publishing the epoch state ───────────────────────────────────────────────

function _publish_epoch_state!(bank::ChannelBank, core::LoopCore, ch::Int, epoch_boundary::Int64)
    T = core.channels
    state = bank.states[ch]
    band = core.bands[T.band[ch]]
    fs = T.sampling_freq[ch] * Hz
    bb = state.bit_buffer
    flags = UInt8(0)
    has_bit_or_secondary_code_been_found(state) && (flags |= HardwareLoopProtocol.STATE_SYNC_FOUND)
    T.bit_phase_anchored[ch] && (flags |= HardwareLoopProtocol.STATE_BIT_PHASE_ANCHORED)
    T.phase_ref_sample[ch] != typemin(Int64) && (flags |= HardwareLoopProtocol.STATE_CODE_PHASE_ANCHORED)
    core.observation_only && (flags |= HardwareLoopProtocol.STATE_OBSERVATION_ONLY)
    boundary = _to_band(core, T.band[ch], epoch_boundary)
    nco_carrier, nco_code = nco_word_at(T.timeline[ch], boundary)
    cn0 = band.noise_density_ready ?
        ustrip(Hz, Unitful.linear(estimate_cn0(state, state.last_num_code_blocks * _code_period_seconds(bank.signal)))) : 0.0
    ev = EpochStateEvent(
        T.carrier_doppler[ch],
        T.code_doppler[ch],
        T.code_phase[ch],
        T.carrier_phase[ch],
        cn0,
        nco_carrier,
        nco_code,
        _to_band(core, T.band[ch], core.current_landing),
        Int32(bb.prompt_accumulator_integrated_code_blocks),
        Int16(bb.secondary_phase),
        bb.polarity,
        flags,
    )
    tag = EventTag(HardwareLoopProtocol.EVENT_EPOCH_STATE, ch, boundary; band = T.band[ch], prn = T.prn[ch],
                   signal_index = T.signal_index[ch])
    publish!(event_ring(core.segment, ch), tag, ev)
    write_snapshot!(snapshot_slot(core.segment, ch), tag, ev)
    core.events_published += 1
    nothing
end

# ── Words ────────────────────────────────────────────────────────────────────

# Schedule the epoch's word for every driver channel whose loop stepped, at the
# epoch's landing sample on the channel's band counter, and enter it in the
# timeline. Passenger channels share their satellite's word: the receiver's
# arm carries the driver channel they follow.
function _schedule_words!(core::LoopCore)
    T = core.channels
    for ch = 1:core.num_channels
        T.armed[ch] && T.confirmed[ch] || continue
        T.signal_index[ch] == 0 && continue
        T.word_dirty[ch] || continue
        T.word_dirty[ch] = false
        source = T.signal_index[ch] == 1 ? ch : T.driver_channel[ch]
        source == 0 && continue
        carrier = T.carrier_doppler[source]
        code = T.code_doppler[source]
        landing = _to_band(core, T.band[ch], core.current_landing)
        if !(isfinite(carrier) && isfinite(code))
            core.words_rejected += 1
            continue
        end
        T.pending_word_sample[ch] = landing
        T.pending_carrier[ch] = carrier
        T.pending_code[ch] = code
        schedule_word!(T.timeline[ch], landing, carrier, code)
    end
    nothing
end

# A passenger follows its driver's Dopplers.
function _propagate_passenger_words!(core::LoopCore)
    T = core.channels
    for ch = 1:core.num_channels
        T.armed[ch] && T.signal_index[ch] >= 2 || continue
        source = T.driver_channel[ch]
        (source == 0 || !T.armed[source]) && continue
        if T.word_dirty[source]
            T.carrier_doppler[ch] = T.carrier_doppler[source]
            T.code_doppler[ch] = T.code_doppler[source]
            T.word_dirty[ch] = true
        end
    end
    nothing
end

"""
    commit_words!(core) -> Int

Write every word the fold scheduled, right now, and return how many were
committed. The device applies a word on the sample after the write, so the
sample counter read straight after it is where the word landed; the channel's
timeline is corrected from the predicted landing to that sample, and a word
landing more than an epoch past its prediction is counted late.
"""
function commit_words!(core::LoopCore)
    T = core.channels
    committed = 0
    for ch = 1:core.num_channels
        T.armed[ch] || continue
        due = T.pending_word_sample[ch]
        due == typemin(Int64) && continue
        T.pending_word_sample[ch] = typemin(Int64)
        if write_word!(core.driver, ch, T.pending_carrier[ch], T.pending_code[ch])
            committed += 1
            core.words_committed += 1
            actual = sample_count(core.driver, T.band[ch])
            actual == due || reschedule_word!(T.timeline[ch], due, actual)
            actual - due > core.config.epoch_length && (core.words_late += 1)
        else
            core.words_rejected += 1
        end
    end
    committed
end

# Fold each channel's timeline forward over the words its folded records ran on.
function _promote_applied_words!(core::LoopCore)
    T = core.channels
    for ch = 1:core.num_channels
        T.armed[ch] || continue
        last_end = T.last_record_end[ch]
        last_end == typemin(Int64) && continue
        promote_words!(T.timeline[ch], last_end - T.last_record_samples[ch])
    end
    nothing
end

# ── The noise reference ──────────────────────────────────────────────────────

# Close this epoch's pooled looks into a density observation and slide the
# window: `N₀ = Σ|b|² / (looks · samples_per_look · fs)` (unit code amplitude;
# the reference and the satellites share the device's scale, which divides out
# of the C/N₀ ratio).
function _close_noise_epoch!(core::LoopCore, band_index::Int)
    b = core.bands[band_index]
    if b.noise_looks > 0 && b.noise_samples_per_look > 0
        density = b.noise_power / (b.noise_looks * b.noise_samples_per_look * b.entry.sampling_freq_hz)
        b.noise_window_index = mod(b.noise_window_index, NOISE_WINDOW_LENGTH) + 1
        b.noise_window[b.noise_window_index] = density
        b.noise_window_count = min(b.noise_window_count + 1, NOISE_WINDOW_LENGTH)
        total = 0.0
        @inbounds for i = 1:b.noise_window_count
            total += b.noise_window[i]
        end
        mean = total / b.noise_window_count
        b.noise_density = mean
        b.noise_density_ready = isfinite(mean) && mean > 0
    end
    b.noise_power = 0.0
    b.noise_looks = 0
    b.noise_epochs_since_rearm += 1
    nothing
end

# ── The epoch clock ──────────────────────────────────────────────────────────

# A record's place on the receiver timebase: strobes are stated on the
# reference band already.
@inline _epoch_sample(core::LoopCore, record::DeviceRecord) =
    is_strobe(record) ? record.sample_index : _to_reference(core, Int(record.band), record.sample_index)

# Whether a record's sample index may move the epoch clock. One record cannot
# move it further than `max_epoch_clock_advance`; a jump that large has to be
# corroborated by a second record.
function _is_plausible_index!(core::LoopCore, sample_index::Int64)
    if core.latest_sample_index == typemin(Int64) ||
       sample_index - core.latest_sample_index <= core.config.max_epoch_clock_advance
        core.implausible_index_candidate = typemin(Int64)
        return true
    end
    if core.implausible_index_candidate != typemin(Int64) &&
       abs(sample_index - core.implausible_index_candidate) <= core.config.max_epoch_clock_advance
        core.implausible_index_candidate = typemin(Int64)
        return true
    end
    core.implausible_index_candidate = sample_index
    core.implausible_dumps += 1
    false
end

"""
    take_records!(core) -> Int

Read every record the driver has, move the plausible ones into the pending
buffer and advance the epoch clock. Returns how many were taken.
"""
function take_records!(core::LoopCore)
    empty!(core.incoming)
    n = read_records!(core.driver, core.incoming)
    taken = 0
    pending = core.pending
    for record in core.incoming
        epoch_sample = _epoch_sample(core, record)
        _is_plausible_index!(core, epoch_sample) || continue
        if length(pending) >= core.max_pending
            core.dropped_records += 1
            continue
        end
        push!(pending, record)
        core.latest_sample_index = max(core.latest_sample_index, epoch_sample)
        taken += 1
    end
    if core.next_epoch_boundary == typemin(Int64) && !isempty(pending)
        first_index = typemax(Int64)
        for record in pending
            first_index = min(first_index, _epoch_sample(core, record))
        end
        core.next_epoch_boundary = first_index + core.config.epoch_length
    end
    taken
end

"""
    fold_closed_epochs!(core, now_reference) -> Int

Fold every epoch that has closed — every record before the boundary is
ingested, each channel's completed records step the loop, the code phases are
advanced to the boundary, the noise references close their epoch, the epoch
states are published and the words are scheduled — and return how many.
`now_reference` is the device counter on the reference band at the start of
this pass; an epoch more than `max_backlog_epochs` behind it is folded
observation-only.
"""
function fold_closed_epochs!(core::LoopCore, now_reference::Int64)
    core.next_epoch_boundary == typemin(Int64) && return 0
    epoch = core.config.epoch_length
    folds = 0
    while core.latest_sample_index >= core.next_epoch_boundary
        boundary = core.next_epoch_boundary
        core.current_landing = _landing_sample(core, boundary, now_reference)
        core.observation_only = now_reference - boundary > core.config.max_backlog_epochs * epoch
        core.observation_only && (core.skipped_epochs += 1)
        # Ingest, in arrival order, every pending record that ended before the
        # boundary; keep the rest for the next epoch.
        pending = core.pending
        keep = 0
        @inbounds for i in eachindex(pending)
            record = pending[i]
            if _epoch_sample(core, record) >= boundary
                keep += 1
                pending[keep] = record
                continue
            end
            is_strobe(record) && continue
            ingest_record!(core, record)
        end
        resize!(pending, keep)
        # Per channel, in the order the link kept: phase bookkeeping, then the
        # restarts, anchors and the epoch state.
        for ch = 1:core.num_channels
            core.channels.armed[ch] && core.channels.confirmed[ch] || continue
            core.channels.signal_index[ch] == 0 && continue
            if core.channels.signal_index[ch] == 1
                with_bank(_advance_code_phase!, core, ch, core, ch, boundary)
            end
            if core.channels.bit_clock_lost[ch]
                core.channels.bit_clock_lost[ch] = false
                with_bank(_restart_lost_bit_clock!, core, ch, core, ch)
            end
            if core.channels.signal_index[ch] == 1
                with_bank(_anchor_bit_phase!, core, ch, core, ch, boundary)
            end
            with_bank((bank, core, ch) -> _anchor_secondary_phase!(core, bank, ch), core, ch, core, ch)
            with_bank(_publish_epoch_state!, core, ch, core, ch, boundary)
        end
        for band_index in eachindex(core.bands)
            _close_noise_epoch!(core, band_index)
        end
        _propagate_passenger_words!(core)
        _schedule_words!(core)
        _promote_applied_words!(core)
        core.next_epoch_boundary = boundary + epoch
        core.epochs_folded += 1
        folds += 1
    end
    folds
end
