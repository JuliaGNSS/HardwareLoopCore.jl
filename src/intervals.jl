# ─────────────────────────────────────────────────────────────────────────────
# Closing an NCO update interval: emitting a completed predetection
# integration into the loop, closing intervals, scheduling and committing NCO commands, keeping the absolute
# code phase, and publishing everything the receiver needs to mirror the
# channel.
# ─────────────────────────────────────────────────────────────────────────────

# The device sample the NCO commands computed in this pass are predicted to take
# effect at, on the reference counter: a command is written before the pass
# ends, so it is the counter read at the start of the pass plus the configured
# NCO command delay — never before the newest record seen. `commit_nco_commands!` confirms
# each command at the sample it really took effect at.
@inline function _predict_application_sample(core::LoopCore, boundary::Int64, now_reference::Int64)
    max(now_reference, core.latest_sample_index) + core.config.nco_command_delay_samples
end

# The carrier-phase offset (cycles) of the signal whose loop steers the
# channel's carrier: a passenger's bit-synchronizer prompt is de-rotated against
# its driver's (the data half of a pilot/data pair may sit in quadrature to the
# pilot).
@inline function _driver_carrier_phase_offset(core::LoopCore, ch::Int)
    T = core.channels
    driver = T.signal_index[ch] >= 2 ? T.driver_channel[ch] : 0
    (driver == 0 || !T.armed[driver]) && return T.carrier_phase_offset[ch]
    T.carrier_phase_offset[driver]
end

# Where a band's counter starts on the time grid every satellite shares, in the
# band's samples. Band counters are taken to share an origin (`_to_reference`
# scales them with no offset), so this is 0; a device whose bands start apart
# would report the offset here.
@inline _band_sample_offset(::LoopCore, ::Int) = 0

# The replica's code phase (chips) at `sample_end`, for the integration handed
# to the loop: the device's own report of the integration's last record where it
# gives one, else the core's absolute code phase advanced to `sample_end`. NaN
# when neither is known — before the first device report, which a device that
# never reports one leaves on integrations cut on the code period grid, where a
# tracking loop reads NaN as a phase on the code period boundary.
function _integration_code_phase(core::LoopCore, signal, ch::Int, sample_end::Int64)
    T = core.channels
    reported = T.partial_code_phase[ch]
    isnan(reported) || return reported
    reference = T.phase_ref_sample[ch]
    reference == typemin(Int64) && return NaN
    T.code_phase[ch] + (sample_end - reference) * _code_frequency_hz(core, ch, signal) / T.sampling_frequency[ch]
end

# The channel whose slot holds the loop state of `ch`'s satellite: its own for a
# driver, its driver channel's for a passenger, 0 for none (a noise reference, or
# a passenger whose driver is not armed).
@inline function _loop_state_channel(core::LoopCore, ch::Int)
    T = core.channels
    T.signal_index[ch] == 1 && return ch
    T.signal_index[ch] >= 2 || return 0
    driver = T.driver_channel[ch]
    driver != 0 && T.armed[driver] ? driver : 0
end

# Whether the secondary code is off the integration about to be processed. The
# device replicates the primary code only and the core wipes the secondary chip
# once sync stands, so an integration before that is primary-only, as
# TrackingLoops takes a pre-sync integration of a dataless signal to be. A
# device that wipes the secondary code itself (`SECONDARY_CODE_DEVICE_WIPES`, reserved)
# gets TrackingLoops' default: wiped once synced.
@inline _is_secondary_code_wiped(core::LoopCore, ch::Int, state) =
    core.channels.secondary_wipe[ch] ? _is_secondary_code_removed(core, ch) :
    is_bit_or_secondary_code_synced(state)

# Hand the channel's open predetection integration to the loop and start a
# fresh one. This is where the tracking loop steps, the events are published and
# the NCO command to be scheduled is refreshed. A no-op when nothing is
# accumulated.
function _emit_integration!(core::LoopCore, bank::ChannelBank, ch::Int)
    T = core.channels
    T.partial_samples[ch] == 0 && return nothing
    signal = bank.signal
    band = core.bands[T.band[ch]]
    samples = T.partial_samples[ch]
    sample_end = T.partial_end[ch]
    correlator_bank = _scale_correlator_outputs(bank.partial[ch], T.scale[ch])
    output = CorrelatorBankOutput(
        correlator_bank, Int(samples), Int(sample_end), _integration_code_phase(core, signal, ch, sample_end))
    fs = T.sampling_frequency[ch] * Hz
    offset = _band_sample_offset(core, T.band[ch])
    noise_density, noise_density_ready = TrackingLoops.noise_density_and_ready(band.noise_estimator)
    state, integration = process_correlator_bank_output!(
        bank.states[ch],
        signal,
        T.prn[ch],
        output,
        fs,
        noise_density,
        noise_density_ready,
        _driver_carrier_phase_offset(core, ch);
        secondary_code_wiped = _is_secondary_code_wiped(core, ch, bank.states[ch]),
        sample_index_offset = offset,
    )
    bank.states[ch] = state
    # The loop: every integration of the satellite, the driver's and its
    # passengers', steps the satellite's one loop state — the one in its driver
    # channel's slot — unless this interval is being closed with the loop open
    # (a stale backlog), in which case the bits and the C/N₀ are kept current and
    # the filter is left alone. A vector loop is stepped regardless: its
    # navigation engine drops a satellite that goes two navigation cycles
    # without an integration and falls back once it starves. The integration's
    # soft bits are a view of the state's, so it is stepped before they are
    # published and drained below. A passenger's integration returns the
    # command in force; the satellite's command stays its driver's, which the
    # passenger follows.
    flags = UInt32(0)
    noise_density_ready && (flags |= HardwareLoopProtocol.INTEGRATION_HAS_CN0)
    _is_secondary_code_removed(core, ch) && (flags |= HardwareLoopProtocol.INTEGRATION_SECONDARY_CODE_WIPED)
    T.partial_first[ch] && (flags |= HardwareLoopProtocol.INTEGRATION_FIRST_AFTER_ARM)
    timeline = T.timeline[ch]
    integration_start = Int64(sample_end) - Int64(samples)
    mean_nco_carrier, mean_nco_code = calc_mean_nco_command(timeline, integration_start, Int64(sample_end))
    owner = _loop_state_channel(core, ch)
    if owner != 0 && (!core.loop_open || _is_vector(core))
        application_sample_index = _to_band(core, T.band[ch], core.application_sample_index) + offset
        loop_state, carrier, code =
            step_loop!(core.tracking_loop, T.loop_state[owner], integration, timeline, application_sample_index)
        T.loop_state[owner] = loop_state
        if owner == ch
            T.carrier_doppler[ch] = ustrip(Hz, uconvert(Hz, carrier))
            T.code_doppler[ch] = ustrip(Hz, uconvert(Hz, code))
            T.command_dirty[ch] = true
        end
    end
    # Publish the integration, then any bits it completed.
    ring = event_ring(core.segment, ch)
    tag = EventTag(
        HardwareLoopProtocol.EVENT_INTEGRATION,
        ch,
        sample_end;
        band = T.band[ch],
        prn = T.prn[ch],
        signal_index = T.signal_index[ch],
        num_correlators = get_num_correlators(bank.template),
    )
    cn0 = noise_density_ready ? ustrip(Hz, Unitful.linear(estimate_cn0(state, samples / fs))) : 0.0
    publish!(
        ring,
        tag,
        IntegrationEvent(
            get_prompt(integration.beamformed_correlator_bank),
            Int64(samples),
            Int32(integration.num_code_periods),
            flags,
            cn0,
            mean_nco_carrier,
            mean_nco_code,
        ),
    )
    core.events_published += 1
    if core.config.publish_correlator_outputs || T.want_correlator_outputs[ch]
        publish!(
            ring,
            EventTag(HardwareLoopProtocol.EVENT_CORRELATOR_OUTPUTS, ch, sample_end; band = T.band[ch], prn = T.prn[ch],
                     signal_index = T.signal_index[ch], num_correlators = get_num_correlators(bank.template)),
            CorrelatorOutputsEvent(_pack_correlator_outputs(correlator_bank), Int64(samples)),
        )
        core.events_published += 1
    end
    @inbounds for soft_bit in integration.new_soft_bits
        T.bit_index[ch] += 1
        publish!(
            ring,
            EventTag(HardwareLoopProtocol.EVENT_BIT, ch, sample_end; band = T.band[ch], prn = T.prn[ch],
                     signal_index = T.signal_index[ch]),
            BitEvent(soft_bit, integration.bit_polarity, T.bit_index[ch]),
        )
        core.events_published += 1
    end
    # The bits have been published; the vector is drained so it never grows.
    drain_soft_bits!(state)
    # A post-sync integration that overshot the bit boundary dropped sync; say so.
    if integration.sync_change == TrackingLoops.SYNC_LOST
        publish!(
            ring,
            EventTag(HardwareLoopProtocol.EVENT_STATUS, ch, sample_end; band = T.band[ch], prn = T.prn[ch]),
            StatusEvent(HardwareLoopProtocol.STATUS_SYNC_RESET, HardwareLoopProtocol.REJECT_NONE, sample_end, 0),
        )
        core.events_published += 1
        T.bit_edge_known[ch] = false
        _forget_secondary_code_phase!(core, ch)
    end
    T.partial_samples[ch] = 0
    T.partial_periods[ch] = 0.0
    T.partial_wraps[ch] = 0
    T.partial_end[ch] = typemin(Int64)
    T.partial_first[ch] = false
    T.partial_code_phase[ch] = NaN
    nothing
end

# The correlator bank's outputs as the fixed tuple a `CorrelatorOutputsEvent` carries.
@inline function _pack_correlator_outputs(correlator_bank::AbstractCorrelatorBank{1})
    outputs = get_correlator_outputs(correlator_bank)
    ntuple(i -> i <= length(outputs) ? ComplexF64(outputs[i]) : complex(0.0, 0.0), Val(MAX_RECORD_CORRELATOR_OUTPUTS))
end
@inline function _pack_correlator_outputs(correlator_bank::AbstractCorrelatorBank{M}) where {M}
    outputs = get_correlator_outputs(correlator_bank)
    N = length(outputs)
    ntuple(Val(MAX_RECORD_CORRELATOR_OUTPUTS)) do i
        a, k = divrem(i - 1, N)
        (a < M && k < N) ? ComplexF64(outputs[k+1][a+1]) : complex(0.0, 0.0)
    end
end

# A lost record cut the bit clock: restart the bit synchronizer. Runs after the
# interval's records so the records up to the hole were credited to the
# synchronizer that was counting them.
function _restart_lost_bit_clock!(bank::ChannelBank, core::LoopCore, ch::Int)
    T = core.channels
    bank.states[ch] = reset_sync!(bank.states[ch])
    T.bit_edge_known[ch] = false
    T.bit_index[ch] = 0
    _forget_secondary_code_phase!(core, ch)
    publish!(
        event_ring(core.segment, ch),
        EventTag(HardwareLoopProtocol.EVENT_STATUS, ch, T.last_record_end[ch]; band = T.band[ch], prn = T.prn[ch]),
        StatusEvent(HardwareLoopProtocol.STATUS_SYNC_RESET, HardwareLoopProtocol.REJECT_NONE, T.last_record_end[ch], 0),
    )
    core.events_published += 1
    nothing
end

# ── Absolute code phase ──────────────────────────────────────────────────────

# Advance the driver channel's absolute code phase to the interval boundary,
# absorbing the replica code phase the device last reported: dead-reckon on
# the channel's band counter, take the wrapped difference to the reported
# phase, extrapolate the short hop to the boundary.
function _advance_code_phase!(bank::ChannelBank, core::LoopCore, ch::Int, interval_boundary::Int64)
    T = core.channels
    signal = bank.signal
    code_length = get_code_length(signal)
    chips_per_sample = _code_frequency_hz(core, ch, signal) / T.sampling_frequency[ch]
    boundary = _to_band(core, T.band[ch], interval_boundary)
    reference = T.phase_ref_sample[ch]
    reported = T.reported_code_phase_sample[ch]
    code_phase = T.code_phase[ch]
    if reported != typemin(Int64)
        predicted = reference == typemin(Int64) ? code_phase : code_phase + (reported - reference) * chips_per_sample
        correction = rem(T.reported_code_phase[ch] - mod(predicted, code_length), code_length, RoundNearest)
        code_phase = predicted + correction + (boundary - reported) * chips_per_sample
        T.reported_code_phase_sample[ch] = typemin(Int64)
        T.reported_code_phase[ch] = NaN
    elseif reference != typemin(Int64)
        code_phase += (boundary - reference) * chips_per_sample
    else
        return nothing
    end
    state = bank.states[ch]
    # Post-sync the phase wraps at the data bit for a data signal, at the
    # secondary code for a pilot; before that at the primary code.
    wrap = is_bit_or_secondary_code_synced(state) ? _post_sync_code_length(signal) : code_length
    T.code_phase[ch] = mod(code_phase, wrap)
    T.phase_ref_sample[ch] = boundary
    nothing
end

@inline _post_sync_code_length(signal::AbstractGNSSSignal) =
    get_code_length(signal) * max(get_secondary_code_length(signal), get_code_periods_per_bit(signal))

# Tie a newly synchronised data signal's integer code period count to its bit
# synchronizer, once per assignment, after the interval's records have been ingested.
function _count_code_phase_from_bit_edge!(bank::ChannelBank, core::LoopCore, ch::Int, interval_boundary::Int64)
    T = core.channels
    T.bit_edge_known[ch] && return nothing
    signal = bank.signal
    get_secondary_code_length(signal) == 1 || return nothing
    iszero(get_bit_rate(signal)) && return nothing
    state = bank.states[ch]
    is_bit_or_secondary_code_synced(state) || return nothing
    boundary = _to_band(core, T.band[ch], interval_boundary)
    T.phase_ref_sample[ch] == boundary || return nothing
    last = T.last_record_end[ch]
    last == typemin(Int64) && return nothing
    primary = get_code_length(signal)
    rate = _code_frequency_hz(core, ch, signal) / T.sampling_frequency[ch]
    elapsed = (boundary - last) * rate
    residual = rem(T.code_phase[ch] - elapsed, primary, RoundNearest)
    T.code_phase[ch] =
        get_num_code_periods_into_bit(state) * primary + residual + elapsed
    T.bit_edge_known[ch] = true
    nothing
end

# ── Publishing the loop state ────────────────────────────────────────────────

function _publish_loop_state!(bank::ChannelBank, core::LoopCore, ch::Int, interval_boundary::Int64)
    T = core.channels
    state = bank.states[ch]
    band = core.bands[T.band[ch]]
    fs = T.sampling_frequency[ch] * Hz
    flags = UInt8(0)
    is_bit_or_secondary_code_synced(state) && (flags |= HardwareLoopProtocol.STATE_BIT_OR_SECONDARY_CODE_SYNCED)
    T.bit_edge_known[ch] && (flags |= HardwareLoopProtocol.STATE_BIT_EDGE_KNOWN)
    T.phase_ref_sample[ch] != typemin(Int64) && (flags |= HardwareLoopProtocol.STATE_CODE_PHASE_KNOWN)
    core.loop_open && (flags |= HardwareLoopProtocol.STATE_LOOP_OPEN)
    boundary = _to_band(core, T.band[ch], interval_boundary)
    nco_carrier, nco_code = get_nco_command(T.timeline[ch], boundary)
    cn0 = last(TrackingLoops.noise_density_and_ready(band.noise_estimator)) ?
        ustrip(Hz, Unitful.linear(estimate_cn0(state, get_last_num_code_periods(state) * _code_period_seconds(bank.signal)))) : 0.0
    ev = LoopStateEvent(
        T.carrier_doppler[ch],
        T.code_doppler[ch],
        T.code_phase[ch],
        T.carrier_phase[ch],
        cn0,
        nco_carrier,
        nco_code,
        _to_band(core, T.band[ch], core.application_sample_index),
        Int32(get_num_code_periods_into_bit(state)),
        Int16(get_secondary_code_phase(state)),
        get_bit_polarity(state),
        flags,
    )
    tag = EventTag(HardwareLoopProtocol.EVENT_LOOP_STATE, ch, boundary; band = T.band[ch], prn = T.prn[ch],
                   signal_index = T.signal_index[ch])
    publish!(event_ring(core.segment, ch), tag, ev)
    write_snapshot!(snapshot_slot(core.segment, ch), tag, ev)
    core.events_published += 1
    nothing
end

# ── NCO commands ─────────────────────────────────────────────────────────────

# Schedule the interval's NCO command for every driver channel whose loop stepped,
# at the interval's application sample on the channel's band counter, and enter it
# in the timeline. Passenger channels share their satellite's command: the
# receiver's arm carries the driver channel they follow. A command the timeline
# has no room for is not written.
function _schedule_nco_commands!(core::LoopCore)
    T = core.channels
    for ch = 1:core.num_channels
        T.armed[ch] && T.confirmed[ch] || continue
        T.signal_index[ch] == 0 && continue
        T.command_dirty[ch] || continue
        T.command_dirty[ch] = false
        source = T.signal_index[ch] == 1 ? ch : T.driver_channel[ch]
        source == 0 && continue
        carrier = T.carrier_doppler[source]
        code = T.code_doppler[source]
        application_sample_index = _to_band(core, T.band[ch], core.application_sample_index)
        if !(isfinite(carrier) && isfinite(code)) ||
           !schedule_command!(T.timeline[ch], application_sample_index, carrier, code)
            core.nco_commands_rejected += 1
            continue
        end
        T.pending_command_sample[ch] = application_sample_index
        T.pending_carrier[ch] = carrier
        T.pending_code[ch] = code
    end
    nothing
end

# A passenger follows its driver's Dopplers.
function _propagate_passenger_commands!(core::LoopCore)
    T = core.channels
    for ch = 1:core.num_channels
        T.armed[ch] && T.signal_index[ch] >= 2 || continue
        source = T.driver_channel[ch]
        (source == 0 || !T.armed[source]) && continue
        if T.command_dirty[source]
            T.carrier_doppler[ch] = T.carrier_doppler[source]
            T.code_doppler[ch] = T.code_doppler[source]
            T.command_dirty[ch] = true
        end
    end
    nothing
end

"""
    commit_nco_commands!(core) -> Int

Write every NCO command the closed intervals scheduled, right now, and return how many were
committed. The device applies a command on the sample after the write, so the
sample counter read straight after it is where the command took effect; the
channel's timeline confirms it at that sample, and a command taking effect
more than an interval past its prediction is counted late. A command the device
refuses is cancelled from the timeline.
"""
function commit_nco_commands!(core::LoopCore)
    T = core.channels
    committed = 0
    for ch = 1:core.num_channels
        T.armed[ch] || continue
        due = T.pending_command_sample[ch]
        due == typemin(Int64) && continue
        T.pending_command_sample[ch] = typemin(Int64)
        if write_nco_command!(core.driver, ch, T.pending_carrier[ch], T.pending_code[ch])
            committed += 1
            core.nco_commands_committed += 1
            actual = sample_count(core.driver, T.band[ch])
            confirm_command!(T.timeline[ch], due, actual)
            actual - due > core.config.nco_update_interval_samples && (core.nco_commands_late += 1)
        else
            cancel_command!(T.timeline[ch], due)
            core.nco_commands_rejected += 1
        end
    end
    committed
end

# Advance each channel's timeline over the commands its ingested records ran on.
function _advance_timelines!(core::LoopCore)
    T = core.channels
    for ch = 1:core.num_channels
        T.armed[ch] || continue
        last_end = T.last_record_end[ch]
        last_end == typemin(Int64) && continue
        advance_timeline!(T.timeline[ch], last_end - T.last_record_samples[ch])
    end
    nothing
end

# ── The noise reference ──────────────────────────────────────────────────────

# Count the intervals the band's noise reference has sat on its noise probe; the records
# themselves went into the noise window as they arrived.
function _close_noise_interval!(core::LoopCore, band_index::Int)
    core.bands[band_index].noise_intervals_since_rearm += 1
    nothing
end

# ── The timebase ─────────────────────────────────────────────────────────────

# A record's place on the receiver timebase: strobes are stated on the
# reference band already.
@inline _timebase_sample(core::LoopCore, record::DeviceRecord) =
    is_strobe(record) ? record.sample_index : _to_reference(core, Int(record.band), record.sample_index)

# Whether a record's sample index may move the timebase. One record cannot
# move it further than `max_timebase_advance`; a jump that large has to be
# corroborated by a second record.
function _is_plausible_index!(core::LoopCore, sample_index::Int64)
    if core.latest_sample_index == typemin(Int64) ||
       sample_index - core.latest_sample_index <= core.config.max_timebase_advance
        core.implausible_index_candidate = typemin(Int64)
        return true
    end
    if core.implausible_index_candidate != typemin(Int64) &&
       abs(sample_index - core.implausible_index_candidate) <= core.config.max_timebase_advance
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
buffer and advance the timebase. Returns how many were taken.
"""
function take_records!(core::LoopCore)
    empty!(core.incoming)
    n = read_records!(core.driver, core.incoming)
    taken = 0
    pending = core.pending
    for record in core.incoming
        timebase_sample = _timebase_sample(core, record)
        _is_plausible_index!(core, timebase_sample) || continue
        if length(pending) >= core.max_pending
            core.dropped_records += 1
            continue
        end
        push!(pending, record)
        core.latest_sample_index = max(core.latest_sample_index, timebase_sample)
        taken += 1
    end
    if core.next_interval_boundary == typemin(Int64) && !isempty(pending)
        first_index = typemax(Int64)
        for record in pending
            first_index = min(first_index, _timebase_sample(core, record))
        end
        core.next_interval_boundary = first_index + core.config.nco_update_interval_samples
    end
    taken
end

"""
    close_intervals!(core, now_reference) -> Int

Close every NCO update interval whose records are all in — every record before the boundary is
ingested, each channel's completed predetection integrations step the loop,
the code phases are advanced to the boundary, the noise references close their
interval, the loop states are published and the NCO commands are scheduled — and
return how many.
`now_reference` is the device counter on the reference band at the start of
this pass; an interval more than `max_backlog_nco_updates` NCO updates behind it is closed
with the loop open.
"""
function close_intervals!(core::LoopCore, now_reference::Int64)
    core.next_interval_boundary == typemin(Int64) && return 0
    interval = core.config.nco_update_interval_samples
    closed = 0
    while core.latest_sample_index >= core.next_interval_boundary
        boundary = core.next_interval_boundary
        core.application_sample_index = _predict_application_sample(core, boundary, now_reference)
        core.loop_open = now_reference - boundary > core.config.max_backlog_nco_updates * interval
        core.loop_open && (core.open_loop_intervals += 1)
        # Ingest, in arrival order, every pending record that ended before the
        # boundary; keep the rest for the next interval.
        pending = core.pending
        keep = 0
        @inbounds for i in eachindex(pending)
            record = pending[i]
            if _timebase_sample(core, record) >= boundary
                keep += 1
                pending[keep] = record
                continue
            end
            is_strobe(record) && continue
            ingest_record!(core, record)
        end
        resize!(pending, keep)
        # Per channel: phase bookkeeping, then the
        # restarts, the bit edge and secondary code phase, and the loop state.
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
                with_bank(_count_code_phase_from_bit_edge!, core, ch, core, ch, boundary)
            end
            with_bank((bank, core, ch) -> _seed_secondary_code_phase!(core, bank, ch), core, ch, core, ch)
            with_bank(_publish_loop_state!, core, ch, core, ch, boundary)
        end
        for band_index in eachindex(core.bands)
            _close_noise_interval!(core, band_index)
        end
        _propagate_passenger_commands!(core)
        _schedule_nco_commands!(core)
        _advance_timelines!(core)
        _publish_navigation!(core, core.tracking_loop)
        core.next_interval_boundary = boundary + interval
        core.intervals_closed += 1
        closed += 1
    end
    closed
end
