# ─────────────────────────────────────────────────────────────────────────────
# Commands from the receiver: arm, release, configure, query, shutdown. Each is
# acknowledged by a status event — on the channel's ring, or on channel 1's for
# a loop-wide command. The allocation *policy* stays with the receiver; the loop
# executes, refusing what the device or the configuration cannot serve.
# ─────────────────────────────────────────────────────────────────────────────

const LOOP_STATUS_CHANNEL = 1

function _publish_status!(core::LoopCore, channel::Int, sample::Int64, status::StatusEvent)
    ch = clamp(channel, 1, core.num_channels)
    T = core.channels
    publish!(
        event_ring(core.segment, ch),
        EventTag(HardwareLoopProtocol.EVENT_STATUS, ch, sample; band = T.band[ch], prn = T.prn[ch],
                 signal_index = T.signal_index[ch]),
        status,
    )
    core.events_published += 1
    nothing
end

# A command for no channel, or for one this loop does not have, is answered on
# the loop-wide channel.
_reject!(core::LoopCore, tag::CommandTag, reason) = _publish_status!(
    core,
    1 <= Int(tag.channel) <= core.num_channels ? Int(tag.channel) : LOOP_STATUS_CHANNEL,
    sample_count(core.driver, 1),
    StatusEvent(HardwareLoopProtocol.STATUS_ARM_REJECTED, reason, sample_count(core.driver, 1), tag.sequence),
)

# Which bank holds `name`'s signal, or 0. Fixed names compare by bytes, so no
# string is built.
function _bank_index_by_name(core::LoopCore, name::FixedName)
    _bank_by_name(core.banks, name, 1)
end
_bank_by_name(::Tuple{}, ::FixedName, ::Int) = 0
_bank_by_name(banks::Tuple, name::FixedName, k::Int) =
    first(banks).name == name ? k : _bank_by_name(Base.tail(banks), name, k + 1)

# The armed channel that drives the satellite `(group_key, prn)`, or 0.
function _find_driver_channel(core::LoopCore, group_key::FixedName, prn::Int, except::Int)
    T = core.channels
    for ch = 1:core.num_channels
        ch == except && continue
        T.armed[ch] && T.signal_index[ch] == 1 && T.prn[ch] == prn && T.group_key[ch] == group_key &&
            return ch
    end
    0
end

# Point every passenger of `(group_key, prn)` at `driver`.
function _link_passengers!(core::LoopCore, group_key::FixedName, prn::Int, driver::Int)
    T = core.channels
    for ch = 1:core.num_channels
        T.armed[ch] && T.signal_index[ch] >= 2 && T.prn[ch] == prn && T.group_key[ch] == group_key || continue
        T.driver_channel[ch] = driver
    end
    nothing
end

# Everything a channel forgets when it changes occupant.
function _clear_channel!(core::LoopCore, ch::Int)
    T = core.channels
    T.confirmed[ch] = false
    T.pending_command_sample[ch] = typemin(Int64)
    T.command_dirty[ch] = false
    T.last_record_end[ch] = typemin(Int64)
    T.last_record_samples[ch] = typemin(Int64)
    T.nominal_record_samples[ch] = typemin(Int64)
    T.code_period_phase[ch] = NaN
    T.primary_wraps[ch] = 0
    T.lost_record_samples[ch] = 0
    T.rearm_dead_samples[ch] = 0
    T.bit_clock_lost[ch] = false
    _discard_partial!(core, ch)
    T.partial_first[ch] = true
    _forget_secondary_code_phase!(core, ch)
    T.phase_ref_sample[ch] = typemin(Int64)
    T.anchor_sample[ch] = typemin(Int64)
    T.anchor_code_phase[ch] = NaN
    T.bit_phase_anchored[ch] = false
    T.carrier_phase[ch] = 0.0
    T.carrier_phase_sample[ch] = typemin(Int64)
    T.bit_index[ch] = 0
    T.records_folded[ch] = 0
    T.stale_records[ch] = 0
    nothing
end

# The signal-typed half of an arm: the fresh post-correlation state, the
# amplitude scale, the secondary code wipe-off decision and the driver call.
function _arm_in_bank!(bank::ChannelBank, core::LoopCore, ch::Int, cmd::ArmCommand)
    T = core.channels
    signal = bank.signal
    bank.states[ch] = reset_post_correlation!(bank.states[ch])
    bank.partial[ch] = zero(bank.template)
    # Records are scaled to the software correlator's, which still carry the
    # signal's code amplitude: post-correlation processing divides it out of a
    # satellite's, `append_noise_taps!` out of the noise reference's.
    T.scale[ch] = cmd.replica_amplitude * cmd.code_amplitude / Float64(get_code_amplitude(signal))
    T.carrier_phase_offset[ch] = Float64(get_carrier_phase_offset(signal))
    T.secondary_wipe[ch] =
        get_secondary_code_length(signal) > 1 &&
        cmd.secondary_code_mode == HardwareLoopProtocol.SECONDARY_PRIMARY_ONLY
    T.loop_state[ch] = init_loop_state(
        core.tracking_loop,
        signal,
        cmd.carrier_doppler_hz * Hz,
        cmd.code_doppler_hz * Hz,
    )
    # A noise reference is programmed with TrackingLoops' noise correlator, whose
    # taps sit far enough apart to be independent looks, whatever taps the
    # arm carries; its records are never published.
    tap_sample_shifts, num_taps = cmd.signal_index == 0 ?
        _noise_tap_shifts(core.bands[Int(cmd.band)], cmd.sampling_freq_hz, signal) :
        (cmd.tap_sample_shifts, Int(cmd.num_taps))
    spec = ArmSpec(
        signal,
        Int(cmd.prn),
        cmd.carrier_doppler_hz,
        cmd.code_doppler_hz,
        cmd.code_phase_chips,
        cmd.valid_at_sample,
        tap_sample_shifts,
        num_taps,
        Int(cmd.band),
        Int(cmd.rf_input),
        Int(cmd.device_index),
        cmd.sampling_freq_hz,
        cmd.replica_amplitude,
        cmd.code_amplitude,
    )
    arm!(core.driver, ch, spec)
end

function _handle_arm!(core::LoopCore, tag::CommandTag, cmd::ArmCommand)
    T = core.channels
    ch = Int(tag.channel)
    1 <= ch <= core.num_channels || return _reject!(core, tag, HardwareLoopProtocol.REJECT_NO_SUCH_CHANNEL)
    bank = _bank_index_by_name(core, cmd.signal)
    bank == 0 && return _reject!(core, tag, HardwareLoopProtocol.REJECT_UNSUPPORTED_SIGNAL)
    # Vector tracking drives a satellite with a signal its tracking loop ranges
    # on: a data signal, or a pilot it pairs with its data component. Any signal
    # may be a passenger; the loop decodes the one it pairs with the driver.
    if _is_vector(core)
        cmd.signal_index == 1 && !core.vector_banks[bank] &&
            return _reject!(core, tag, HardwareLoopProtocol.REJECT_UNSUPPORTED_SIGNAL)
    end
    band = Int(cmd.band)
    1 <= band <= length(core.bands) || return _reject!(core, tag, HardwareLoopProtocol.REJECT_BAD_CONFIG)
    (1 <= cmd.num_taps <= 5 && cmd.sampling_freq_hz > 0) ||
        return _reject!(core, tag, HardwareLoopProtocol.REJECT_BAD_CONFIG)
    # A noise reference may only be re-pointed by a release; a satellite channel
    # may be re-armed in place (the same or another satellite).
    if T.armed[ch] && T.signal_index[ch] == 0 && cmd.signal_index != 0
        return _reject!(core, tag, HardwareLoopProtocol.REJECT_CHANNEL_BUSY)
    end
    if T.armed[ch]
        was_noise = T.signal_index[ch] == 0
        was_noise && (core.bands[T.band[ch]].noise_channel = 0)
    end
    _clear_channel!(core, ch)
    T.bank[ch] = bank
    T.band[ch] = band
    T.prn[ch] = Int(cmd.prn)
    T.signal_index[ch] = Int(cmd.signal_index)
    T.group_key[ch] = cmd.group_key
    T.arm_sequence[ch] = tag.sequence
    T.want_taps[ch] = cmd.want_taps != 0
    T.sampling_frequency[ch] = cmd.sampling_freq_hz
    T.carrier_doppler[ch] = cmd.carrier_doppler_hz
    T.code_doppler[ch] = cmd.code_doppler_hz
    T.code_phase[ch] = cmd.code_phase_chips
    reset_timeline!(T.timeline[ch], cmd.carrier_doppler_hz, cmd.code_doppler_hz)
    outcome = with_bank(_arm_in_bank!, core, ch, core, ch, cmd)::ArmOutcome
    if !outcome.accepted
        T.armed[ch] = false
        T.bank[ch] = 0
        return _reject!(core, tag, outcome.reason)
    end
    T.armed[ch] = true
    T.confirmed[ch] = false
    if cmd.signal_index == 0
        b = core.bands[band]
        b.noise_channel = ch
        b.noise_prn = Int(cmd.prn)
        b.noise_code_phase = cmd.code_phase_chips
        b.noise_carrier_doppler = cmd.carrier_doppler_hz
        b.noise_replica_amplitude = cmd.replica_amplitude
        b.noise_code_amplitude = cmd.code_amplitude
        b.noise_epochs_since_rearm = 0
    elseif cmd.signal_index == 1
        _link_passengers!(core, cmd.group_key, Int(cmd.prn), ch)
    else
        T.driver_channel[ch] = _find_driver_channel(core, cmd.group_key, Int(cmd.prn), ch)
    end
    nothing
end

function _handle_release!(core::LoopCore, tag::CommandTag)
    T = core.channels
    ch = Int(tag.channel)
    1 <= ch <= core.num_channels || return _reject!(core, tag, HardwareLoopProtocol.REJECT_NO_SUCH_CHANNEL)
    T.armed[ch] || return _publish_status!(core, ch, sample_count(core.driver, T.band[ch]),
        StatusEvent(HardwareLoopProtocol.STATUS_COMMAND_REJECTED, HardwareLoopProtocol.REJECT_NOT_ARMED, 0, tag.sequence))
    release!(core.driver, ch)
    T.signal_index[ch] == 0 && (core.bands[T.band[ch]].noise_channel = 0)
    sample = sample_count(core.driver, T.band[ch])
    _publish_status!(core, ch, sample, StatusEvent(HardwareLoopProtocol.STATUS_RELEASED, HardwareLoopProtocol.REJECT_NONE, sample, tag.sequence))
    T.armed[ch] = false
    T.confirmed[ch] = false
    T.bank[ch] = 0
    T.signal_index[ch] = 0
    nothing
end

function _handle_configure!(core::LoopCore, tag::CommandTag, cmd::ConfigureCommand)
    cfg = core.config
    cmd.epoch_length_samples > 0 && (cfg.epoch_length = cmd.epoch_length_samples)
    cmd.commit_lead_samples > 0 && (cfg.commit_lead_samples = Int64(cmd.commit_lead_samples))
    cmd.coherent_code_blocks >= 0 && cmd.epoch_length_samples > 0 && (cfg.num_code_periods_to_integrate = Int(cmd.coherent_code_blocks))
    cmd.max_integration_time_s > 0 && (cfg.max_integration_time = cmd.max_integration_time_s)
    cmd.noise_rearm_epochs > 0 && (cfg.noise_rearm_epochs = Int(cmd.noise_rearm_epochs))
    cmd.max_backlog_epochs > 0 && (cfg.max_backlog_epochs = Int(cmd.max_backlog_epochs))
    cfg.publish_taps = (cmd.event_flags & HardwareLoopProtocol.EVENTS_TAPS) != 0
    sample = sample_count(core.driver, 1)
    _publish_status!(core, LOOP_STATUS_CHANNEL, sample,
        StatusEvent(HardwareLoopProtocol.STATUS_CONFIGURED, HardwareLoopProtocol.REJECT_NONE, sample, tag.sequence))
    nothing
end

# The full seed a receiver needs to adopt each armed channel's satellite.
function _publish_channel_states!(core::LoopCore, sequence::UInt64)
    T = core.channels
    for ch = 1:core.num_channels
        T.armed[ch] && T.signal_index[ch] >= 1 || continue
        sample = T.phase_ref_sample[ch] == typemin(Int64) ? sample_count(core.driver, T.band[ch]) : T.phase_ref_sample[ch]
        _publish_status!(core, ch, sample, StatusEvent(
            HardwareLoopProtocol.STATUS_CHANNEL_STATE,
            T.confirmed[ch] ? HardwareLoopProtocol.REJECT_NONE : HardwareLoopProtocol.REJECT_NOT_ARMED,
            sample,
            sequence,
            T.carrier_doppler[ch],
            T.code_doppler[ch],
            T.code_phase[ch],
            _bank_name(core, T.bank[ch]),
            T.group_key[ch],
        ))
    end
    nothing
end

_bank_name(core::LoopCore, index::Int) = _bank_name(core.banks, index, 1)
_bank_name(::Tuple{}, ::Int, ::Int) = FixedName()
_bank_name(banks::Tuple, index::Int, k::Int) =
    index == k ? first(banks).name : _bank_name(Base.tail(banks), index, k + 1)

"""
    handle_commands!(core) -> Int

Drain the command ring, executing and acknowledging every command. Returns how
many were handled.
"""
function handle_commands!(core::LoopCore)
    ring = command_ring(core.segment)
    handled = 0
    while true
        status, view, _ = peek!(ring, CommandTag)
        status === :empty && break
        tag = view.tag
        kind = tag.kind
        if kind == HardwareLoopProtocol.COMMAND_ARM
            cmd = payload(ArmCommand, ring, view)
            isnothing(cmd) || _handle_arm!(core, tag, cmd)
        elseif kind == HardwareLoopProtocol.COMMAND_RELEASE
            _handle_release!(core, tag)
        elseif kind == HardwareLoopProtocol.COMMAND_CONFIGURE
            cmd = payload(ConfigureCommand, ring, view)
            isnothing(cmd) || _handle_configure!(core, tag, cmd)
        elseif kind == HardwareLoopProtocol.COMMAND_QUERY_STATE
            _publish_channel_states!(core, tag.sequence)
        elseif kind == HardwareLoopProtocol.COMMAND_SHUTDOWN
            sample = sample_count(core.driver, 1)
            _publish_status!(core, LOOP_STATUS_CHANNEL, sample,
                StatusEvent(HardwareLoopProtocol.STATUS_SHUTDOWN, HardwareLoopProtocol.REJECT_NONE, sample, tag.sequence))
            core.running = false
        else
            _publish_status!(core, LOOP_STATUS_CHANNEL, sample_count(core.driver, 1),
                StatusEvent(HardwareLoopProtocol.STATUS_COMMAND_REJECTED, HardwareLoopProtocol.REJECT_UNKNOWN_COMMAND, 0, tag.sequence))
        end
        commit!(ring, view)
        handled += 1
        core.commands_handled += 1
    end
    handled
end

"""
    confirm_arms!(core)

Publish a `STATUS_ARMED` status event, stamped with the device sample the
assignment took effect at, for every arm the device has confirmed
([`assignment_start`](@ref)) since the last pass. From then on the channel's
records are believed. An arm the device gave up on (`typemin(Int64)`) is
answered with `STATUS_ARM_REJECTED` (`REJECT_DEVICE_ERROR`) and the channel is
released.
"""
function confirm_arms!(core::LoopCore)
    T = core.channels
    for ch = 1:core.num_channels
        T.armed[ch] && !T.confirmed[ch] || continue
        start = assignment_start(core.driver, ch)
        start == typemax(Int64) && continue
        if start == typemin(Int64)
            # The device could not commit the handover: the arm is rejected and
            # the channel freed, as if the command had been refused outright.
            _publish_status!(core, ch, sample_count(core.driver, T.band[ch]), StatusEvent(
                HardwareLoopProtocol.STATUS_ARM_REJECTED, HardwareLoopProtocol.REJECT_DEVICE_ERROR,
                sample_count(core.driver, T.band[ch]), T.arm_sequence[ch]))
            release!(core.driver, ch)
            T.signal_index[ch] == 0 && (core.bands[T.band[ch]].noise_channel = 0)
            T.armed[ch] = false
            T.bank[ch] = 0
            continue
        end
        T.confirmed[ch] = true
        _publish_status!(core, ch, start, StatusEvent(
            HardwareLoopProtocol.STATUS_ARMED, HardwareLoopProtocol.REJECT_NONE, start, T.arm_sequence[ch],
            T.carrier_doppler[ch], T.code_doppler[ch], T.code_phase[ch], _bank_name(core, T.bank[ch]), T.group_key[ch]))
    end
    nothing
end

# Re-arm each band's noise reference onto a fresh decoy every
# `noise_rearm_epochs`: TrackingLoops' `next_noise_probe` draws it from the band's
# noise estimator — the next PRN the signal has a code for, a uniform code phase
# and a carrier within the estimator's dither of the intermediate frequency.
function _rearm_in_bank!(bank::ChannelBank, core::LoopCore, ch::Int, band_index::Int)
    b = core.bands[band_index]
    T = core.channels
    signal = bank.signal
    intermediate_frequency = b.entry.intermediate_frequency_hz
    probe = TrackingLoops.next_noise_probe(b.noise_estimator, signal, intermediate_frequency * Hz)
    carrier_doppler = ustrip(Hz, probe.carrier_frequency) - intermediate_frequency
    now = sample_count(core.driver, band_index)
    tap_sample_shifts, num_taps = _noise_tap_shifts(b, T.sampling_frequency[ch], signal)
    # The amplitudes the receiver declared, which the channel's scale divides out.
    spec = ArmSpec(
        signal, probe.prn, carrier_doppler, 0.0, probe.code_phase, now, tap_sample_shifts, num_taps,
        band_index, Int(b.entry.rf_input), Int(b.entry.device_index), T.sampling_frequency[ch],
        b.noise_replica_amplitude, b.noise_code_amplitude,
    )
    outcome = arm!(core.driver, ch, spec)
    outcome.accepted || return nothing
    b.noise_prn = probe.prn
    b.noise_code_phase = probe.code_phase
    b.noise_carrier_doppler = carrier_doppler
    T.prn[ch] = probe.prn
    T.confirmed[ch] = false
    b.noise_epochs_since_rearm = 0
    nothing
end

# A correlator's quantised offsets at the channel's rate, as the fixed tuple an
# arm carries.
function _template_tap_shifts(template::AbstractCorrelator, sampling_frequency_hz::Float64, signal)
    shifts = calc_correlator_sample_offsets(template, sampling_frequency_hz, _chip_rate_hz(signal))
    ntuple(i -> i <= length(shifts) ? Int32(shifts[i]) : Int32(0), Val(5))
end

# The taps a band's noise reference is programmed with: its noise correlator's,
# and how many.
function _noise_tap_shifts(band::BandState, sampling_frequency_hz::Float64, signal)
    correlator = TrackingLoops.noise_correlator(band.noise_estimator)
    _template_tap_shifts(correlator, sampling_frequency_hz, signal), get_num_correlators(correlator)
end

"""
    rearm_noise_references!(core)

Re-arm every band's noise reference that has served
[`LoopConfig`](@ref)`.noise_rearm_epochs` epochs onto a fresh decoy drawn by
TrackingLoops' `next_noise_probe`: the next PRN of its signal, a random code
phase and a carrier within ±5 kHz. A decoy that lines up with a visible but
untracked satellite biases the density for as long as it is parked, so the
re-arm interval bounds how much of the noise window one such alignment spoils.
"""
function rearm_noise_references!(core::LoopCore)
    for (band_index, b) in enumerate(core.bands)
        ch = b.noise_channel
        ch == 0 && continue
        b.noise_epochs_since_rearm >= core.config.noise_rearm_epochs || continue
        with_bank(_rearm_in_bank!, core, ch, core, ch, band_index)
    end
    nothing
end
