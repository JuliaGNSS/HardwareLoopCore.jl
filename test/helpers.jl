# Fixtures shared by the test files: a core over the simulated device, arming
# over the command ring, draining a channel's events, a synthetic satellite.

using HardwareLoopProtocol
const HLP = HardwareLoopProtocol

const CORE_SYSTEM = GPSL1CA()
const CORE_FS = 4e6
const CORE_EPOCH = 4000

# A device, a heap-backed segment and a core over them.
function core_fixture(; num_channels = 4, record_delay = 0, handover_error = 0.25, config = nothing)
    dev = SimulatedDevice(
        CORE_SYSTEM;
        sampling_freq = CORE_FS,
        num_channels,
        handover_code_phase_error = handover_error,
        record_delay_samples = record_delay,
    )
    seg = create_segment(
        nothing,
        SegmentConfig(;
            channel_count = num_channels,
            bands = [BandEntry(get_band_id(get_band(CORE_SYSTEM)), CORE_FS)],
        ),
    )
    core = LoopCore(dev, (CORE_SYSTEM,), seg; config)
    (; dev, seg, core)
end

core_tap_shifts(core) = HardwareLoopCore._template_tap_shifts(core.banks[1].template, CORE_FS, CORE_SYSTEM)

function arm!(f, channel, prn; doppler, code_phase, signal_index = 1, sequence, signal = get_signal_id(CORE_SYSTEM),
              valid_at_sample = 0, replica_amplitude = 1.0, num_taps = 3, sampling_freq_hz = CORE_FS,
              band = 1, group_key = signal, want_taps = false,
              secondary_code_mode = HLP.SECONDARY_PRIMARY_ONLY)
    cmd = ArmCommand(;
        signal,
        prn,
        signal_index,
        carrier_doppler_hz = doppler,
        code_doppler_hz = doppler / 1540,
        code_phase_chips = code_phase,
        valid_at_sample,
        tap_sample_shifts = core_tap_shifts(f.core),
        num_taps,
        sampling_freq_hz,
        replica_amplitude,
        band,
        group_key,
        want_taps,
        secondary_code_mode,
    )
    publish!(command_ring(f.seg), CommandTag(HLP.COMMAND_ARM, channel, sequence), cmd)
end

# Every event on a channel's ring, typed.
function drain_events!(f, channel)
    ring = event_ring(f.seg, channel)
    states = Tuple{EventTag,EpochStateEvent}[]
    statuses = Tuple{EventTag,StatusEvent}[]
    bits = Tuple{EventTag,BitEvent}[]
    records = Tuple{EventTag,RecordEvent}[]
    taps = Tuple{EventTag,TapsEvent}[]
    while true
        status, view, _ = peek!(ring, EventTag)
        status === :empty && break
        tag = view.tag
        if tag.kind == HLP.EVENT_EPOCH_STATE
            push!(states, (tag, payload(EpochStateEvent, ring, view)))
        elseif tag.kind == HLP.EVENT_STATUS
            push!(statuses, (tag, payload(StatusEvent, ring, view)))
        elseif tag.kind == HLP.EVENT_BIT
            push!(bits, (tag, payload(BitEvent, ring, view)))
        elseif tag.kind == HLP.EVENT_RECORD
            push!(records, (tag, payload(RecordEvent, ring, view)))
        elseif tag.kind == HLP.EVENT_TAPS
            push!(taps, (tag, payload(TapsEvent, ring, view)))
        end
        commit!(ring, view)
    end
    (; states, statuses, bits, records, taps)
end

# One PRN at `true_doppler`, a 20 ms bit stream, unit-variance noise.
function synthesize!(buf, n0, prn, true_doppler, code_phase0, amplitude, rng)
    code_freq = 1.023e6 + true_doppler / 1540
    @inbounds for k in eachindex(buf)
        t = (n0 + k - 1) / CORE_FS
        code = get_code(CORE_SYSTEM, code_phase0 + code_freq * t, prn)
        bit = isodd(div(n0 + k - 1, 80_000)) ? -1.0 : 1.0
        buf[k] = amplitude * bit * code * cis(2π * true_doppler * t) + randn(rng, ComplexF64)
    end
    buf
end

# Run the loop closed for `seconds`: the satellite on channel 1, a noise
# reference on the last channel, one service pass per epoch-long chunk.
function run_closed_loop(; record_delay = 0, seconds = 1.4, prn = 11, true_doppler = 1200.0,
                         handover_doppler_error = -20.0, code_phase0 = 137.4, amplitude = 0.126,
                         config = nothing, warmup_chunks = 300, replica_amplitude = 1.0)
    f = core_fixture(; record_delay, config)
    arm!(f, 1, prn; doppler = true_doppler + handover_doppler_error, code_phase = code_phase0, sequence = 1, replica_amplitude)
    arm!(f, 4, 30; doppler = 3000.0, code_phase = 500.0, signal_index = 0, sequence = 2, replica_amplitude)
    num_chunks = round(Int, seconds * CORE_FS / CORE_EPOCH)
    rng = Xoshiro(0xC0FFEE)
    buf = Vector{ComplexF64}(undef, CORE_EPOCH)
    allocated = 0
    states = Tuple{EventTag,EpochStateEvent}[]
    statuses = Tuple{EventTag,StatusEvent}[]
    bits = Tuple{EventTag,BitEvent}[]
    records = Tuple{EventTag,RecordEvent}[]
    for c = 0:(num_chunks-1)
        synthesize!(buf, c * CORE_EPOCH, prn, true_doppler, code_phase0, amplitude, rng)
        correlate_chunk!(f.dev, buf)
        a = @allocated service_pass!(f.core; wait_ms = 0)
        c >= warmup_chunks && (allocated += a)
        ev = drain_events!(f, 1)
        append!(states, ev.states); append!(statuses, ev.statuses)
        append!(bits, ev.bits); append!(records, ev.records)
    end
    code_freq = 1.023e6 + true_doppler / 1540
    true_code_phase = mod(code_phase0 + code_freq * f.dev.sample_count / CORE_FS, 1023)
    device_code_error = mod(f.dev.channels[1].code_phase - true_code_phase + 511.5, 1023) - 511.5
    (; f, states, statuses, bits, records, allocated, device_code_error,
       device_doppler = f.dev.channels[1].carrier_doppler, true_doppler)
end

