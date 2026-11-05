# ─────────────────────────────────────────────────────────────────────────────
# A simulated hardware correlator behind the driver API: the software stand-in
# for an FPGA correlator's gateware.
# It correlates the samples it is handed with the replicas its channels hold,
# cuts a record per primary code period (and, when told to, inside one), strobes
# the timebase, and applies a committed NCO command on the next sample. Nothing here
# is a mock: the loop core really has to close through it.
# ─────────────────────────────────────────────────────────────────────────────

const SIM_MAX_CORRELATORS = 5

# One channel's replica state, i.e. what the gateware's NCOs hold.
mutable struct SimulatedChannel
    active::Bool
    signal_index::Int
    prn::Int
    carrier_phase::Float64      # cycles
    carrier_doppler::Float64    # Hz
    code_phase::Float64         # chips
    code_doppler::Float64       # Hz
    chip_rate::Float64  # Hz
    code_length::Float64        # chips
    correlator_sample_offsets::NTuple{SIM_MAX_CORRELATORS,Int32}   # samples, latest replica first
    num_correlators::Int
    # The replica and code amplitudes the arm declared: the device's
    # correlator outputs carry both, as the gateware's ±127 carrier ROM and an integer
    # code table (GNSSSignals' ±25/±13 CBOC) do. `get_code` is unit RMS.
    gain::Float64
    correlator_outputs::MVector{SIM_MAX_CORRELATORS,ComplexF64}
    num_integrated_samples::Int
    assignment_start::Int64
end

SimulatedChannel() = SimulatedChannel(
    false, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    ntuple(_ -> Int32(0), SIM_MAX_CORRELATORS), 0, 1.0,
    zero(MVector{SIM_MAX_CORRELATORS,ComplexF64}), 0, typemax(Int64),
)

"""
    SimulatedDevice(signals::Tuple; sampling_frequency, num_channels = 6,
                    nco_update_interval_samples, dump_interval_samples = 0,
                    handover_code_phase_error = 0.0, record_delay_samples = 0,
                    intermediate_frequency = 0.0, band_id = get_band_id(get_band(first(signals))))
    SimulatedDevice(signal; kwargs...)

A simulated hardware correlator for any of `signals` (one band, one antenna) at
`sampling_frequency` (Hz), fed raw samples through [`correlate_chunk!`](@ref) and
strobing the timebase every `nco_update_interval_samples` samples, the NCO
update interval (default: one primary code period of the first signal).

`dump_interval_samples` makes it dump *inside* a primary code period as well as
on the wrap. `handover_code_phase_error` is a deliberate error, in chips, added
to every arm's code phase: real handovers are never exact, and it is what makes
the code loop's sign observable over a short run. `record_delay_samples` holds
every record back until the counter is that far past its end — the DMA latency
of a real device, and the knob the delay-tolerance tests turn.
`intermediate_frequency` (Hz) is where the samples' zero Doppler sits, reported
as the band's `intermediate_frequency_hz`: every channel's carrier NCO runs at it
plus the Doppler the core commands, as a gateware NCO does. Besides a real IF it
takes a front end whose tuned frequency is off by a fixed amount (an RTL-SDR's
LO synthesizer lands a fixed number of Hz off).
"""
mutable struct SimulatedDevice{S<:Tuple} <: AbstractLoopDriver
    const signals::S
    const channels::Vector{SimulatedChannel}
    const sampling_frequency::Float64
    const nco_update_interval_samples::Int
    const dump_interval_samples::Int
    const handover_code_phase_error::Float64
    const record_delay_samples::Int
    const bands::Vector{BandEntry}
    sample_count::Int64
    # Records produced and not yet read: `records[read_head+1:end]`. A head
    # index rather than a shrinking vector, so reading never allocates.
    const records::Vector{DeviceRecord}
    read_head::Int
    # Every NCO command committed: (channel, device sample it took effect at, carrier, code).
    const nco_commands::Vector{NTuple{4,Float64}}
    # Every arm accepted, for the tests: (channel, prn, application_sample_index, sample armed at).
    const arms::Vector{NTuple{4,Int64}}
end

function SimulatedDevice(
    signals::Tuple{AbstractGNSSSignal,Vararg{AbstractGNSSSignal}};
    sampling_frequency,
    num_channels::Integer = 6,
    nco_update_interval_samples::Union{Nothing,Integer} = nothing,
    dump_interval_samples::Integer = 0,
    handover_code_phase_error::Real = 0.0,
    record_delay_samples::Integer = 0,
    intermediate_frequency::Real = 0.0,
    band_id = get_band_id(get_band(first(signals))),
)
    fs = sampling_frequency isa Real ? Float64(sampling_frequency) : Float64(ustrip(Hz, uconvert(Hz, sampling_frequency)))
    interval = something(
        nco_update_interval_samples,
        round(Int, get_code_length(first(signals)) * fs / _sim_chip_rate(first(signals))),
    )
    SimulatedDevice(
        signals,
        [SimulatedChannel() for _ = 1:num_channels],
        fs,
        Int(interval),
        Int(dump_interval_samples),
        Float64(handover_code_phase_error),
        Int(record_delay_samples),
        [BandEntry(band_id, fs; intermediate_frequency_hz = intermediate_frequency)],
        Int64(0),
        sizehint!(DeviceRecord[], 1 << 16),
        0,
        sizehint!(NTuple{4,Float64}[], 1 << 16),
        sizehint!(NTuple{4,Int64}[], 1 << 10),
    )
end

SimulatedDevice(signal::AbstractGNSSSignal; kwargs...) = SimulatedDevice((signal,); kwargs...)

_sim_chip_rate(signal::AbstractGNSSSignal) = ustrip(Hz, uconvert(Hz, get_chip_rate(signal)))

# The index of `signal`'s type in the device's tuple, or 0.
_sim_signal_index(::Tuple{}, signal, k::Int) = 0
_sim_signal_index(signals::Tuple, signal, k::Int) =
    typeof(first(signals)) === typeof(signal) ? k : _sim_signal_index(Base.tail(signals), signal, k + 1)

# The code chip of the `k`th signal, dispatched down the tuple.
@inline _sim_code(signals::Tuple, k::Int, phase::Float64, prn::Int) =
    k == 1 ? Float64(get_code(first(signals), phase, prn)) :
    _sim_code(Base.tail(signals), k - 1, phase, prn)
@inline _sim_code(::Tuple{}, ::Int, ::Float64, ::Int) = 0.0

# ── The driver API ────────────────────────────────────────────────────────────

driver_capabilities(dev::SimulatedDevice) =
    DriverCapabilities(length(dev.channels), SIM_MAX_CORRELATORS, 1, dev.bands)

sample_count(dev::SimulatedDevice, ::Integer) = dev.sample_count

assignment_start(dev::SimulatedDevice, channel::Integer) = dev.channels[channel].assignment_start

function read_records!(dev::SimulatedDevice, records::Vector{DeviceRecord})
    queue = dev.records
    visible = dev.sample_count - dev.record_delay_samples
    head = dev.read_head
    n = 0
    @inbounds while head + n < length(queue) && queue[head+n+1].sample_index <= visible
        push!(records, queue[head+n+1])
        n += 1
    end
    head += n
    if head == length(queue)
        empty!(queue)
        head = 0
    end
    dev.read_head = head
    n
end

function write_nco_command!(dev::SimulatedDevice, channel::Integer, carrier_doppler_hz::Float64, code_doppler_hz::Float64)
    ch = dev.channels[channel]
    ch.active || return false
    ch.carrier_doppler = carrier_doppler_hz
    ch.code_doppler = code_doppler_hz
    push!(dev.nco_commands, (Float64(channel), Float64(dev.sample_count), carrier_doppler_hz, code_doppler_hz))
    true
end

function arm!(dev::SimulatedDevice, channel::Integer, spec::ArmSpec)
    k = _sim_signal_index(dev.signals, spec.signal, 1)
    k == 0 && return arm_rejected(HardwareLoopProtocol.REJECT_UNSUPPORTED_SIGNAL)
    1 <= spec.num_correlators <= SIM_MAX_CORRELATORS || return arm_rejected(HardwareLoopProtocol.REJECT_BAD_CONFIG)
    spec.band == 1 || return arm_rejected(HardwareLoopProtocol.REJECT_BAD_CONFIG)
    ch = dev.channels[channel]
    code_length = Float64(get_code_length(spec.signal))
    nominal = _sim_chip_rate(spec.signal)
    code_freq = nominal + spec.code_doppler_hz
    # The handover describes the satellite at `application_sample_index` on the device's
    # own counter; the phase is propagated over the samples since.
    elapsed = dev.sample_count - spec.application_sample_index
    ch.active = true
    ch.signal_index = k
    ch.prn = spec.prn
    ch.carrier_doppler = spec.carrier_doppler_hz
    ch.carrier_phase = 0.0
    ch.code_doppler = spec.code_doppler_hz
    ch.chip_rate = nominal
    ch.code_length = code_length
    ch.code_phase = mod(
        spec.code_phase_chips + dev.handover_code_phase_error + code_freq * elapsed / dev.sampling_frequency,
        code_length,
    )
    ch.correlator_sample_offsets = spec.correlator_sample_offsets
    ch.num_correlators = spec.num_correlators
    ch.gain = spec.replica_amplitude * spec.code_amplitude
    ch.correlator_outputs .= 0
    ch.num_integrated_samples = 0
    ch.assignment_start = dev.sample_count
    push!(dev.arms, (Int64(channel), Int64(spec.prn), spec.application_sample_index, dev.sample_count))
    ARM_ACCEPTED
end

function release!(dev::SimulatedDevice, channel::Integer)
    ch = dev.channels[channel]
    ch.active = false
    ch.assignment_start = typemax(Int64)
    nothing
end

# ── The device itself ────────────────────────────────────────────────────────

"""
    correlate_chunk!(dev::SimulatedDevice, samples) -> Int

Correlate one chunk of raw samples with every active channel's replica, queue
the records it completed (and the timebase strobes) for `read_records!`, and
return how many were queued.
"""
function correlate_chunk!(dev::SimulatedDevice, samples::AbstractVector)
    queued = 0
    channels = dev.channels
    fs = dev.sampling_frequency
    intermediate_frequency = dev.bands[1].intermediate_frequency_hz
    @inbounds for k in eachindex(samples)
        sample = ComplexF64(samples[k])
        for index in eachindex(channels)
            ch = channels[index]
            ch.active || continue
            code_freq = ch.chip_rate + ch.code_doppler
            wipeoff = ch.gain * sample * cis(-2π * ch.carrier_phase)
            for k = 1:ch.num_correlators
                offset_chips = ch.correlator_sample_offsets[k] * code_freq / fs
                chip = _sim_code(dev.signals, ch.signal_index, ch.code_phase + offset_chips, ch.prn)
                ch.correlator_outputs[k] += wipeoff * chip
            end
            ch.carrier_phase += (intermediate_frequency + ch.carrier_doppler) / fs
            ch.code_phase += code_freq / fs
            ch.num_integrated_samples += 1
            wrapped = ch.code_phase >= ch.code_length
            wrapped && (ch.code_phase -= ch.code_length)
            if wrapped || (dev.dump_interval_samples > 0 && ch.num_integrated_samples >= dev.dump_interval_samples)
                push!(
                    dev.records,
                    DeviceRecord(
                        index,
                        ch.prn,
                        dev.sample_count + 1,
                        ch.num_integrated_samples,
                        pack_correlator_outputs(ch.correlator_outputs),
                        ch.num_correlators;
                        code_phase = ch.code_phase,
                    ),
                )
                queued += 1
                ch.correlator_outputs .= 0
                ch.num_integrated_samples = 0
            end
        end
        dev.sample_count += 1
        if dev.sample_count % dev.nco_update_interval_samples == 0
            push!(dev.records, strobe_record(dev.sample_count))
            queued += 1
        end
    end
    queued
end
