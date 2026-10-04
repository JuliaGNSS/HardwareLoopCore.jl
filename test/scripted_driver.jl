# A driver whose records the test writes by hand: every branch of the core
# that a well-behaved simulated FPGA never takes — lost and stale records,
# refused arms and words, a device that gives up on a handover, several antennas
# and bands — is reached by queueing the record or setting the answer here.

mutable struct ScriptedDriver <: AbstractLoopDriver
    num_ants::Int
    bands::Vector{BandEntry}
    sample_count::Int64
    queue::Vector{DeviceRecord}
    # Per channel: what `assignment_start` answers.
    starts::Vector{Int64}
    # What the next `arm!` answers, and whether an accepted arm is confirmed at once.
    arm_outcome::ArmOutcome
    confirm_on_arm::Bool
    accept_words::Bool
    arms::Vector{Tuple{Int,ArmSpec}}
    words::Vector{Tuple{Int,Int64,Float64,Float64}}
    releases::Vector{Int}
    waits::Int
end

function ScriptedDriver(; num_channels = 4, num_ants = 1, bands = [BandEntry("L1", CORE_FS)])
    ScriptedDriver(num_ants, bands, 0, DeviceRecord[], fill(typemax(Int64), num_channels),
                   ARM_ACCEPTED, true, true, Tuple{Int,ArmSpec}[], Tuple{Int,Int64,Float64,Float64}[],
                   Int[], 0)
end

HardwareLoopCore.driver_capabilities(d::ScriptedDriver) =
    DriverCapabilities(length(d.starts), 5, d.num_ants, d.bands)
HardwareLoopCore.sample_count(d::ScriptedDriver, band::Integer) =
    round(Int64, d.sample_count * d.bands[band].sampling_freq_hz / d.bands[1].sampling_freq_hz)
HardwareLoopCore.assignment_start(d::ScriptedDriver, channel::Integer) = d.starts[channel]
HardwareLoopCore.wait_records(d::ScriptedDriver, ::Integer) = (d.waits += 1; nothing)

function HardwareLoopCore.read_records!(d::ScriptedDriver, records::Vector{DeviceRecord})
    append!(records, d.queue)
    n = length(d.queue)
    empty!(d.queue)
    n
end

function HardwareLoopCore.write_word!(d::ScriptedDriver, channel::Integer, carrier::Float64, code::Float64)
    d.accept_words || return false
    push!(d.words, (Int(channel), d.sample_count, carrier, code))
    true
end

function HardwareLoopCore.arm!(d::ScriptedDriver, channel::Integer, spec::ArmSpec)
    push!(d.arms, (Int(channel), spec))
    d.arm_outcome.accepted || return d.arm_outcome
    d.starts[channel] = d.confirm_on_arm ? d.sample_count : typemax(Int64)
    d.arm_outcome
end

function HardwareLoopCore.release!(d::ScriptedDriver, channel::Integer)
    push!(d.releases, Int(channel))
    d.starts[channel] = typemax(Int64)
    nothing
end

# A core over a scripted driver and a heap segment.
function scripted_fixture(signals = (CORE_SYSTEM,); num_channels = 4, num_ants = 1,
                          bands = [BandEntry("L1", CORE_FS)], config = nothing, kwargs...)
    dev = ScriptedDriver(; num_channels, num_ants, bands)
    seg = create_segment(nothing, SegmentConfig(; channel_count = num_channels, bands))
    core = LoopCore(dev, signals, seg; config, num_ants = NumAnts(num_ants), kwargs...)
    (; dev, seg, core)
end

# An early/prompt/late record of a GPS L1 C/A satellite perfectly in lock: the
# prompt carries `amplitude · bit` per sample, the early and late taps half of it.
function epl_record(channel, prn, sample_end, bit; samples = CORE_EPOCH, amplitude = 1.0,
                    code_phase = 0.0, num_ants = 1, band = 1)
    prompt = amplitude * bit * samples
    taps = ComplexF64[]
    for _ = 1:num_ants
        append!(taps, (prompt / 2, prompt, prompt / 2))
    end
    DeviceRecord(channel, prn, sample_end, samples, pack_taps(taps), 3; num_ants, band, code_phase)
end

# Queue `count` one-code-period records on `channel` and the epoch strobes, with
# a navigation bit that flips every 20 records (counting from record `first`),
# advancing the device counter; then run one service pass per record.
function feed_epl!(f, channel, prn, count; first = 1, skip = (), amplitude = 1.0, passes = true,
                   code_phase = 0.0)
    for k = first:(first+count-1)
        f.dev.sample_count = Int64(k) * CORE_EPOCH
        bit = isodd(div(k - 1, 20)) ? -1.0 : 1.0
        k in skip || push!(f.dev.queue, epl_record(channel, prn, f.dev.sample_count, bit; amplitude, code_phase))
        push!(f.dev.queue, strobe_record(f.dev.sample_count))
        passes && service_pass!(f.core; wait_ms = 0)
    end
    nothing
end
