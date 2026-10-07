# ─────────────────────────────────────────────────────────────────────────────
# The driver API: what the loop core asks of a hardware correlator. Concrete
# and statically dispatched — the driver is a type parameter of the core — so
# the loop process compiles to direct calls with nothing left to resolve at run
# time. The simulated FPGA in `simulated_device.jl` implements it.
# ─────────────────────────────────────────────────────────────────────────────

"""
    AbstractLoopDriver

Supertype of a hardware correlator's driver as the loop core sees it. Required:

  - [`read_records!`](@ref)`(driver, records)` — append every record the device
    has produced since the last call to `records` (a `Vector{DeviceRecord}`
    sized once), returning how many. Never blocks.
  - [`write_word!`](@ref)`(driver, channel, carrier_hz, code_hz)` — commit a
    carrier and code NCO word on `channel`, effective on the next sample. The
    carrier is a Doppler: the channel's carrier NCO runs at its band's
    intermediate frequency plus it.
  - [`arm!`](@ref)`(driver, channel, spec::ArmSpec)` — load a replica and start
    correlating; returns [`ArmOutcome`](@ref).
  - [`release!`](@ref)`(driver, channel)`.
  - [`assignment_start`](@ref)`(driver, channel)` — the device sample the
    channel's current assignment took effect at, `typemax(Int64)` while a
    scheduled arm has not been confirmed, or `typemin(Int64)` once the device
    has given up on it (the core then rejects the arm and frees the channel).
  - [`sample_count`](@ref)`(driver, band)` — the device's free-running sample
    counter on `band`'s counter.
  - [`driver_capabilities`](@ref)`(driver)` — the fixed limits.

Optional: [`wait_records`](@ref)`(driver, timeout_ms)` — block in the kernel
until records may be available (default: return immediately), and
[`overflowed_channels!`](@ref)`(driver)` — the device's own record-loss report,
cleared on read (default 0).
"""
abstract type AbstractLoopDriver end

"How many tap × antenna values one record can carry."
const MAX_RECORD_TAPS = HardwareLoopProtocol.MAX_TAP_VALUES

"""
    DeviceRecord(channel, prn, sample_index, integrated_samples, taps, num_taps;
                 band = 1, num_ants = 1, code_phase = NaN)

One correlator dump, or one epoch strobe, as the driver hands it to the core.
`isbits`, so the ingest buffer is one flat vector.

  - `channel` — hardware channel (1-based), or `0` for an epoch strobe.
  - `band` — the band the record's `sample_index` is counted on (1-based index
    into the core's band table).
  - `prn` — the PRN the channel was correlating, for stale-record detection.
  - `sample_index` — the device counter at the end of the integration.
  - `integrated_samples` — samples integrated.
  - `code_phase` — the replica's code phase in chips at `sample_index`, `NaN`
    when the device does not report it.
  - `num_taps`, `num_ants` — how many of `taps` are meaningful: latest tap first,
    antenna-major (`taps[(ant - 1) * num_taps + tap]`), raw accumulator sums.
"""
struct DeviceRecord
    channel::Int32
    band::UInt8
    num_taps::UInt8
    num_ants::UInt8
    flags::UInt8
    prn::Int32
    sample_index::Int64
    integrated_samples::Int32
    pad::Int32
    code_phase::Float64
    taps::NTuple{MAX_RECORD_TAPS,ComplexF64}
end

const RECORD_STROBE_CHANNEL = Int32(0)

function DeviceRecord(
    channel::Integer,
    prn::Integer,
    sample_index::Integer,
    integrated_samples::Integer,
    taps::NTuple{MAX_RECORD_TAPS,ComplexF64},
    num_taps::Integer;
    band::Integer = 1,
    num_ants::Integer = 1,
    code_phase::Real = NaN,
)
    DeviceRecord(
        Int32(channel),
        UInt8(band),
        UInt8(num_taps),
        UInt8(num_ants),
        0x00,
        Int32(prn),
        Int64(sample_index),
        Int32(integrated_samples),
        Int32(0),
        Float64(code_phase),
        taps,
    )
end

"""
    strobe_record(sample_index; band = 1) -> DeviceRecord

An epoch strobe: a timebase marker at `sample_index` on `band`'s counter. A
driver hands one to the core wherever the device strobes its epoch clock; the
core uses it to advance its epoch clock and folds nothing from it.
"""
strobe_record(sample_index::Integer; band::Integer = 1) = DeviceRecord(
    RECORD_STROBE_CHANNEL,
    0,
    sample_index,
    0,
    ntuple(_ -> complex(0.0, 0.0), MAX_RECORD_TAPS),
    0;
    band,
)

"""
    is_strobe(record::DeviceRecord) -> Bool

Whether `record` is an epoch strobe ([`strobe_record`](@ref)) rather than a
correlator dump.
"""
is_strobe(record::DeviceRecord) = record.channel == RECORD_STROBE_CHANNEL

"""
    pack_taps(values::AbstractVector{<:Complex}) -> NTuple{MAX_RECORD_TAPS,ComplexF64}

Pack tap values into the fixed tuple a [`DeviceRecord`](@ref) carries, in the
order given (latest tap first, antenna-major), zero beyond `length(values)`.
Throws an `ArgumentError` for more than [`MAX_RECORD_TAPS`](@ref) values.
"""
function pack_taps(values::AbstractVector{<:Complex})
    n = length(values)
    n <= MAX_RECORD_TAPS || throw(ArgumentError("more than $MAX_RECORD_TAPS tap values"))
    ntuple(i -> i <= n ? ComplexF64(values[i]) : complex(0.0, 0.0), MAX_RECORD_TAPS)
end

"""
    ArmSpec

What the core asks a driver to program when arming a channel: the signal (as
the concrete GNSSSignals object), the PRN, the handover Dopplers and code
phase valid at `valid_at_sample` (on the channel's band counter), the quantised
tap offsets, the band and RF routing, and the amplitude declarations the record
scaling depends on.
"""
struct ArmSpec{S<:AbstractGNSSSignal}
    signal::S
    prn::Int
    carrier_doppler_hz::Float64
    code_doppler_hz::Float64
    code_phase_chips::Float64
    valid_at_sample::Int64
    tap_sample_shifts::NTuple{5,Int32}
    num_taps::Int
    band::Int
    rf_input::Int
    device_index::Int
    sampling_freq_hz::Float64
    replica_amplitude::Float64
    code_amplitude::Float64
end

"The outcome of `arm!`: accepted (confirmation follows through `assignment_start`) or rejected with a protocol reason code."
struct ArmOutcome
    accepted::Bool
    reason::UInt32
end

"""
    ARM_ACCEPTED

The [`ArmOutcome`](@ref) a driver's [`arm!`](@ref) returns when it accepted the
arm. The core then waits for [`assignment_start`](@ref) to confirm it.
"""
const ARM_ACCEPTED = ArmOutcome(true, HardwareLoopProtocol.REJECT_NONE)

"""
    arm_rejected(reason) -> ArmOutcome

The [`ArmOutcome`](@ref) of a refused arm. `reason` is one of
`HardwareLoopProtocol`'s `REJECT_*` codes (e.g. `REJECT_UNSUPPORTED_SIGNAL`,
`REJECT_BAD_CONFIG`); the core forwards it to the receiver in a
`STATUS_ARM_REJECTED` status event and frees the channel.
"""
arm_rejected(reason) = ArmOutcome(false, UInt32(reason))

"""
    DriverCapabilities

The device's fixed limits as the core needs them: channels, the widest tap
layout its records carry, antennas, and the band table.

Each band's `BandEntry` carries its `intermediate_frequency_hz`: where a signal
at zero Doppler sits in the band's samples, including any fixed offset the
front end's tuning leaves (an RTL-SDR's LO synthesizer lands a fixed number of
Hz off the requested frequency, which follows from its settings). The driver
runs every carrier NCO of the band at that IF plus the Doppler the core
commands; the core, the protocol and the receiver deal in Dopplers only.
"""
struct DriverCapabilities
    num_channels::Int
    max_taps::Int
    num_ants::Int
    bands::Vector{BandEntry}
end

"""
    read_records!(driver, records::Vector{DeviceRecord}) -> Int

Append every record the device has produced since the last call — correlator
dumps and epoch strobes, in the order the device produced them — to `records`
and return how many were appended. Must never block, and should not allocate:
the core passes the same vector, emptied and with its capacity reserved, on
every pass. Required for every [`AbstractLoopDriver`](@ref).
"""
function read_records! end

"""
    write_word!(driver, channel, carrier_hz::Float64, code_hz::Float64) -> Bool

Commit a carrier and a code NCO word (the Dopplers, in Hz) on `channel`,
effective on the device's next sample. The carrier NCO runs at the band's
intermediate frequency ([`DriverCapabilities`](@ref)) plus `carrier_hz`. Return `false` if the device refused the
word (e.g. the channel is not running), which the core counts as rejected.
Required for every [`AbstractLoopDriver`](@ref).
"""
function write_word! end

"""
    arm!(driver, channel, spec::ArmSpec) -> ArmOutcome

Load the replica [`ArmSpec`](@ref) describes onto `channel` and start
correlating, replacing whatever the channel ran before. Return
[`ARM_ACCEPTED`](@ref), and report the sample the assignment took effect at
through [`assignment_start`](@ref) once it has; or return
[`arm_rejected`](@ref)`(reason)` if the device cannot serve it. Required for
every [`AbstractLoopDriver`](@ref).
"""
function arm! end

"""
    release!(driver, channel)

Stop `channel` correlating. Records the device has already produced for it may
still arrive; the core drops them. Required for every
[`AbstractLoopDriver`](@ref).
"""
function release! end

"""
    assignment_start(driver, channel) -> Int64

The device sample (on the channel's band counter) the channel's current
assignment took effect at. While an accepted arm is still waiting to take
effect, `typemax(Int64)`; once the device has given up on it, `typemin(Int64)`,
and the core then rejects the arm and releases the channel. Records whose
integration began before this sample are dropped as stale. Required for every
[`AbstractLoopDriver`](@ref).
"""
function assignment_start end

"""
    sample_count(driver, band::Integer) -> Int64

The device's free-running sample counter for `band` (an index into
[`DriverCapabilities`](@ref)`.bands`), read now. Band 1 is the reference band:
its counter is the receiver timebase the epoch clock runs on. Required for
every [`AbstractLoopDriver`](@ref).
"""
function sample_count end

"""
    driver_capabilities(driver) -> DriverCapabilities

The device's fixed limits ([`DriverCapabilities`](@ref)), read once when the
[`LoopCore`](@ref) is built. Required for every [`AbstractLoopDriver`](@ref).
"""
function driver_capabilities end

"""
    wait_records(driver, timeout_ms::Integer)

Block until the device may have records, for at most `timeout_ms` milliseconds
— e.g. on a DMA interrupt. Optional: the default returns at once, so the
service loop polls.
"""
wait_records(::AbstractLoopDriver, timeout_ms::Integer) = nothing

"""
    overflowed_channels!(driver) -> Integer

The channels the device reports having lost records on since the last call (a
bitmap or a count, as the device keeps it), cleared on read. Optional: the
default reports none (`0`).
"""
overflowed_channels!(::AbstractLoopDriver) = 0
