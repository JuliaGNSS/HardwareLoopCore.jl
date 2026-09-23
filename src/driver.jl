# ─────────────────────────────────────────────────────────────────────────────
# The driver API: what the loop core asks of a hardware correlator. Concrete
# and statically dispatched — the driver is a type parameter of the core — so
# the loop process compiles to direct calls with nothing left to resolve at run
# time. GNSSM2SDR's driver half and the simulated FPGA in `simulated_device.jl`
# implement it.
# ─────────────────────────────────────────────────────────────────────────────

"""
    AbstractLoopDriver

Supertype of a hardware correlator's driver as the loop core sees it. Required:

  - [`read_records!`](@ref)`(driver, records)` — append every record the device
    has produced since the last call to `records` (a `Vector{DeviceRecord}`
    sized once), returning how many. Never blocks.
  - [`write_word!`](@ref)`(driver, channel, carrier_hz, code_hz)` — commit a
    carrier and code NCO word on `channel`, effective on the next sample.
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
    DeviceRecord

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

"An epoch strobe: a timebase marker on `band`'s counter."
strobe_record(sample_index::Integer; band::Integer = 1) = DeviceRecord(
    RECORD_STROBE_CHANNEL,
    0,
    sample_index,
    0,
    ntuple(_ -> complex(0.0, 0.0), MAX_RECORD_TAPS),
    0;
    band,
)

is_strobe(record::DeviceRecord) = record.channel == RECORD_STROBE_CHANNEL

# Pack tap values into the fixed tuple, latest first, zeros beyond `n`.
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

const ARM_ACCEPTED = ArmOutcome(true, HardwareLoopProtocol.REJECT_NONE)
arm_rejected(reason) = ArmOutcome(false, UInt32(reason))

"""
    DriverCapabilities

The device's fixed limits as the core needs them: channels, the widest tap
layout its records carry, antennas, and the band table.
"""
struct DriverCapabilities
    num_channels::Int
    max_taps::Int
    num_ants::Int
    bands::Vector{BandEntry}
end

function read_records! end
function write_word! end
function arm! end
function release! end
function assignment_start end
function sample_count end
function driver_capabilities end

"Block until the device may have records, at most `timeout_ms`; the default returns at once."
wait_records(::AbstractLoopDriver, timeout_ms::Integer) = nothing
"Channels the device reports having lost records on since the last call (a bitmap or count), cleared on read."
overflowed_channels!(::AbstractLoopDriver) = 0
