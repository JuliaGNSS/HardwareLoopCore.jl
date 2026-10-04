"""
    HardwareLoopCore

The engine of a hardware correlator's loop process: the dedicated,
allocation-free process that closes the tracking loops of an FPGA correlator.
It reads correlator records from a device, folds them epoch by epoch into
`TrackingLoops`' loop arithmetic, writes the resulting NCO words back to the
device, and publishes everything a receiver needs into a `HardwareLoopProtocol`
segment.

  - [`AbstractLoopDriver`](@ref) — the driver API a device implements.
  - [`LoopCore`](@ref) and [`service_pass!`](@ref) — the per-channel state and
    the epoch fold that turn device records into loop steps, the commands the
    core executes and the events it publishes.
  - [`SimulatedDevice`](@ref) — a software correlator behind the driver API, for
    tests and for trying the core without hardware.

The per-record arithmetic — discriminators, loop filters, the bit buffer, the
C/N₀ estimators, the delay-aware estimator and its NCO timelines — is
`TrackingLoops`'. Warm service passes allocate nothing.
"""
module HardwareLoopCore

using GNSSSignals
using StaticArrays
using TrackingLoopFilters
using HardwareLoopProtocol
import Unitful
using Unitful: uconvert, ustrip, Hz
using TrackingLoops
# The core folds records with TrackingLoops' own machinery — the bit buffer,
# the C/N₀ rings, the correlator helpers — and reads their internals by name.
for name in names(TrackingLoops; all = true)
    (name === :TrackingLoops || name === :eval || name === :include) && continue
    startswith(String(name), "#") && continue
    @eval import TrackingLoops: $name
end

export AbstractLoopDriver,
    DeviceRecord,
    strobe_record,
    pack_taps,
    ArmSpec,
    ArmOutcome,
    ARM_ACCEPTED,
    arm_rejected,
    DriverCapabilities,
    read_records!,
    write_word!,
    arm!,
    release!,
    assignment_start,
    sample_count,
    driver_capabilities,
    wait_records,
    LoopConfig,
    LoopCore,
    LATENCY_EDGES_US,
    take_records!,
    fold_closed_epochs!,
    commit_words!,
    handle_commands!,
    confirm_arms!,
    service_pass!,
    run!,
    SimulatedDevice,
    correlate_chunk!,
    is_strobe,
    overflowed_channels!,
    rearm_noise_references!,
    MAX_RECORD_TAPS

include("driver.jl")
include("state.jl")
include("ingest.jl")
include("fold.jl")
include("commands.jl")
include("service.jl")
include("simulated_device.jl")

end # module HardwareLoopCore
