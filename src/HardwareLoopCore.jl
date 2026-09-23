"""
    HardwareLoopCore

The engine of a hardware correlator's loop process
(GNSSReceiver.jl, `docs/plans/2026-09-22-loop-process.md`): the driver API a
device implements (`AbstractLoopDriver`), the per-channel state and the epoch
fold that turn device records into loop steps ([`LoopCore`](@ref),
[`service_pass!`](@ref)), the commands it executes and the events it publishes
into a `HardwareLoopProtocol` segment, and the simulated FPGA that is its first
driver ([`SimulatedDevice`](@ref)).

The per-record arithmetic — discriminators, loop filters, the bit buffer, the
C/N₀ estimators, the delay-aware estimator and its NCO timelines — is
`TrackingLoops`', shared with Tracking.jl's software receiver; this package is
what a vendor's loop executable (e.g. GNSSM2SDR's `M2SDRLoop`) plugs its driver
into. Warm service passes allocate nothing (`test/core.jl`).
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
