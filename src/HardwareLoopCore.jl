"""
    HardwareLoopCore

The engine of a hardware correlator's loop process: the dedicated,
allocation-free process that closes the tracking loops of an FPGA correlator.
It reads correlator records from a device, folds them epoch by epoch into
predetection integrations for `TrackingLoops`' tracking loops, writes the
resulting NCO commands back to the device, and publishes everything a receiver
needs into a `HardwareLoopProtocol` segment.

  - [`AbstractLoopDriver`](@ref) — the driver API a device implements.
  - [`LoopCore`](@ref) and [`service_pass!`](@ref) — the per-channel state and
    the epoch fold that turn device records into loop steps, the commands the
    core executes and the events it publishes.
  - Vector tracking: built with TrackingLoops' `VectorTrackingLoop`, the core
    runs every satellite on one navigation engine and publishes its solution
    on the segment's loop-wide nav ring.
  - [`SimulatedDevice`](@ref) — a software correlator behind the driver API, for
    tests and for trying the core without hardware.

The per-integration arithmetic — post-correlation processing, the bit
synchronizer, the C/N₀ estimators, discriminators, loop filters, the
delay-compensated tracking loop and its NCO timelines — is `TrackingLoops`'.
Warm service passes allocate nothing.
"""
module HardwareLoopCore

using GNSSSignals
using StaticArrays
using TrackingLoopFilters
using HardwareLoopProtocol
import Unitful
using Unitful: uconvert, ustrip, Hz
using TrackingLoops
# The documented API TrackingLoops leaves unexported: the correlator building
# blocks the core sums device records with.
using TrackingLoops:
    AbstractCorrelator,
    calc_correlator_sample_offsets,
    get_num_correlators,
    max_num_code_periods_to_integrate,
    with_accumulators

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
    write_nco_command!,
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
    commit_nco_commands!,
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
include("navigation.jl")
include("commands.jl")
include("service.jl")
include("simulated_device.jl")

end # module HardwareLoopCore
