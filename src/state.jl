# ─────────────────────────────────────────────────────────────────────────────
# The loop core's state: one fixed-size table per hardware channel, one bank
# per signal type the process was compiled for, and the loop-wide configuration.
# Everything is sized once at construction; nothing on the service path grows.
# ─────────────────────────────────────────────────────────────────────────────

"""
    LoopConfig(; kwargs...)

Loop-wide configuration, changed at run time only through a
`ConfigureCommand`:

  - `epoch_length` — the fold epoch in reference-band samples (default: one
    primary code period of the first supported signal).
  - `commit_lead_samples` — how far past the device counter read at the start
    of a pass the NCO commands that pass computes are predicted to take effect
    (0). A command is written in the very pass that computes it; the timeline
    is confirmed at the sample it really took effect at.
  - `num_code_periods_to_integrate` — the predetection integration ceiling in
    primary code periods once bit/secondary sync is found; `0` for one whole
    bit. The default is 1: the loops are tuned for one step per code period
    (an 18 Hz PLL at 1 ms), and stepping them once per 20 ms bit instead
    puts the bandwidth–time product at 0.36, where the delay-compensated loop
    limit-cycles or drifts (measured on the board and reproduced with the
    simulated device: a 48 dBHz satellite's Doppler ran 30 Hz off in six
    seconds after bit sync). The bit synchronizer integrates the bit itself.
  - `max_integration_time` — the longest predetection integration, in seconds
    (20 ms).
  - `max_backlog_epochs` — how many epochs behind the device the fold may run
    before older records are folded observation-only (4).
  - `noise_rearm_epochs` — how often a band's noise reference is re-armed onto
    a fresh decoy (1000 epochs, one second at a 1 ms epoch).
  - `publish_taps` — whether every armed channel publishes `TapsEvent`s.
  - `max_epoch_clock_advance` — the furthest a single record may move the epoch
    clock, in reference samples (one second).
"""
mutable struct LoopConfig
    epoch_length::Int64
    commit_lead_samples::Int64
    num_code_periods_to_integrate::Int
    max_integration_time::Float64
    max_backlog_epochs::Int
    noise_rearm_epochs::Int
    publish_taps::Bool
    max_epoch_clock_advance::Int64
end

function LoopConfig(;
    epoch_length::Integer,
    commit_lead_samples::Integer = 0,
    num_code_periods_to_integrate::Integer = 1,
    max_integration_time::Real = 20e-3,
    max_backlog_epochs::Integer = 4,
    noise_rearm_epochs::Integer = 1000,
    publish_taps::Bool = false,
    max_epoch_clock_advance::Integer = 0,
)
    epoch_length >= 1 || throw(ArgumentError("epoch_length must be at least one sample"))
    max_integration_time > 0 || throw(ArgumentError("max_integration_time must be positive"))
    LoopConfig(
        Int64(epoch_length),
        Int64(commit_lead_samples),
        Int(num_code_periods_to_integrate),
        Float64(max_integration_time),
        Int(max_backlog_epochs),
        Int(noise_rearm_epochs),
        publish_taps,
        max_epoch_clock_advance == 0 ? Int64(1000) * Int64(epoch_length) :
        Int64(max_epoch_clock_advance),
    )
end

"""
    ChannelBank{S,C,B,PCF,CN0}

Everything about a hardware channel whose *type* depends on the signal it
tracks: the signal object, its post-correlation state (bit synchronizer,
C/N₀ estimator, post-correlation filter) and the open predetection
integration's correlator. One bank per supported signal type, each with a slot
for every hardware channel, so every slot is concretely typed and a
per-integration rebuild stores inline instead of boxing.
"""
struct ChannelBank{S<:AbstractGNSSSignal,C<:AbstractCorrelator,B<:Unsigned,PCF,CN0}
    signal::S
    # The signal's id as the fixed name an arm command carries.
    name::FixedName
    # The host-side correlator whose tap spacing the discriminators read; the
    # device is programmed with exactly its quantised offsets.
    template::C
    states::Vector{PostCorrelationState{B,PCF,CN0}}
    partial::Vector{C}
end

function ChannelBank(signal::AbstractGNSSSignal, num_channels::Integer, num_antennas::NumAntennas = NumAntennas(1))
    template = get_default_correlator(signal, num_antennas)
    ChannelBank(
        signal,
        FixedName(get_signal_id(signal)),
        template,
        [PostCorrelationState(signal) for _ = 1:num_channels],
        [zero(template) for _ = 1:num_channels],
    )
end

"""
    BandState

One RF band the loop serves: its entry in the band table, the scale from its
sample counter onto the reference band's, and its noise reference — the
hardware channel (0 for none), the decoy PRN it replicates, the replica and
code amplitudes the receiver declared for it (a re-arm programs the same), the
pooled `Σ|b|²` of this epoch's looks and their count, and the epochs since the
last re-arm.
"""
mutable struct BandState
    const entry::BandEntry
    const timebase_scale::Float64   # reference samples per one of this band's samples
    noise_channel::Int
    noise_prn::Int
    noise_replica_amplitude::Float64
    noise_code_amplitude::Float64
    noise_power::Float64
    noise_looks::Int
    noise_samples_per_look::Int
    noise_epochs_since_rearm::Int
    noise_density::Float64          # the window's mean density, 1/Hz, 0 for none
    noise_density_ready::Bool
    noise_window::Vector{Float64}   # ring of per-epoch densities
    noise_window_count::Int
    noise_window_index::Int
    noise_rng::UInt64               # LCG state for the decoy dither
end

const NOISE_WINDOW_LENGTH = 1000

function BandState(entry::BandEntry, reference_freq::Float64)
    BandState(
        entry,
        reference_freq / entry.sampling_freq_hz,
        0,
        0,
        1.0,
        1.0,
        0.0,
        0,
        0,
        0,
        0.0,
        false,
        zeros(Float64, NOISE_WINDOW_LENGTH),
        0,
        0,
        0x9E3779B97F4A7C15,
    )
end

# A small linear congruential generator for the noise reference's dither:
# `Random` is neither needed nor wanted in a trimmed binary.
@inline function _next_uniform!(band::BandState)
    band.noise_rng = band.noise_rng * 6364136223846793005 + 1442695040888963407
    (band.noise_rng >> 11) * (1.0 / 9007199254740992.0)
end

# Per-channel scalar state, all fixed-size vectors indexed by hardware channel.
# `E` is the tracking loop's per-satellite loop state type.
struct ChannelTable{E}
    # ── Occupancy ─────────────────────────────────────────────────────────────
    armed::Vector{Bool}
    confirmed::Vector{Bool}          # the device confirmed the arm (records are believed)
    bank::Vector{Int}                # which signal bank; 0 while free
    band::Vector{Int}                # which band
    prn::Vector{Int}
    signal_index::Vector{Int}        # 0 = noise reference, 1 = driver, 2+ = passenger
    group_key::Vector{FixedName}
    arm_sequence::Vector{UInt64}     # the command sequence the arm answers
    want_taps::Vector{Bool}
    driver_channel::Vector{Int}      # a passenger's driver channel (0 for a driver)
    carrier_phase_offset::Vector{Float64}  # the signal's carrier-phase offset, cycles
    command_dirty::Vector{Bool}      # the loop stepped since the last command was scheduled
    scale::Vector{Float64}           # accumulator amplitude divisor (replica × code amplitude ratio)
    sampling_frequency::Vector{Float64}   # the channel's band rate, Hz
    # ── Loop state ────────────────────────────────────────────────────────────
    loop_state::Vector{E}
    carrier_doppler::Vector{Float64}   # Hz, the loop's current command
    code_doppler::Vector{Float64}
    timeline::Vector{NCOTimeline}
    # A command waiting for its device sample.
    pending_command_sample::Vector{Int64}
    pending_carrier::Vector{Float64}
    pending_code::Vector{Float64}
    # ── Record continuity and the code period grid ────────────────────────────
    last_record_end::Vector{Int64}
    last_record_samples::Vector{Int64}
    nominal_record_samples::Vector{Int64}
    code_period_phase::Vector{Float64}
    primary_wraps::Vector{Int64}
    lost_record_samples::Vector{Int64}
    rearm_dead_samples::Vector{Int64}
    bit_clock_lost::Vector{Bool}
    # ── The open predetection integration ─────────────────────────────────────
    partial_samples::Vector{Int64}
    partial_periods::Vector{Float64}
    partial_wraps::Vector{Int}
    partial_end::Vector{Int64}
    partial_first::Vector{Bool}      # the open integration is the first after the arm
    partial_code_phase::Vector{Float64}  # the device's code phase at `partial_end`, NaN if unreported
    # ── Secondary code wipe-off ───────────────────────────────────────────────
    secondary_wipe::Vector{Bool}
    secondary_code_phase::Vector{Int}
    secondary_code_phase_sample::Vector{Int64}
    # ── Absolute code phase (pseudoranges) ────────────────────────────────────
    code_phase::Vector{Float64}      # chips, at `phase_ref_sample`
    phase_ref_sample::Vector{Int64}
    anchor_sample::Vector{Int64}
    anchor_code_phase::Vector{Float64}
    bit_phase_anchored::Vector{Bool}
    carrier_phase::Vector{Float64}   # cycles, dead-reckoned from the commands
    # ── Bits ──────────────────────────────────────────────────────────────────
    bit_index::Vector{Int64}
    # ── Counters ──────────────────────────────────────────────────────────────
    records_folded::Vector{Int64}
    stale_records::Vector{Int64}
end

function ChannelTable(n::Integer, loop_state_template::E) where {E}
    ChannelTable{E}(
        fill(false, n), fill(false, n), zeros(Int, n), ones(Int, n), zeros(Int, n), zeros(Int, n),
        [FixedName() for _ = 1:n], zeros(UInt64, n), fill(false, n), zeros(Int, n), zeros(Float64, n), fill(false, n), ones(Float64, n), zeros(Float64, n),
        [loop_state_template for _ = 1:n], zeros(Float64, n), zeros(Float64, n),
        [NCOTimeline() for _ = 1:n], fill(typemin(Int64), n), zeros(Float64, n), zeros(Float64, n),
        fill(typemin(Int64), n), fill(typemin(Int64), n), fill(typemin(Int64), n), fill(NaN, n),
        zeros(Int64, n), zeros(Int64, n), zeros(Int64, n), fill(false, n),
        zeros(Int64, n), zeros(Float64, n), zeros(Int, n), fill(typemin(Int64), n), fill(false, n), fill(NaN, n),
        fill(false, n), fill(-1, n), fill(typemin(Int64), n),
        zeros(Float64, n), fill(typemin(Int64), n), fill(typemin(Int64), n), fill(NaN, n), fill(false, n), zeros(Float64, n),
        zeros(Int64, n), zeros(Int64, n), zeros(Int64, n),
    )
end

"""
    LoopCore(driver, signals, segment; tracking_loop = DelayCompensatedTrackingLoop(),
             config = nothing, max_pending_records = 65536, num_antennas = NumAntennas(1))

The loop process's whole state: the driver, the signal banks, the channel
table, the bands, the protocol segment it publishes into, the epoch clock and
the counters. `signals` is the tuple of signal objects this loop can track
(fixed at construction, so a trimmed binary knows every type it needs);
`segment` is the `HardwareLoopProtocol.Segment` the receiver attaches to, with
one event ring per hardware channel of the driver.

  - `tracking_loop` — the `TrackingLoops` tracking loop every satellite channel
    steps: any whose `step_loop!` takes an NCO timeline and an application
    sample index, by default the `DelayCompensatedTrackingLoop`. With a
    `VectorTrackingLoop` the core runs vector tracking: every satellite shares
    the loop's navigation engine, the segment's `navigation_mode` is
    `NAV_VECTOR` and each navigation cycle is published on the segment's nav
    ring. Its `scalar_loop` must be a `DelayCompensatedTrackingLoop`, its
    `filter_config` a `NavigationFilterConfig` (not `nothing`), and it must
    list every signal of `signals` that carries data.
  - `config` — a [`LoopConfig`](@ref); by default one whose epoch is a primary
    code period of the first signal on the reference band.
  - `max_pending_records` — the capacity of the ingest buffers; records past it
    are dropped and counted.
  - `num_antennas` — the antenna blocks per record as a static `NumAntennas`;
    must match [`driver_capabilities`](@ref)`(driver).num_antennas`.

Drive it with [`service_pass!`](@ref) or [`run!`](@ref).
"""
mutable struct LoopCore{D<:AbstractLoopDriver,Banks<:Tuple,E<:AbstractTrackingLoop,CT<:ChannelTable}
    const driver::D
    const banks::Banks
    const channels::CT
    const bands::Vector{BandState}
    const segment::Segment
    const config::LoopConfig
    const tracking_loop::E
    # Per bank, whether a vector tracking loop lists its signal: only those
    # may be armed as drivers in vector mode.
    const vector_banks::Vector{Bool}
    const num_channels::Int
    const num_antennas::Int
    # Records read from the driver and not yet folded (they belong to an epoch
    # that has not closed). Sized once.
    const pending::Vector{DeviceRecord}
    const incoming::Vector{DeviceRecord}
    const max_pending::Int
    # The epoch being folded: where its NCO commands take effect, and whether it is a stale
    # backlog folded for observation only.
    application_sample_index::Int64
    observation_only::Bool
    # The epoch grid on the reference band's counter.
    next_epoch_boundary::Int64
    latest_sample_index::Int64
    implausible_index_candidate::Int64
    epochs_folded::Int64
    # Commands.
    commands_handled::Int64
    running::Bool
    # Counters (published as status where they matter).
    dropped_records::Int64
    stale_dumps::Int64
    implausible_dumps::Int64
    skipped_epochs::Int64
    lost_record_gaps::Int64
    rearm_gaps::Int64
    tap_layout_mismatches::Int64
    events_published::Int64
    nco_commands_committed::Int64
    nco_commands_late::Int64
    nco_commands_rejected::Int64
    max_pass_ns::Int64
    # Navigation (vector mode): the latest cycle published, how many nav events
    # were, and the longest pass that published a cycle (the cycle itself runs
    # inside the `step_loop!` of one predetection integration).
    last_nav_cycle::Int
    nav_events_published::Int64
    max_nav_cycle_ns::Int64
    # Record age at the fold — the device counter read at the start of the pass
    # minus the newest record's end — is the record-to-command latency, since
    # the NCO commands are committed in the same pass. Histogram over `LATENCY_EDGES_US`
    # (the last bin overflows) and the maximum, in microseconds.
    const latency_hist::Vector{Int64}
    max_record_age_us::Int64
end

"Record-age histogram edges in microseconds; ages past the last edge land in the overflow bin."
const LATENCY_EDGES_US = (250, 500, 1000, 2000, 3000, 4000, 8000, 16000, 50000)

@inline function _latency_bin(age_us::Int64)
    for (i, edge) in enumerate(LATENCY_EDGES_US)
        age_us < edge && return i
    end
    length(LATENCY_EDGES_US) + 1
end

function LoopCore(
    driver::AbstractLoopDriver,
    signals::Tuple{AbstractGNSSSignal,Vararg{AbstractGNSSSignal}},
    segment::Segment;
    tracking_loop::AbstractTrackingLoop = DelayCompensatedTrackingLoop(),
    config::Union{Nothing,LoopConfig} = nothing,
    max_pending_records::Integer = 1 << 16,
    # Antenna blocks per record, as a static count: the banks' correlator
    # types depend on it, and a trimmed binary needs them known at build time.
    num_antennas::NumAntennas{N} = NumAntennas(1),
) where {N}
    caps = driver_capabilities(driver)
    band_table(segment) == caps.bands || throw(ArgumentError(
        "the segment's band table differs from the driver's bands (ids, rates, intermediate frequencies, " *
        "RF inputs); create the segment with `bands = driver_capabilities(driver).bands`"))
    n = caps.num_channels
    reference = first(caps.bands)
    cfg = something(
        config,
        LoopConfig(;
            epoch_length = round(
                Int64,
                get_code_length(first(signals)) * reference.sampling_freq_hz / _chip_rate_hz(first(signals)),
            ),
        ),
    )
    caps.num_antennas == N ||
        throw(ArgumentError("the driver reads $(caps.num_antennas) antenna block(s) per record but the core was built for $N"))
    vector_banks = _check_tracking_loop(tracking_loop, signals)
    banks = map(signal -> ChannelBank(signal, n, num_antennas), signals)
    template = init_loop_state(tracking_loop, first(signals), 0.0Hz, 0.0Hz)
    pending = DeviceRecord[]
    sizehint!(pending, max_pending_records)
    incoming = DeviceRecord[]
    sizehint!(incoming, max_pending_records)
    set_navigation_mode!(segment, _navigation_mode(tracking_loop))
    LoopCore(
        driver,
        banks,
        ChannelTable(n, template),
        [BandState(entry, reference.sampling_freq_hz) for entry in caps.bands],
        segment,
        cfg,
        tracking_loop,
        vector_banks,
        n,
        caps.num_antennas,
        pending,
        incoming,
        Int(max_pending_records),
        typemin(Int64),
        false,
        typemin(Int64),
        typemin(Int64),
        typemin(Int64),
        0,
        0,
        true,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0,
        zeros(Int64, length(LATENCY_EDGES_US) + 1),
        0,
    )
end

# ── The tracking loop ────────────────────────────────────────────────────────

# Whether the core runs vector tracking: every satellite shares the tracking
# loop's navigation engine, which the core publishes.
_is_vector(::AbstractTrackingLoop) = false
_is_vector(::VectorTrackingLoop) = true
@inline _is_vector(core::LoopCore) = _is_vector(core.tracking_loop)

_navigation_mode(tracking_loop::AbstractTrackingLoop) =
    _is_vector(tracking_loop) ? HardwareLoopProtocol.NAV_VECTOR : HardwareLoopProtocol.NAV_NONE

# Whether a vector tracking loop was built for `signal` as a driver, and whether
# it decodes `signal` (a plain data signal, or a pilot's data component).
_lists_signal(tracking_loop::VectorTrackingLoop, signal::AbstractGNSSSignal) =
    any(pair -> typeof(first(pair)) === typeof(signal), get_signal_pairs(tracking_loop))
_decodes_signal(tracking_loop::VectorTrackingLoop, signal::AbstractGNSSSignal) =
    any(pair -> typeof(last(pair)) === typeof(signal), get_signal_pairs(tracking_loop))

# Check the tracking loop against the core's signals, and return per bank
# whether it may drive a satellite in vector mode.
_check_tracking_loop(::AbstractTrackingLoop, signals::Tuple) = fill(false, length(signals))
function _check_tracking_loop(tracking_loop::VectorTrackingLoop, signals::Tuple)
    get_scalar_loop(tracking_loop) isa DelayCompensatedTrackingLoop || throw(ArgumentError(
        "vector tracking in the loop process needs the delay-compensated loop as its scalar loop: " *
        "`VectorTrackingLoop(signals...; scalar_loop = DelayCompensatedTrackingLoop())`"))
    is_vector_tracking_enabled(tracking_loop) || throw(ArgumentError(
        "a `VectorTrackingLoop` built with `filter_config = nothing` only solves a scalar PVT; " *
        "the loop process runs either vector tracking or the scalar loops, which publish no PVT"))
    listed = Bool[_lists_signal(tracking_loop, signal) for signal in signals]
    for (signal, ok) in zip(signals, listed)
        ok || iszero(get_bit_rate(signal)) || _decodes_signal(tracking_loop, signal) ||
            throw(ArgumentError(
                "the vector tracking loop does not list $(nameof(typeof(signal))), " *
                "which the core can arm as a driver; build it with every data signal of " *
                "the core, plain or as the data component of a `pilot => data` pair"))
    end
    listed
end

# ── Bank dispatch ────────────────────────────────────────────────────────────
#
# A channel's signal-typed state lives in bank `channels.bank[ch]`. The walk
# below is a compile-time unrolled recursion over the banks tuple, so calling
# `f(bank, args...)` on the right bank costs an integer compare per bank and no
# dynamic dispatch — the shape Julia's union-splitting needs to keep the fold
# allocation-free with heterogeneous signal types.
@inline _with_bank(f::F, ::Tuple{}, index::Int, k::Int, args::Vararg{Any,N}) where {F,N} = nothing
@inline function _with_bank(f::F, banks::Tuple, index::Int, k::Int, args::Vararg{Any,N}) where {F,N}
    if index == k
        return f(first(banks), args...)
    end
    _with_bank(f, Base.tail(banks), index, k + 1, args...)
end
@inline with_bank(f::F, core::LoopCore, channel::Integer, args::Vararg{Any,N}) where {F,N} =
    _with_bank(f, core.banks, core.channels.bank[channel], 1, args...)

# One of a channel's device samples on the reference timebase, and back.
@inline _to_reference(core::LoopCore, band::Int, sample) =
    _scale_sample(sample, core.bands[band].timebase_scale)
@inline _to_band(core::LoopCore, band::Int, sample) =
    _scale_sample(sample, 1 / core.bands[band].timebase_scale)
@inline function _scale_sample(sample, scale::Float64)
    n = Int64(sample)
    (scale == 1.0 || n == typemin(Int64) || n == typemax(Int64)) && return n
    round(Int64, n * scale)
end
