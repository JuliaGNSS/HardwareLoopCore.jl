# ─────────────────────────────────────────────────────────────────────────────
# Vector tracking: publishing the navigation solution. With a `VectorTrackingLoop`
# every satellite's `step_loop!` feeds one shared navigation engine, and the
# predetection integration that brings the last satellite past a navigation
# epoch runs that epoch's cycle. After each fold the core looks for a new cycle and publishes
# it on the segment's loop-wide nav ring: one `NavSatelliteEvent` per armed
# driver channel, then the `NavSolutionEvent`, which is also the nav snapshot.
# The scalar tracking loops compute no solution and publish nothing.
# ─────────────────────────────────────────────────────────────────────────────

_publish_navigation!(::LoopCore, ::AbstractTrackingLoop) = nothing

function _publish_navigation!(core::LoopCore, tracking_loop::VectorTrackingLoop)
    cycle = get_navigation_cycle_index(tracking_loop)
    cycle > core.last_nav_cycle || return nothing
    core.last_nav_cycle = cycle
    # The cycle's epoch is on the integrations' time grid, seconds since the origin
    # the band counters share: on the reference band's counter, its sample.
    epoch = get_navigation_elapsed_time(tracking_loop)
    epoch_sample =
        isnothing(epoch) ? Int64(0) :
        round(Int64, ustrip(Unitful.s, epoch) * core.bands[1].entry.sampling_freq_hz) -
        _band_sample_offset(core, 1)
    pvt = get_navigation_solution(tracking_loop)
    T = core.channels
    for ch = 1:core.num_channels
        T.armed[ch] && T.confirmed[ch] && T.signal_index[ch] == 1 || continue
        with_bank(_publish_nav_satellite!, core, ch, core, tracking_loop, ch, cycle, epoch_sample, pvt)
    end
    publish_nav_solution!(
        core.segment,
        EventTag(HardwareLoopProtocol.EVENT_NAV_SOLUTION, 0, epoch_sample),
        _nav_solution_event(cycle, pvt, get_navigation_status(tracking_loop)),
    )
    core.nav_events_published += 1
    nothing
end

const _NO_POSITION = (NaN, NaN, NaN)

function _publish_nav_satellite!(
    bank::ChannelBank,
    core::LoopCore,
    tracking_loop::VectorTrackingLoop,
    ch::Int,
    cycle::Int,
    epoch_sample::Int64,
    pvt,
)
    T = core.channels
    prn = T.prn[ch]
    report = get_satellite_report(tracking_loop, bank.signal, prn)
    isnothing(report) && return nothing
    flags = UInt8(0)
    report.tracked && (flags |= HardwareLoopProtocol.NAV_SAT_TRACKED)
    report.bit_or_secondary_code_synced && (flags |= HardwareLoopProtocol.NAV_SAT_BIT_SYNCED)
    report.measurement_ready && (flags |= HardwareLoopProtocol.NAV_SAT_IN_LOCK)
    report.positioning_ready && (flags |= HardwareLoopProtocol.NAV_SAT_PVT_READY)
    report.vector_controlled && (flags |= HardwareLoopProtocol.NAV_SAT_IN_VECTOR_LOOP)
    # The solution's entry for the satellite, looked up by key: iterating the
    # dictionary would allocate.
    residuals = get(pvt.observations, (get_signal_id(bank.signal), prn), nothing)
    position, transmit_time, residual, rate_residual = if isnothing(residuals)
        _NO_POSITION, NaN, NaN, NaN
    else
        flags |= HardwareLoopProtocol.NAV_SAT_IN_SOLUTION
        (residuals.position.x, residuals.position.y, residuals.position.z),
        ustrip(Unitful.s, residuals.transmit_time), ustrip(Unitful.m, residuals.residual),
        ustrip(Unitful.m / Unitful.s, residuals.rate_residual)
    end
    publish!(
        nav_ring(core.segment),
        EventTag(HardwareLoopProtocol.EVENT_NAV_SATELLITE, 0, epoch_sample; band = T.band[ch], prn),
        NavSatelliteEvent(
            Int64(cycle),
            bank.name,
            position,
            transmit_time,
            residual,
            rate_residual,
            report.cn0_dbhz,
            UInt16(ch),
            flags,
            UInt8(Int(report.release_reason)),
            UInt32(0),
        ),
    )
    core.nav_events_published += 1
    nothing
end

const SPEED_OF_LIGHT = 299_792_458.0 # m/s

# PositionVelocityTime's solution carries no receiver clock bias (its epoch,
# `time`, is the receiver clock corrected), so `clock_bias_m` is `NaN`; its
# clock drift is in m/s, which the protocol's relative `clock_drift` divides by
# the speed of light.
function _nav_solution_event(cycle::Int, pvt, status)
    position = pvt.position
    velocity = pvt.velocity
    time = pvt.time
    dop = pvt.dop
    flags = UInt32(0)
    status.vector_tracking_active && (flags |= HardwareLoopProtocol.NAV_RUNNING)
    # The scalar fixes before the filter is seeded (and after it falls back)
    # are the engine's own; the rest are the filter's.
    (status.vector_tracking_active || status.fell_back) && (flags |= HardwareLoopProtocol.NAV_SEEDED)
    status.fell_back && (flags |= HardwareLoopProtocol.NAV_FELL_BACK)
    status.released && (flags |= HardwareLoopProtocol.NAV_RELEASED)
    iszero(position.x) && iszero(position.y) && iszero(position.z) || (flags |= HardwareLoopProtocol.NAV_VALID)
    isnothing(time) || (flags |= HardwareLoopProtocol.NAV_TIME_VALID)
    NavSolutionEvent(
        Int64(cycle),
        (position.x, position.y, position.z),
        (velocity.x, velocity.y, velocity.z),
        NaN,
        ustrip(Unitful.m / Unitful.s, pvt.clock_drift) / SPEED_OF_LIGHT,
        isnothing(time) ? Int64(0) : time.second,
        isnothing(time) ? NaN : time.fraction,
        ustrip(Unitful.m, status.position_std),
        ustrip(Unitful.m, status.clock_bias_std),
        ustrip(Unitful.s, status.measurement_deficit_duration),
        isnothing(dop) ? ntuple(_ -> NaN32, Val(5)) :
        (Float32(dop.GDOP), Float32(dop.PDOP), Float32(dop.VDOP), Float32(dop.HDOP), Float32(dop.TDOP)),
        Int32(status.num_members),
        Int32(length(pvt.observations)),
        flags,
    )
end
