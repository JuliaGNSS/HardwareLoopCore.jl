# ─────────────────────────────────────────────────────────────────────────────
# Vector tracking: publishing the navigation solution. With a `VectorTrackingLoop`
# every satellite's `step_loop!` feeds one shared navigation processor, and the
# integration that brings the last satellite past a measurement epoch runs that
# epoch's navigation update. After each closed NCO update span the core looks for a
# new update and publishes it on the segment's loop-wide nav ring: one
# `NavSatelliteEvent` per armed driver channel, then the `NavSolutionEvent`,
# which is also the nav snapshot.
# The scalar tracking loops compute no solution and publish nothing.
# ─────────────────────────────────────────────────────────────────────────────

_publish_navigation!(::LoopCore, ::AbstractTrackingLoop) = nothing

function _publish_navigation!(core::LoopCore, tracking_loop::VectorTrackingLoop)
    update_index = get_navigation_update_index(tracking_loop)
    update_index > core.last_nav_update_index || return nothing
    core.last_nav_update_index = update_index
    # The measurement epoch is on the integrations' time grid, seconds since the origin
    # the band counters share: on the reference band's counter, its sample.
    epoch = get_navigation_elapsed_time(tracking_loop)
    epoch_sample =
        isnothing(epoch) ? Int64(0) :
        round(Int64, ustrip(Unitful.s, epoch) * core.bands[1].entry.sampling_frequency_hz) -
        _sample_index_offset(core, 1)
    pvt = get_navigation_solution(tracking_loop)
    T = core.channels
    for ch = 1:core.num_channels
        T.armed[ch] && T.confirmed[ch] && T.signal_index[ch] == 1 || continue
        with_bank(_publish_nav_satellite!, core, ch, core, tracking_loop, ch, update_index, epoch_sample, pvt)
    end
    publish_nav_solution!(
        core.segment,
        EventTag(HardwareLoopProtocol.EVENT_NAV_SOLUTION, 0, epoch_sample),
        _nav_solution_event(update_index, pvt, get_navigation_status(tracking_loop)),
    )
    core.nav_events_published += 1
    nothing
end

const _NO_POSITION = (NaN, NaN, NaN)

# Why this update released the satellite, on the wire: TrackingLoops keeps a
# satellite's latest release in its report until another satellite takes the
# slot, dated by the update that made it, so an older release is no release now.
_release_reason(report, update_index::Int) =
    report.release_update_index == update_index ? UInt8(Int(report.release_reason)) :
    HardwareLoopProtocol.NAV_NOT_RELEASED

function _publish_nav_satellite!(
    bank::ChannelBank,
    core::LoopCore,
    tracking_loop::VectorTrackingLoop,
    ch::Int,
    update_index::Int,
    epoch_sample::Int64,
    pvt,
)
    T = core.channels
    prn = T.prn[ch]
    report = get_satellite_report(tracking_loop, bank.signal, prn)
    isnothing(report) && return nothing
    flags = UInt8(0)
    report.bit_or_secondary_code_synced && (flags |= HardwareLoopProtocol.NAV_SAT_BIT_OR_SECONDARY_CODE_SYNCED)
    report.measurement_ready && (flags |= HardwareLoopProtocol.NAV_SAT_MEASUREMENT_READY)
    report.positioning_ready && (flags |= HardwareLoopProtocol.NAV_SAT_POSITIONING_READY)
    report.vector_controlled && (flags |= HardwareLoopProtocol.NAV_SAT_VECTOR_CONTROLLED)
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
            Int64(update_index),
            bank.name,
            position,
            transmit_time,
            residual,
            rate_residual,
            _dbhz(report.cn0),
            UInt16(ch),
            flags,
            _release_reason(report, update_index),
            UInt32(0),
        ),
    )
    core.nav_events_published += 1
    nothing
end

# PositionVelocityTime's solution carries no receiver clock bias (its epoch,
# `time`, is the receiver clock corrected), and neither does the event.
function _nav_solution_event(update_index::Int, pvt, status)
    position = pvt.position
    velocity = pvt.velocity
    time = pvt.time
    dop = pvt.dop
    flags = UInt32(0)
    status.vector_tracking_active && (flags |= HardwareLoopProtocol.NAV_VECTOR_TRACKING_ACTIVE)
    status.solution_from_nav_filter && (flags |= HardwareLoopProtocol.NAV_SOLUTION_FROM_NAV_FILTER)
    status.fell_back && (flags |= HardwareLoopProtocol.NAV_FELL_BACK)
    status.released && (flags |= HardwareLoopProtocol.NAV_RELEASED)
    iszero(position.x) && iszero(position.y) && iszero(position.z) || (flags |= HardwareLoopProtocol.NAV_VALID)
    isnothing(time) || (flags |= HardwareLoopProtocol.NAV_TIME_VALID)
    NavSolutionEvent(
        Int64(update_index),
        (position.x, position.y, position.z),
        (velocity.x, velocity.y, velocity.z),
        ustrip(Unitful.m / Unitful.s, pvt.clock_drift),
        isnothing(time) ? Int64(0) : time.second,
        isnothing(time) ? NaN : time.fraction,
        ustrip(Unitful.m, status.position_std),
        ustrip(Unitful.m, status.clock_bias_std),
        ustrip(Unitful.s, status.measurement_deficit_duration),
        isnothing(dop) ? ntuple(_ -> NaN32, Val(5)) :
        (Float32(dop.GDOP), Float32(dop.PDOP), Float32(dop.HDOP), Float32(dop.VDOP), Float32(dop.TDOP)),
        Int32(status.num_vector_tracking_members),
        Int32(length(pvt.observations)),
        flags,
    )
end
