# ─────────────────────────────────────────────────────────────────────────────
# Vector tracking: publishing the navigation solution. With a `VectorPLLAndDLL`
# every satellite's `step_loop` feeds one shared navigation engine, and the
# record that brings the last satellite past a navigation epoch runs that
# epoch's cycle. After each fold the core looks for a new cycle and publishes
# it on the segment's loop-wide nav ring: one `NavSatelliteEvent` per armed
# driver channel, then the `NavSolutionEvent`, which is also the nav snapshot.
# The scalar estimators compute no solution and publish nothing.
# ─────────────────────────────────────────────────────────────────────────────

_publish_navigation!(::LoopCore, ::AbstractDopplerEstimator) = nothing

function _publish_navigation!(core::LoopCore, estimator::VectorPLLAndDLL)
    cycle = navigation_cycle(estimator)
    cycle > core.last_nav_cycle || return nothing
    core.last_nav_cycle = cycle
    # The cycle's epoch is on the records' time grid, seconds since the origin
    # the band counters share: on the reference band's counter, its sample.
    epoch = navigation_epoch(estimator)
    epoch_sample =
        isnothing(epoch) ? Int64(0) :
        round(Int64, ustrip(Unitful.s, epoch) * core.bands[1].entry.sampling_freq_hz) -
        _band_sample_offset(core, 1)
    pvt = navigation_solution(estimator)
    T = core.channels
    for ch = 1:core.num_channels
        T.armed[ch] && T.confirmed[ch] && T.signal_index[ch] == 1 || continue
        with_bank(_publish_nav_satellite!, core, ch, core, estimator, ch, cycle, epoch_sample, pvt)
    end
    publish_nav_solution!(
        core.segment,
        EventTag(HardwareLoopProtocol.EVENT_NAV_SOLUTION, 0, epoch_sample),
        _nav_solution_event(cycle, pvt, navigation_status(estimator)),
    )
    core.nav_events_published += 1
    nothing
end

const _NO_POSITION = (NaN, NaN, NaN)

function _publish_nav_satellite!(
    bank::ChannelBank,
    core::LoopCore,
    estimator::VectorPLLAndDLL,
    ch::Int,
    cycle::Int,
    epoch_sample::Int64,
    pvt,
)
    T = core.channels
    prn = T.prn[ch]
    report = satellite_report(estimator, bank.signal, prn)
    isnothing(report) && return nothing
    flags = UInt8(0)
    report.tracked && (flags |= HardwareLoopProtocol.NAV_SAT_TRACKED)
    report.bit_synced && (flags |= HardwareLoopProtocol.NAV_SAT_BIT_SYNCED)
    report.in_lock && (flags |= HardwareLoopProtocol.NAV_SAT_IN_LOCK)
    report.pvt_ready && (flags |= HardwareLoopProtocol.NAV_SAT_PVT_READY)
    report.in_vector_loop && (flags |= HardwareLoopProtocol.NAV_SAT_IN_VECTOR_LOOP)
    # The solution's entry for the satellite, looked up by key: iterating the
    # dictionary would allocate.
    info = get(pvt.sats, (get_signal_id(bank.signal), prn), nothing)
    position, time, residual, rate_residual = if isnothing(info)
        _NO_POSITION, NaN, NaN, NaN
    else
        flags |= HardwareLoopProtocol.NAV_SAT_IN_SOLUTION
        (info.position.x, info.position.y, info.position.z), info.time, ustrip(info.residual),
        ustrip(info.rate_residual)
    end
    publish!(
        nav_ring(core.segment),
        EventTag(HardwareLoopProtocol.EVENT_NAV_SATELLITE, 0, epoch_sample; band = T.band[ch], prn),
        NavSatelliteEvent(
            Int64(cycle),
            bank.name,
            position,
            time,
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

function _nav_solution_event(cycle::Int, pvt, status)
    position = pvt.position
    velocity = pvt.velocity
    time = pvt.time
    dop = pvt.dop
    flags = UInt32(0)
    status.running && (flags |= HardwareLoopProtocol.NAV_RUNNING)
    # The scalar fixes before the filter is seeded (and after it falls back)
    # are the engine's own; the rest are the filter's.
    (status.running || status.fell_back) && (flags |= HardwareLoopProtocol.NAV_SEEDED)
    status.fell_back && (flags |= HardwareLoopProtocol.NAV_FELL_BACK)
    status.released && (flags |= HardwareLoopProtocol.NAV_RELEASED)
    iszero(position.x) && iszero(position.y) && iszero(position.z) || (flags |= HardwareLoopProtocol.NAV_VALID)
    isnothing(time) || (flags |= HardwareLoopProtocol.NAV_TIME_VALID)
    NavSolutionEvent(
        Int64(cycle),
        (position.x, position.y, position.z),
        (velocity.x, velocity.y, velocity.z),
        ustrip(Unitful.m, pvt.time_correction),
        Float64(pvt.relative_clock_drift),
        isnothing(time) ? Int64(0) : time.second,
        isnothing(time) ? NaN : time.fraction,
        ustrip(Unitful.m, status.position_std),
        ustrip(Unitful.m, status.clock_std),
        ustrip(Unitful.s, status.time_with_insufficient_meas),
        isnothing(dop) ? ntuple(_ -> NaN32, Val(5)) :
        (Float32(dop.GDOP), Float32(dop.PDOP), Float32(dop.VDOP), Float32(dop.HDOP), Float32(dop.TDOP)),
        Int32(status.num_members),
        Int32(length(pvt.sats)),
        flags,
    )
end
