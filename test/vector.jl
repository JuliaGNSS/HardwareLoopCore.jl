# Vector tracking and the generic estimator: the records name their satellite,
# vector-mode arms and constructions are checked, the engine is kept fed through
# a stale backlog, a re-armed channel hands its slot over, and every navigation
# cycle reaches the nav ring — while a scalar core publishes none.

# An estimator that remembers what every record told it, then runs the
# delay-aware loop: a host-side stand-in for an estimator that keeps
# per-satellite state of its own.
const SeenRecord = NamedTuple{(:prn, :code_phase, :sample_index, :signal),Tuple{Int,Float64,Int,Symbol}}
struct RecordingEstimator{E} <: TrackingLoops.AbstractDopplerEstimator
    inner::E
    seen::Vector{SeenRecord}
end
RecordingEstimator() = RecordingEstimator(NCOReferencedPLLAndDLL(), SeenRecord[])
TrackingLoops.init_estimator_state(e::RecordingEstimator, signal::AbstractGNSSSignal, carrier, code) =
    TrackingLoops.init_estimator_state(e.inner, signal, carrier, code)
function TrackingLoops.step_loop(e::RecordingEstimator, state, record::TrackingLoops.LoopRecord, words, landing::Int64)
    push!(e.seen, (; record.prn, record.code_phase, record.sample_index, signal = get_signal_id(record.signal)))
    TrackingLoops.step_loop(e.inner, state, record, words, landing)
end

vector_estimator(signals...; kwargs...) =
    VectorPLLAndDLL(signals...; inner = NCOReferencedPLLAndDLL(), approximate_year = 2017, kwargs...)

# Every event on the loop-wide nav ring, typed.
function drain_nav!(seg)
    ring = nav_ring(seg)
    solutions = Tuple{EventTag,NavSolutionEvent}[]
    satellites = Tuple{EventTag,NavSatelliteEvent}[]
    kinds = UInt8[]
    while true
        status, view, _ = peek!(ring, EventTag)
        status === :empty && break
        push!(kinds, view.tag.kind)
        if view.tag.kind == HLP.EVENT_NAV_SOLUTION
            push!(solutions, (view.tag, payload(NavSolutionEvent, ring, view)))
        elseif view.tag.kind == HLP.EVENT_NAV_SATELLITE
            push!(satellites, (view.tag, payload(NavSatelliteEvent, ring, view)))
        end
        commit!(ring, view)
    end
    (; solutions, satellites, kinds)
end

@testset "Records name their satellite and its code phase" begin
    estimator = RecordingEstimator()
    f = scripted_fixture(; estimator)
    @test f.core.estimator === estimator
    arm!(f, 1, 7; doppler = 0.0, code_phase = 0.0, sequence = 1)
    service_pass!(f.core; wait_ms = 0)
    # The device reports the replica's code phase (a chip past the block
    # boundary, inside the grid's tolerance): the record carries it.
    # The last record waits for its epoch to close.
    feed_epl!(f, 1, 7, 3; code_phase = 1.0)
    @test [s.prn for s in estimator.seen] == [7, 7]
    @test all(s.code_phase == 1.0 for s in estimator.seen)
    @test [s.sample_index for s in estimator.seen] == [1, 2] .* CORE_EPOCH
    # A device that stops reporting it: the core's own absolute code phase,
    # anchored by the reports, advanced to the record's end — on the block
    # boundary these records end on.
    empty!(estimator.seen)
    feed_epl!(f, 1, 7, 3; first = 4, code_phase = NaN)
    @test [s.sample_index for s in estimator.seen] == [3, 4, 5] .* CORE_EPOCH
    @test all(s.prn == 7 for s in estimator.seen)
    @test all(!isnan(s.code_phase) for s in estimator.seen)
    @test all(abs(rem(s.code_phase - 1.0, 1023, RoundNearest)) < 1e-6 for s in estimator.seen)
    # A scalar core keeps its navigation area empty.
    @test navigation_mode(f.seg) == HLP.NAV_NONE
    @test isempty(drain_nav!(f.seg).kinds)
    @test isnothing(read_nav_snapshot(f.seg))
end

@testset "A vector core is built for every data signal it can drive" begin
    @test_throws "inner estimator" scripted_fixture(; estimator = VectorPLLAndDLL(GPSL1CA(); approximate_year = 2017))
    @test_throws "config = nothing" scripted_fixture(; estimator = vector_estimator(GPSL1CA(); config = nothing))
    @test_throws "does not list GalileoE1B" scripted_fixture((GPSL1CA(), GalileoE1B()); estimator = vector_estimator(GPSL1CA()))
    # A pilot bank needs no listing: it is never armed as a driver.
    f = scripted_fixture((GPSL1CA(), GPSL1C_P()); estimator = vector_estimator(GPSL1CA()))
    @test navigation_mode(f.seg) == HLP.NAV_VECTOR
    @test f.core.vector_banks == [true, false]
end

@testset "Vector mode refuses an unpaired pilot as a driver and serves passengers" begin
    f = scripted_fixture((GPSL1CA(), GPSL1C_P()); estimator = vector_estimator(GPSL1CA()))
    arm!(f, 1, 7; doppler = 0.0, code_phase = 0.0, sequence = 1, signal = "GPSL1C_P")
    # A driver of a listed signal, passengers of any signal and a noise
    # reference of any are served.
    arm!(f, 2, 7; doppler = 0.0, code_phase = 0.0, sequence = 2, signal = "GPSL1C_P", signal_index = 2, group_key = "sat7")
    arm!(f, 3, 7; doppler = 0.0, code_phase = 0.0, sequence = 3, signal_index = 2, group_key = "sat7")
    arm!(f, 4, 7; doppler = 0.0, code_phase = 0.0, sequence = 4, group_key = "sat7")
    service_pass!(f.core; wait_ms = 0)
    rejected = only(statuses(f, 1))
    @test rejected.code == HLP.STATUS_ARM_REJECTED && rejected.reason == HLP.REJECT_UNSUPPORTED_SIGNAL
    @test all(only(statuses(f, ch)).code == HLP.STATUS_ARMED for ch = 2:4)
    @test f.core.channels.armed == [false, true, true, true]
    @test f.core.channels.driver_channel[2:3] == [4, 4]
    g = scripted_fixture((GPSL1CA(), GPSL1C_P()); estimator = vector_estimator(GPSL1CA()))
    arm!(g, 1, 30; doppler = 0.0, code_phase = 0.0, sequence = 1, signal = "GPSL1C_P", signal_index = 0)
    service_pass!(g.core; wait_ms = 0)
    @test only(statuses(g, 1)).code == HLP.STATUS_ARMED
    @test g.core.bands[1].noise_channel == 1
end

@testset "Every navigation cycle reaches the nav ring, its satellites first" begin
    estimator = vector_estimator(GPSL1CA())
    f = scripted_fixture(; estimator)
    arm!(f, 1, 7; doppler = 0.0, code_phase = 0.0, sequence = 1)
    arm!(f, 2, 9; doppler = 0.0, code_phase = 0.0, sequence = 2)
    service_pass!(f.core; wait_ms = 0)
    for k = 1:450
        f.dev.sample_count = Int64(k) * CORE_EPOCH
        bit = isodd(div(k - 1, 20)) ? -1.0 : 1.0
        push!(f.dev.queue, epl_record(1, 7, f.dev.sample_count, bit; code_phase = 0.0))
        push!(f.dev.queue, epl_record(2, 9, f.dev.sample_count, -bit; code_phase = 0.0))
        push!(f.dev.queue, strobe_record(f.dev.sample_count))
        service_pass!(f.core; wait_ms = 0)
    end
    nav = drain_nav!(f.seg)
    cycles = navigation_cycle(estimator)
    # One cycle every 100 ms of records (the first epoch is the first one a
    # satellite reaches after joining).
    @test cycles >= 3
    @test f.core.last_nav_cycle == cycles
    @test [s.cycle for (_, s) in nav.solutions] == 1:cycles
    @test length(nav.satellites) == 2cycles
    @test f.core.nav_events_published == 3cycles
    @test nav.kinds == repeat([HLP.EVENT_NAV_SATELLITE, HLP.EVENT_NAV_SATELLITE, HLP.EVENT_NAV_SOLUTION], cycles)
    @test f.core.max_nav_cycle_ns > 0
    # The epochs are the multiples of the cycle time on the reference counter.
    @test all(tag.device_sample % 400_000 == 0 && tag.channel == 0 for (tag, _) in nav.solutions)
    @test issorted([tag.device_sample for (tag, _) in nav.solutions])
    for ((tag, sat), prn, ch) in zip(nav.satellites[end-1:end], (7, 9), (1, 2))
        @test tag.prn == prn && tag.band == 1 && tag.channel == 0
        @test sat.channel == ch && sat.signal == FixedName(:GPSL1CA) && sat.cycle == cycles
        @test sat.flags & HLP.NAV_SAT_TRACKED != 0 && sat.flags & HLP.NAV_SAT_BIT_SYNCED != 0
        # Nothing decoded, so no solution and no satellite in it.
        @test sat.flags & HLP.NAV_SAT_IN_SOLUTION == 0 && isnan(sat.residual_m)
    end
    _, last_solution = nav.solutions[end]
    @test last_solution.flags & (HLP.NAV_VALID | HLP.NAV_SEEDED | HLP.NAV_RUNNING) == 0
    @test last_solution.num_sats == 0 && all(isnan, last_solution.dop)
    tag, snapshot = read_nav_snapshot(f.seg)
    @test snapshot == last_solution && tag == nav.solutions[end][1]
end

@testset "A stale backlog still feeds the vector engine" begin
    for (estimator, steps) in ((NCOReferencedPLLAndDLL(), false), (vector_estimator(GPSL1CA()), true))
        f = scripted_fixture(; estimator)
        arm!(f, 1, 7; doppler = 0.0, code_phase = 0.0, sequence = 1)
        service_pass!(f.core; wait_ms = 0)
        before = f.core.channels.estimator[1]
        # Ten records the pass reads twenty epochs after they ended.
        feed_epl!(f, 1, 7, 10; passes = false, code_phase = 0.0)
        f.dev.sample_count += 20CORE_EPOCH
        service_pass!(f.core; wait_ms = 0)
        # Every epoch but the last, which waits for a later record, is closed
        # observation-only.
        @test f.core.skipped_epochs == 9
        @test (f.core.channels.estimator[1] != before) == steps
    end
end

@testset "A channel re-armed onto another PRN drops the old one from the engine" begin
    estimator = vector_estimator(GPSL1CA())
    f = scripted_fixture(; estimator)
    arm!(f, 1, 7; doppler = 0.0, code_phase = 0.0, sequence = 1)
    service_pass!(f.core; wait_ms = 0)
    feed_epl!(f, 1, 7, 300; code_phase = 0.0)
    @test satellite_report(estimator, GPSL1CA(), 7).tracked
    arm!(f, 1, 9; doppler = 0.0, code_phase = 0.0, sequence = 2)
    service_pass!(f.core; wait_ms = 0)
    drain_nav!(f.seg)
    feed_epl!(f, 1, 9, 300; first = 301, code_phase = 0.0)
    # Two cycles without a record drop PRN 7; PRN 9 is stepped on the channel.
    @test !satellite_report(estimator, GPSL1CA(), 7).tracked
    @test satellite_report(estimator, GPSL1CA(), 9).tracked
    @test !haskey(member_sats(estimator), (:GPSL1CA, 7))
    nav = drain_nav!(f.seg)
    @test !isempty(nav.satellites)
    @test all(tag.prn == 9 && sat.channel == 1 for (tag, sat) in nav.satellites)
    # The engine's storage was sized at construction and is reused.
    @test length(estimator.navigation.groups[1].slots) == 16
end

# A pilot + data satellite on the scripted device: GPS L1C-P (10 ms code blocks,
# five taps) drives, GPS L1 C/A (1 ms, LNAV-style bits every 20 ms) is the
# passenger. Each signal's prompt sits at its carrier phase offset against the
# driver's. The core's banks are `(GPSL1CA(), GPSL1C_P())`, so its epoch is a
# C/A code period.
const PAIR_BANKS = (GPSL1CA(), GPSL1C_P())

function arm_pair!(f, prn)
    pilot = f.core.banks[2].template
    arm!(f, 1, prn; doppler = 0.0, code_phase = 0.0, sequence = 1, signal = "GPSL1C_P", group_key = "sat$prn",
        num_taps = TrackingLoops.get_num_accumulators(pilot),
        tap_sample_shifts = HardwareLoopCore._template_tap_shifts(pilot, CORE_FS, GPSL1C_P()))
    arm!(f, 2, prn; doppler = 0.0, code_phase = 0.0, sequence = 2, signal = "GPSL1CA", signal_index = 2,
        group_key = "sat$prn")
end

function feed_pair!(f, driver_ch, passenger_ch, prn, count; first = 1)
    rotation = cis(get_carrier_phase_offset(GPSL1CA()) - get_carrier_phase_offset(GPSL1C_P()))
    for k = first:(first+count-1)
        f.dev.sample_count = Int64(k) * CORE_EPOCH
        bit = isodd(div(k - 1, 20)) ? -1.0 : 1.0
        prompt = rotation * bit * CORE_EPOCH
        push!(f.dev.queue, DeviceRecord(passenger_ch, prn, f.dev.sample_count, CORE_EPOCH,
            pack_taps(ComplexF64[prompt / 2, prompt, prompt / 2]), 3; code_phase = 0.0))
        if k % 10 == 0
            pilot = 10.0 * CORE_EPOCH
            push!(f.dev.queue, DeviceRecord(driver_ch, prn, f.dev.sample_count, 10CORE_EPOCH,
                pack_taps(ComplexF64[0.2pilot, 0.7pilot, pilot, 0.7pilot, 0.2pilot]), 5; code_phase = 0.0))
        end
        push!(f.dev.queue, strobe_record(f.dev.sample_count))
        service_pass!(f.core; wait_ms = 0)
    end
    nothing
end

@testset "A passenger's records step its driver channel's estimator state" begin
    estimator = RecordingEstimator()
    f = scripted_fixture(PAIR_BANKS; estimator)
    arm_pair!(f, 7)
    service_pass!(f.core; wait_ms = 0)
    @test f.core.channels.driver_channel[2] == 1
    template = f.core.channels.estimator[2]
    feed_pair!(f, 1, 2, 7, 60)
    # Both channels' records reached the estimator…
    @test count(s -> s.signal == :GPSL1CA, estimator.seen) >= 50
    @test count(s -> s.signal == :GPSL1C_P, estimator.seen) >= 5
    @test all(s -> s.prn == 7, estimator.seen)
    # …with the satellite's one state, kept in the driver channel's slot.
    @test f.core.channels.estimator[2] === template
    @test f.core.channels.estimator[1] != template
end

@testset "A vector core ranges on a pilot and decodes its passenger data component" begin
    estimator = vector_estimator(GPSL1C_P() => GPSL1CA())
    f = scripted_fixture(PAIR_BANKS; estimator)
    # The data signal is decoded, not driven: the core is built, and only the
    # pilot may drive.
    @test f.core.vector_banks == [false, true]
    arm_pair!(f, 7)
    service_pass!(f.core; wait_ms = 0)
    @test all(only(statuses(f, ch)).code == HLP.STATUS_ARMED for ch = 1:2)
    feed_pair!(f, 1, 2, 7, 450)
    # The satellite is known by its pilot, and its bit clock is the data
    # component's: the passenger channel's bits synced it.
    report = satellite_report(estimator, GPSL1C_P(), 7)
    @test report.tracked && report.bit_synced
    @test isnothing(satellite_report(estimator, GPSL1CA(), 7))
    @test estimator.navigation.registrations == 1
    @test navigation_cycle(estimator) >= 3
    nav = drain_nav!(f.seg)
    @test !isempty(nav.satellites)
    @test all(sat.signal == FixedName(:GPSL1C_P) && sat.channel == 1 for (_, sat) in nav.satellites)
    @test all(sat.flags & HLP.NAV_SAT_BIT_SYNCED != 0 for (_, sat) in nav.satellites[end-1:end])
end
