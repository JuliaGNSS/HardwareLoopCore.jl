# Records the core must not believe, records that never arrived, and the
# timebase's defences — through the scripted driver, record by record.

# A scripted core with a satellite armed and confirmed on channel 1.
function armed_fixture(; prn = 7, kwargs...)
    f = scripted_fixture(; kwargs...)
    arm!(f, 1, prn; doppler = 100.0, code_phase = 0.0, sequence = 1)
    service_pass!(f.core; wait_ms = 0)
    drain_events!(f, 1)
    f
end

@testset "Stale records are dropped and counted" begin
    f = armed_fixture()
    f.dev.sample_count = 4CORE_INTERVAL
    # Channel 2 is free, channel 9 does not exist, PRN 8 is not channel 1's,
    # and an integration that began before the arm was confirmed is stale too.
    f.dev.starts[1] = CORE_INTERVAL
    push!(f.dev.queue,
          epl_record(2, 7, CORE_INTERVAL, 1.0),
          epl_record(9, 7, CORE_INTERVAL, 1.0),
          epl_record(1, 8, 2CORE_INTERVAL, 1.0),
          epl_record(1, 7, CORE_INTERVAL + 2000, 1.0),
          strobe_record(4CORE_INTERVAL))
    service_pass!(f.core; wait_ms = 0)
    @test f.core.stale_records == 4
    @test f.core.channels.stale_records[1] == 2
    @test f.core.channels.records_ingested[1] == 0
    # While a re-arm is not yet confirmed, everything on the channel is stale.
    f.dev.starts[1] = typemax(Int64)
    f.dev.sample_count = 6CORE_INTERVAL
    push!(f.dev.queue, epl_record(1, 7, 5CORE_INTERVAL, 1.0), strobe_record(6CORE_INTERVAL))
    service_pass!(f.core; wait_ms = 0)
    @test f.core.stale_records == 5
end

@testset "A record with another number of correlators is not ingested" begin
    f = armed_fixture()
    f.dev.sample_count = 2CORE_INTERVAL
    push!(f.dev.queue,
          DeviceRecord(1, 7, CORE_INTERVAL, CORE_INTERVAL, pack_correlator_outputs(ones(ComplexF64, 5)), 5),
          strobe_record(2CORE_INTERVAL))
    service_pass!(f.core; wait_ms = 0)
    @test f.core.correlator_layout_mismatches == 1
    @test f.core.channels.records_ingested[1] == 0
end

@testset "A lost record restarts the bit clock" begin
    f = armed_fixture()
    feed_epl!(f, 1, 7, 200)
    ev = drain_events!(f, 1)
    @test ev.states[end][2].flags & HLP.STATE_BIT_OR_SECONDARY_CODE_SYNCED != 0
    bits_before = length(ev.bits)
    @test bits_before > 0
    feed_epl!(f, 1, 7, 10; first = 201, skip = (205,))
    @test f.core.lost_record_gaps == 1
    @test f.core.channels.lost_record_samples[1] == CORE_INTERVAL
    ev = drain_events!(f, 1)
    restart = [s for (_, s) in ev.statuses if s.code == HLP.STATUS_SYNC_RESET]
    @test length(restart) == 1
    @test ev.states[end][2].flags & HLP.STATE_BIT_OR_SECONDARY_CODE_SYNCED == 0
    @test ev.states[end][2].flags & HLP.STATE_BIT_EDGE_KNOWN == 0
    # The bit count starts again once sync is found anew.
    feed_epl!(f, 1, 7, 200; first = 211)
    ev = drain_events!(f, 1)
    @test ev.states[end][2].flags & HLP.STATE_BIT_OR_SECONDARY_CODE_SYNCED != 0
    @test first(ev.bits)[2].bit_index == 1
end

@testset "A short hole before a short record is a re-arm, not a loss" begin
    f = armed_fixture()
    feed_epl!(f, 1, 7, 5)
    # The next record starts 2000 samples late and ends on the grid.
    f.dev.sample_count = 6CORE_INTERVAL
    push!(f.dev.queue, epl_record(1, 7, 6CORE_INTERVAL, 1.0; samples = 2000), strobe_record(6CORE_INTERVAL))
    service_pass!(f.core; wait_ms = 0)
    # It is ingested once the next span closes.
    f.dev.sample_count = 7CORE_INTERVAL
    push!(f.dev.queue, strobe_record(7CORE_INTERVAL))
    service_pass!(f.core; wait_ms = 0)
    @test f.core.rearm_gaps == 1
    @test f.core.lost_record_gaps == 0
    @test f.core.channels.rearm_dead_samples[1] == 2000
end

@testset "A post-sync record that overshoots the bit boundary drops sync" begin
    f = armed_fixture()
    # Bits flip every 20 records; after record 219 the bit synchronizer has 19 of
    # bit 11's 20 code periods.
    feed_epl!(f, 1, 7, 219)
    @test drain_events!(f, 1).states[end][2].flags & HLP.STATE_BIT_OR_SECONDARY_CODE_SYNCED != 0
    # One record spanning records 220 and 221: it closes bit 11 and spills a
    # code period past the boundary.
    f.dev.sample_count = 221CORE_INTERVAL
    push!(f.dev.queue, epl_record(1, 7, 221CORE_INTERVAL, 1.0; samples = 2CORE_INTERVAL),
          strobe_record(220CORE_INTERVAL), strobe_record(221CORE_INTERVAL))
    service_pass!(f.core; wait_ms = 0)
    f.dev.sample_count = 222CORE_INTERVAL
    push!(f.dev.queue, strobe_record(222CORE_INTERVAL))
    service_pass!(f.core; wait_ms = 0)
    ev = drain_events!(f, 1)
    @test any(s.code == HLP.STATUS_SYNC_RESET for (_, s) in ev.statuses)
    @test ev.states[end][2].flags & HLP.STATE_BIT_OR_SECONDARY_CODE_SYNCED == 0
    @test f.core.lost_record_gaps == 0
end

@testset "A sample index far ahead moves the timebase only when corroborated" begin
    f = armed_fixture()
    feed_epl!(f, 1, 7, 3)
    closed = f.core.nco_updates_closed
    far = 3CORE_INTERVAL + f.core.config.max_timebase_advance_samples + 10CORE_INTERVAL
    push!(f.dev.queue, strobe_record(far))
    service_pass!(f.core; wait_ms = 0)
    @test f.core.implausible_records == 1
    @test f.core.nco_updates_closed == closed
    # A second, unrelated jump is no corroboration either.
    push!(f.dev.queue, strobe_record(far + 10 * f.core.config.max_timebase_advance_samples))
    service_pass!(f.core; wait_ms = 0)
    @test f.core.implausible_records == 2
    # A record close to the last candidate is: the clock jumps.
    f.dev.sample_count = far + 10 * f.core.config.max_timebase_advance_samples + CORE_INTERVAL
    push!(f.dev.queue, strobe_record(f.dev.sample_count))
    service_pass!(f.core; wait_ms = 0)
    @test f.core.implausible_records == 2
    @test f.core.latest_sample_index == f.dev.sample_count
    @test f.core.nco_updates_closed > closed
end

@testset "Records beyond the pending buffer are dropped" begin
    f = armed_fixture(; max_pending_records = 4)
    f.dev.sample_count = 10CORE_INTERVAL
    for k = 1:10
        push!(f.dev.queue, epl_record(1, 7, k * CORE_INTERVAL, 1.0))
    end
    @test take_records!(f.core) == 4
    @test f.core.dropped_records == 6
end

@testset "A backlog older than max_backlog_nco_updates is closed with the loop open" begin
    f = armed_fixture()
    feed_epl!(f, 1, 7, 3)
    drain_events!(f, 1)
    commands = length(f.dev.nco_commands)
    # Ten NCO update spans of records arrive at once, long after they were cut.
    for k = 4:13
        push!(f.dev.queue, epl_record(1, 7, k * CORE_INTERVAL, 1.0), strobe_record(k * CORE_INTERVAL))
    end
    f.dev.sample_count = 14CORE_INTERVAL
    service_pass!(f.core; wait_ms = 0)
    ev = drain_events!(f, 1)
    observation = [s.flags & HLP.STATE_LOOP_OPEN != 0 for (_, s) in ev.states]
    @test f.core.open_loop_nco_updates == count(observation)
    # The oldest spans only observe; the recent ones step the loop again.
    @test observation[1] && !observation[end]
    @test length(ev.integrations) == 10
    @test length(f.dev.nco_commands) == commands + 1
end

@testset "NCO commands the device refuses, or that are not finite, are counted" begin
    f = armed_fixture()
    f.dev.accept_nco_commands = false
    # A record is ingested in the pass after it ends: three records, two NCO commands.
    feed_epl!(f, 1, 7, 3)
    @test f.core.nco_commands_rejected == 2
    @test f.core.nco_commands_committed == 0
    # Refused commands are cancelled: the timeline keeps the handover's.
    @test get_nco_command(f.core.channels.nco_timeline[1], typemax(Int64)) == NCOCommand(100.0Hz, 100.0Hz / 1540)
    f.dev.accept_nco_commands = true
    # A record of NaN correlator outputs steers the loop to NaN: that command is never scheduled.
    nan_outputs = pack_correlator_outputs(fill(complex(NaN, NaN), 3))
    for k = 4:5
        f.dev.sample_count = k * CORE_INTERVAL
        k == 4 && push!(f.dev.queue, DeviceRecord(1, 7, 4CORE_INTERVAL, CORE_INTERVAL, nan_outputs, 3))
        push!(f.dev.queue, strobe_record(k * CORE_INTERVAL))
        service_pass!(f.core; wait_ms = 0)
    end
    @test f.core.nco_commands_rejected == 3
    @test f.core.nco_commands_committed == 1
    @test all(isfinite(w[3]) && isfinite(w[4]) for w in f.dev.nco_commands)
end

@testset "Without a reported code phase the code phase is dead-reckoned" begin
    f = armed_fixture()
    # Record k is ingested at the boundary (k + 1) · interval.
    feed_epl!(f, 1, 7, 2; code_phase = NaN)
    # No reported code phase yet: no phase to report.
    @test f.core.channels.code_phase_sample_index[1] == typemin(Int64)
    # Record 3 reports the replica at chip 0 as it ends.
    feed_epl!(f, 1, 7, 1; first = 3, code_phase = 0.0)
    feed_epl!(f, 1, 7, 1; first = 4, code_phase = NaN)
    @test f.core.channels.code_phase_sample_index[1] == 4CORE_INTERVAL
    feed_epl!(f, 1, 7, 2; first = 5, code_phase = NaN)
    @test f.core.channels.code_phase_sample_index[1] == 6CORE_INTERVAL
    # Three code periods on from the reported code phase, at the commanded code Doppler.
    rate = (1.023e6 + f.core.channels.code_doppler[1]) / CORE_FS
    @test f.core.channels.code_phase[1] ≈ mod(3CORE_INTERVAL * rate, 1023) atol = 0.01
    state = drain_events!(f, 1).states[end][2]
    @test state.code_phase_chips == f.core.channels.code_phase[1]
    @test state.flags & HLP.STATE_CODE_PHASE_KNOWN != 0
end
