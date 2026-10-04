# Records the core must not believe, records that never arrived, and the epoch
# clock's defences — through the scripted driver, record by record.

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
    f.dev.sample_count = 4CORE_EPOCH
    # Channel 2 is free, channel 9 does not exist, PRN 8 is not channel 1's,
    # and an integration that began before the arm was confirmed is stale too.
    f.dev.starts[1] = CORE_EPOCH
    push!(f.dev.queue,
          epl_record(2, 7, CORE_EPOCH, 1.0),
          epl_record(9, 7, CORE_EPOCH, 1.0),
          epl_record(1, 8, 2CORE_EPOCH, 1.0),
          epl_record(1, 7, CORE_EPOCH + 2000, 1.0),
          strobe_record(4CORE_EPOCH))
    service_pass!(f.core; wait_ms = 0)
    @test f.core.stale_dumps == 4
    @test f.core.channels.stale_records[1] == 2
    @test f.core.channels.records_folded[1] == 0
    # While a re-arm is not yet confirmed, everything on the channel is stale.
    f.dev.starts[1] = typemax(Int64)
    f.dev.sample_count = 6CORE_EPOCH
    push!(f.dev.queue, epl_record(1, 7, 5CORE_EPOCH, 1.0), strobe_record(6CORE_EPOCH))
    service_pass!(f.core; wait_ms = 0)
    @test f.core.stale_dumps == 5
end

@testset "A record with another tap layout is not folded" begin
    f = armed_fixture()
    f.dev.sample_count = 2CORE_EPOCH
    push!(f.dev.queue,
          DeviceRecord(1, 7, CORE_EPOCH, CORE_EPOCH, pack_taps(ones(ComplexF64, 5)), 5),
          strobe_record(2CORE_EPOCH))
    service_pass!(f.core; wait_ms = 0)
    @test f.core.tap_layout_mismatches == 1
    @test f.core.channels.records_folded[1] == 0
end

@testset "A lost record restarts the bit clock" begin
    f = armed_fixture()
    feed_epl!(f, 1, 7, 200)
    ev = drain_events!(f, 1)
    @test ev.states[end][2].flags & HLP.STATE_SYNC_FOUND != 0
    bits_before = length(ev.bits)
    @test bits_before > 0
    feed_epl!(f, 1, 7, 10; first = 201, skip = (205,))
    @test f.core.lost_record_gaps == 1
    @test f.core.channels.lost_record_samples[1] == CORE_EPOCH
    ev = drain_events!(f, 1)
    restart = [s for (_, s) in ev.statuses if s.code == HLP.STATUS_BIT_CLOCK_RESTART]
    @test length(restart) == 1
    @test ev.states[end][2].flags & HLP.STATE_SYNC_FOUND == 0
    @test ev.states[end][2].flags & HLP.STATE_BIT_PHASE_ANCHORED == 0
    # The bit count starts again once sync is found anew.
    feed_epl!(f, 1, 7, 200; first = 211)
    ev = drain_events!(f, 1)
    @test ev.states[end][2].flags & HLP.STATE_SYNC_FOUND != 0
    @test first(ev.bits)[2].bit_index == 1
end

@testset "A short hole before a short record is a re-arm, not a loss" begin
    f = armed_fixture()
    feed_epl!(f, 1, 7, 5)
    # The next record starts 2000 samples late and ends on the grid.
    f.dev.sample_count = 6CORE_EPOCH
    push!(f.dev.queue, epl_record(1, 7, 6CORE_EPOCH, 1.0; samples = 2000), strobe_record(6CORE_EPOCH))
    service_pass!(f.core; wait_ms = 0)
    # It is folded once the next epoch closes.
    f.dev.sample_count = 7CORE_EPOCH
    push!(f.dev.queue, strobe_record(7CORE_EPOCH))
    service_pass!(f.core; wait_ms = 0)
    @test f.core.rearm_gaps == 1
    @test f.core.lost_record_gaps == 0
    @test f.core.channels.rearm_dead_samples[1] == 2000
end

@testset "A post-sync record that overshoots the bit boundary drops sync" begin
    f = armed_fixture()
    # Bits flip every 20 records; after record 219 the bit buffer has 19 of
    # bit 11's 20 blocks.
    feed_epl!(f, 1, 7, 219)
    @test drain_events!(f, 1).states[end][2].flags & HLP.STATE_SYNC_FOUND != 0
    # One record spanning records 220 and 221: it closes bit 11 and spills a
    # block past the boundary.
    f.dev.sample_count = 221CORE_EPOCH
    push!(f.dev.queue, epl_record(1, 7, 221CORE_EPOCH, 1.0; samples = 2CORE_EPOCH),
          strobe_record(220CORE_EPOCH), strobe_record(221CORE_EPOCH))
    service_pass!(f.core; wait_ms = 0)
    f.dev.sample_count = 222CORE_EPOCH
    push!(f.dev.queue, strobe_record(222CORE_EPOCH))
    service_pass!(f.core; wait_ms = 0)
    ev = drain_events!(f, 1)
    @test any(s.code == HLP.STATUS_BIT_CLOCK_RESTART for (_, s) in ev.statuses)
    @test ev.states[end][2].flags & HLP.STATE_SYNC_FOUND == 0
    @test f.core.lost_record_gaps == 0
end

@testset "A sample index far ahead moves the epoch clock only when corroborated" begin
    f = armed_fixture()
    feed_epl!(f, 1, 7, 3)
    folded = f.core.epochs_folded
    far = 3CORE_EPOCH + f.core.config.max_epoch_clock_advance + 10CORE_EPOCH
    push!(f.dev.queue, strobe_record(far))
    service_pass!(f.core; wait_ms = 0)
    @test f.core.implausible_dumps == 1
    @test f.core.epochs_folded == folded
    # A second, unrelated jump is no corroboration either.
    push!(f.dev.queue, strobe_record(far + 10 * f.core.config.max_epoch_clock_advance))
    service_pass!(f.core; wait_ms = 0)
    @test f.core.implausible_dumps == 2
    # A record close to the last candidate is: the clock jumps.
    f.dev.sample_count = far + 10 * f.core.config.max_epoch_clock_advance + CORE_EPOCH
    push!(f.dev.queue, strobe_record(f.dev.sample_count))
    service_pass!(f.core; wait_ms = 0)
    @test f.core.implausible_dumps == 2
    @test f.core.latest_sample_index == f.dev.sample_count
    @test f.core.epochs_folded > folded
end

@testset "Records beyond the pending buffer are dropped" begin
    f = armed_fixture(; max_pending_records = 4)
    f.dev.sample_count = 10CORE_EPOCH
    for k = 1:10
        push!(f.dev.queue, epl_record(1, 7, k * CORE_EPOCH, 1.0))
    end
    @test take_records!(f.core) == 4
    @test f.core.dropped_records == 6
end

@testset "A backlog older than max_backlog_epochs is folded observation-only" begin
    f = armed_fixture()
    feed_epl!(f, 1, 7, 3)
    drain_events!(f, 1)
    words = length(f.dev.words)
    # Ten epochs of records arrive at once, long after they were cut.
    for k = 4:13
        push!(f.dev.queue, epl_record(1, 7, k * CORE_EPOCH, 1.0), strobe_record(k * CORE_EPOCH))
    end
    f.dev.sample_count = 14CORE_EPOCH
    service_pass!(f.core; wait_ms = 0)
    ev = drain_events!(f, 1)
    observation = [s.flags & HLP.STATE_OBSERVATION_ONLY != 0 for (_, s) in ev.states]
    @test f.core.skipped_epochs == count(observation)
    # The oldest epochs only observe; the recent ones step the loop again.
    @test observation[1] && !observation[end]
    @test length(ev.records) == 10
    @test length(f.dev.words) == words + 1
end

@testset "Words the device refuses, or that are not finite, are counted" begin
    f = armed_fixture()
    f.dev.accept_words = false
    # A record is folded in the pass after it ends: three records, two words.
    feed_epl!(f, 1, 7, 3)
    @test f.core.words_rejected == 2
    @test f.core.words_committed == 0
    f.dev.accept_words = true
    # A record of NaN taps steers the loop to NaN: that word is never scheduled.
    nan_taps = pack_taps(fill(complex(NaN, NaN), 3))
    for k = 4:5
        f.dev.sample_count = k * CORE_EPOCH
        k == 4 && push!(f.dev.queue, DeviceRecord(1, 7, 4CORE_EPOCH, CORE_EPOCH, nan_taps, 3))
        push!(f.dev.queue, strobe_record(k * CORE_EPOCH))
        service_pass!(f.core; wait_ms = 0)
    end
    @test f.core.words_rejected == 3
    @test f.core.words_committed == 1
    @test all(isfinite(w[3]) && isfinite(w[4]) for w in f.dev.words)
end

@testset "Without a reported code phase the code phase is dead-reckoned" begin
    f = armed_fixture()
    # Record k is folded at the boundary (k + 1) · epoch.
    feed_epl!(f, 1, 7, 2; code_phase = NaN)
    # No anchor yet: no phase to report.
    @test f.core.channels.phase_ref_sample[1] == typemin(Int64)
    # Record 3 reports the replica at chip 0 as it ends.
    feed_epl!(f, 1, 7, 1; first = 3, code_phase = 0.0)
    feed_epl!(f, 1, 7, 1; first = 4, code_phase = NaN)
    @test f.core.channels.phase_ref_sample[1] == 4CORE_EPOCH
    feed_epl!(f, 1, 7, 2; first = 5, code_phase = NaN)
    @test f.core.channels.phase_ref_sample[1] == 6CORE_EPOCH
    # Three code periods on from the anchor, at the commanded code Doppler.
    rate = (1.023e6 + f.core.channels.code_doppler[1]) / CORE_FS
    @test f.core.channels.code_phase[1] ≈ mod(3CORE_EPOCH * rate, 1023) atol = 0.01
    state = drain_events!(f, 1).states[end][2]
    @test state.code_phase_chips == f.core.channels.code_phase[1]
    @test state.flags & HLP.STATE_CODE_PHASE_ANCHORED != 0
end
