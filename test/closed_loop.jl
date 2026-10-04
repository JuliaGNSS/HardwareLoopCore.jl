# The loop core closed through the simulated device: arming over the command
# ring, records folded per epoch, words committed to the device, the events the
# receiver mirrors — and, once warm, a service pass that allocates nothing.

@testset "Arming answers with STATUS_ARMED at the sample the channel started" begin
    f = core_fixture()
    arm!(f, 2, 7; doppler = 100.0, code_phase = 10.0, sequence = 41)
    correlate_chunk!(f.dev, zeros(ComplexF64, CORE_EPOCH))
    service_pass!(f.core; wait_ms = 0)
    ev = drain_events!(f, 2)
    @test length(ev.statuses) == 1
    tag, status = ev.statuses[1]
    @test status.code == HLP.STATUS_ARMED
    @test status.sequence == 41
    @test tag.prn == 7
    # Armed while the counter stood at the end of the first chunk.
    @test status.sample == CORE_EPOCH
    @test f.core.channels.armed[2] && f.core.channels.confirmed[2]
    @test f.dev.channels[2].active
end

@testset "An arm for a signal the loop does not serve is rejected" begin
    f = core_fixture()
    arm!(f, 1, 7; doppler = 0.0, code_phase = 0.0, sequence = 5, signal = "GalileoE1B")
    service_pass!(f.core; wait_ms = 0)
    ev = drain_events!(f, 1)
    @test length(ev.statuses) == 1
    @test ev.statuses[1][2].code == HLP.STATUS_ARM_REJECTED
    @test ev.statuses[1][2].reason == HLP.REJECT_UNSUPPORTED_SIGNAL
    @test !f.core.channels.armed[1]
    @test !f.dev.channels[1].active
end

@testset "The loop closes through the simulated device ($delay-epoch record delay)" for delay in (0, 2, 4)
    r = run_closed_loop(; record_delay = delay * CORE_EPOCH)
    # Pulled in from 20 Hz and a quarter chip off. Once bit sync is found the
    # FLL-assisted loop's Doppler wanders about the truth by several Hz from
    # record to record (Tracking's own software loop does the same on this
    # signal), so the instantaneous word is judged loosely and its mean over the
    # last half second tightly.
    @test abs(r.device_doppler - r.true_doppler) < 15.0
    mean_doppler = sum(s[2].carrier_doppler_hz for s in r.states[end-499:end]) / 500
    @test abs(mean_doppler - r.true_doppler) < 3.0
    @test abs(r.device_code_error) < 0.1
    # A word every fold pre-sync, one per bit after it; nothing landed late.
    @test r.f.core.words_committed > 500
    @test r.f.core.words_late == 0
    @test r.f.core.words_rejected == 0
    # An epoch state per fold, and the last one is synchronised and anchored.
    @test length(r.states) >= 1390
    last_state = r.states[end][2]
    @test last_state.flags & HLP.STATE_SYNC_FOUND != 0
    @test last_state.flags & HLP.STATE_CODE_PHASE_ANCHORED != 0
    @test last_state.flags & HLP.STATE_BIT_PHASE_ANCHORED != 0
    @test abs(last_state.carrier_doppler_hz - r.true_doppler) < 15.0
    # C/N₀ from the noise reference: 0.126² · 4 MS/s against unit variance is
    # 48 dBHz; allow the estimator's spread.
    @test 44 < 10log10(last_state.cn0_linear_hz) < 50
    # Bits alternate every 20 ms, on the 80 000-sample grid of the synthetic
    # bit stream (offset by the code phase the records are cut on).
    @test length(r.bits) >= 25
    # The first bit or two after sync close a partial buffer; from then on
    # the grid holds.
    samples = [Int(tag.device_sample) for (tag, _) in r.bits]
    @test all(abs.(diff(samples)[3:end] .- 80_000) .<= 4)
    signs = [sign(b.soft_bit) for (_, b) in r.bits]
    @test all(signs[3:end] .!= signs[2:end-1])
    @test all(b.bit_index == i for (i, (_, b)) in enumerate(r.bits))
    @test r.f.core.implausible_dumps == 0
end

@testset "A device that scales its accumulators reads the same C/N₀" begin
    # The gateware's ±127 carrier ROM: satellite records and the noise reference
    # both carry the gain, and both are divided by it, so the C/N₀ is unchanged.
    r = run_closed_loop(; seconds = 1.0, replica_amplitude = 127.0)
    last_state = r.states[end][2]
    @test 44 < 10log10(last_state.cn0_linear_hz) < 50
    @test abs(r.device_doppler - r.true_doppler) < 15.0
    @test abs(r.device_code_error) < 0.1
end

@testset "A warm service pass allocates nothing" begin
    r = run_closed_loop(; seconds = 1.0, warmup_chunks = 200)
    @test r.allocated == 0
end

@testset "Release, query, configure and shutdown are acknowledged" begin
    f = core_fixture()
    arm!(f, 1, 7; doppler = 100.0, code_phase = 10.0, sequence = 1)
    correlate_chunk!(f.dev, zeros(ComplexF64, CORE_EPOCH))
    service_pass!(f.core; wait_ms = 0)
    drain_events!(f, 1)
    ring = command_ring(f.seg)
    publish!(ring, CommandTag(HLP.COMMAND_QUERY_STATE, 0, 2), QueryStateCommand())
    publish!(ring, CommandTag(HLP.COMMAND_CONFIGURE, 0, 3),
             ConfigureCommand(Int64(8000), Int32(0), Int32(0), 0.0, Int32(0), HLP.EVENTS_TAPS, Int32(0), UInt32(0)))
    publish!(ring, CommandTag(HLP.COMMAND_RELEASE, 1, 4), ReleaseCommand())
    publish!(ring, CommandTag(HLP.COMMAND_RELEASE, 1, 5), ReleaseCommand())
    publish!(ring, CommandTag(HLP.COMMAND_SHUTDOWN, 0, 6), ShutdownCommand())
    service_pass!(f.core; wait_ms = 0)
    ev = drain_events!(f, 1)
    codes = [s.code for (_, s) in ev.statuses]
    seqs = [s.sequence for (_, s) in ev.statuses]
    @test codes == [HLP.STATUS_CHANNEL_STATE, HLP.STATUS_CONFIGURED, HLP.STATUS_RELEASED, HLP.STATUS_COMMAND_REJECTED, HLP.STATUS_SHUTDOWN]
    @test seqs == [2, 3, 4, 5, 6]
    @test ev.statuses[1][2].carrier_doppler_hz == 100.0
    @test ev.statuses[4][2].reason == HLP.REJECT_NOT_ARMED
    @test f.core.config.epoch_length == 8000
    @test f.core.config.publish_taps
    @test !f.core.channels.armed[1]
    @test !f.dev.channels[1].active
    @test !f.core.running
end

@testset "A noise reference hops to a fresh decoy every noise_rearm_epochs" begin
    cfg = LoopConfig(; epoch_length = CORE_EPOCH, noise_rearm_epochs = 50)
    f = core_fixture(; config = cfg)
    arm!(f, 4, 30; doppler = 3000.0, code_phase = 500.0, signal_index = 0, sequence = 1)
    buf = Vector{ComplexF64}(undef, CORE_EPOCH)
    rng = Xoshiro(1)
    for c = 0:199
        randn!(rng, buf)
        correlate_chunk!(f.dev, buf)
        service_pass!(f.core; wait_ms = 0)
    end
    prns = [a[2] for a in f.dev.arms if a[1] == 4]
    @test length(prns) >= 4
    @test all(prns[2:end] .!= prns[1:end-1])
    @test f.core.bands[1].noise_density_ready
    @test 0.8 < f.core.bands[1].noise_density * CORE_FS < 1.25
end
