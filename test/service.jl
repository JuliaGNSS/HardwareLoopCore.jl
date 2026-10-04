# The service loop around the passes.

@testset "run! serves until a shutdown command and marks the loop state" begin
    f = scripted_fixture()
    @test loop_state(f.seg) != HLP.LOOP_STATE_RUNNING
    publish!(command_ring(f.seg), CommandTag(HLP.COMMAND_SHUTDOWN, 0, 1), ShutdownCommand())
    run!(f.core; max_wait_ms = 0)
    @test !f.core.running
    @test loop_state(f.seg) == HLP.LOOP_STATE_STOPPED
    @test f.dev.waits == 1
    @test f.core.commands_handled == 1
    @test loop_heartbeat(f.seg) > 0
end

@testset "A pass records the age of the newest record it read" begin
    f = scripted_fixture()
    arm!(f, 1, 7; doppler = 0.0, code_phase = 0.0, sequence = 1)
    service_pass!(f.core; wait_ms = 0)
    feed_epl!(f, 1, 7, 2)
    # Records read the moment they end: no age.
    @test f.core.latency_hist[1] == 2
    # A record read 2 ms (8000 samples) after it ended.
    push!(f.dev.queue, epl_record(1, 7, 3CORE_EPOCH, 1.0))
    f.dev.sample_count = 5CORE_EPOCH
    service_pass!(f.core; wait_ms = 0)
    @test f.core.max_record_age_us == 2000
    @test f.core.latency_hist[findfirst(>(2000), LATENCY_EDGES_US)] == 1
    @test sum(f.core.latency_hist) == 3
    @test f.core.max_pass_ns > 0
end
