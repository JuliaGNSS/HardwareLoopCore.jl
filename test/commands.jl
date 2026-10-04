# Commands from the receiver: every refusal, the device's own refusals, noise
# references and passengers, configuration and unknown commands.

statuses(f, channel) = [s for (_, s) in drain_events!(f, channel).statuses]

@testset "Arms the loop cannot serve are refused with the reason" begin
    f = scripted_fixture()
    arm!(f, 0, 7; doppler = 0.0, code_phase = 0.0, sequence = 1)
    arm!(f, 9, 7; doppler = 0.0, code_phase = 0.0, sequence = 2)
    arm!(f, 2, 7; doppler = 0.0, code_phase = 0.0, sequence = 3, band = 2)
    arm!(f, 2, 7; doppler = 0.0, code_phase = 0.0, sequence = 4, num_taps = 0)
    arm!(f, 2, 7; doppler = 0.0, code_phase = 0.0, sequence = 5, num_taps = 6)
    arm!(f, 2, 7; doppler = 0.0, code_phase = 0.0, sequence = 6, sampling_freq_hz = 0.0)
    service_pass!(f.core; wait_ms = 0)
    # A refusal for a channel that does not exist is answered on channel 1.
    loop_wide = statuses(f, 1)
    @test [s.sequence for s in loop_wide] == [1, 2]
    @test all(s.code == HLP.STATUS_ARM_REJECTED for s in loop_wide)
    @test all(s.reason == HLP.REJECT_NO_SUCH_CHANNEL for s in loop_wide)
    on_channel = statuses(f, 2)
    @test [s.sequence for s in on_channel] == [3, 4, 5, 6]
    @test all(s.reason == HLP.REJECT_BAD_CONFIG for s in on_channel)
    @test isempty(f.dev.arms)
    @test !any(f.core.channels.armed)
end

@testset "An arm the driver refuses frees the channel" begin
    f = scripted_fixture()
    f.dev.arm_outcome = arm_rejected(HLP.REJECT_DEVICE_ERROR)
    arm!(f, 1, 7; doppler = 0.0, code_phase = 0.0, sequence = 1)
    service_pass!(f.core; wait_ms = 0)
    s = only(statuses(f, 1))
    @test s.code == HLP.STATUS_ARM_REJECTED && s.reason == HLP.REJECT_DEVICE_ERROR
    @test !f.core.channels.armed[1] && f.core.channels.bank[1] == 0
end

@testset "A signal the core serves but the device cannot is refused by the device" begin
    dev = SimulatedDevice(CORE_SYSTEM; sampling_freq = CORE_FS, num_channels = 2)
    seg = create_segment(nothing, SegmentConfig(; channel_count = 2, bands = dev.bands))
    f = (; dev, seg, core = LoopCore(dev, (CORE_SYSTEM, GalileoE1B()), seg))
    arm!(f, 1, 7; doppler = 0.0, code_phase = 0.0, sequence = 1, signal = "GalileoE1B")
    service_pass!(f.core; wait_ms = 0)
    s = only(statuses(f, 1))
    @test s.code == HLP.STATUS_ARM_REJECTED && s.reason == HLP.REJECT_UNSUPPORTED_SIGNAL
    @test !f.core.channels.armed[1]
end

@testset "An arm the device gives up on is rejected and the channel released" begin
    f = scripted_fixture()
    f.dev.confirm_on_arm = false
    arm!(f, 1, 7; doppler = 0.0, code_phase = 0.0, sequence = 1)
    arm!(f, 2, 30; doppler = 0.0, code_phase = 0.0, sequence = 2, signal_index = 0)
    service_pass!(f.core; wait_ms = 0)
    # Scheduled, not yet confirmed: nothing to say.
    @test isempty(statuses(f, 1)) && isempty(statuses(f, 2))
    @test f.core.channels.armed[1] && !f.core.channels.confirmed[1]
    @test f.core.bands[1].noise_channel == 2
    f.dev.starts[1] = typemin(Int64)
    f.dev.starts[2] = typemin(Int64)
    service_pass!(f.core; wait_ms = 0)
    for (ch, seq) in ((1, 1), (2, 2))
        s = only(statuses(f, ch))
        @test s.code == HLP.STATUS_ARM_REJECTED && s.reason == HLP.REJECT_DEVICE_ERROR
        @test s.sequence == seq
        @test !f.core.channels.armed[ch]
    end
    @test sort(f.dev.releases) == [1, 2]
    @test f.core.bands[1].noise_channel == 0
end

@testset "A noise reference is re-pointed only by another noise arm" begin
    f = scripted_fixture()
    arm!(f, 4, 30; doppler = 3000.0, code_phase = 0.0, sequence = 1, signal_index = 0)
    service_pass!(f.core; wait_ms = 0)
    @test f.core.bands[1].noise_channel == 4 && f.core.bands[1].noise_prn == 30
    arm!(f, 4, 7; doppler = 0.0, code_phase = 0.0, sequence = 2)
    service_pass!(f.core; wait_ms = 0)
    s = statuses(f, 4)
    @test s[1].code == HLP.STATUS_ARMED
    @test s[2].code == HLP.STATUS_ARM_REJECTED && s[2].reason == HLP.REJECT_CHANNEL_BUSY
    @test f.core.channels.signal_index[4] == 0 && f.core.channels.prn[4] == 30
    arm!(f, 4, 31; doppler = 3000.0, code_phase = 0.0, sequence = 3, signal_index = 0)
    service_pass!(f.core; wait_ms = 0)
    @test only(statuses(f, 4)).code == HLP.STATUS_ARMED
    @test f.core.bands[1].noise_channel == 4 && f.core.bands[1].noise_prn == 31
    # A satellite channel may be re-armed in place.
    arm!(f, 1, 7; doppler = 0.0, code_phase = 0.0, sequence = 4)
    arm!(f, 1, 8; doppler = 0.0, code_phase = 0.0, sequence = 5)
    service_pass!(f.core; wait_ms = 0)
    @test [s.sequence for s in statuses(f, 1)] == [5]
    @test f.core.channels.prn[1] == 8
    # Releasing the noise reference frees the band's reference.
    publish!(command_ring(f.seg), CommandTag(HLP.COMMAND_RELEASE, 4, 6), ReleaseCommand())
    service_pass!(f.core; wait_ms = 0)
    @test only(statuses(f, 4)).code == HLP.STATUS_RELEASED
    @test f.core.bands[1].noise_channel == 0
end

@testset "Releasing a channel that does not exist is refused" begin
    f = scripted_fixture()
    publish!(command_ring(f.seg), CommandTag(HLP.COMMAND_RELEASE, 9, 1), ReleaseCommand())
    service_pass!(f.core; wait_ms = 0)
    s = only(statuses(f, 1))
    @test s.code == HLP.STATUS_ARM_REJECTED && s.reason == HLP.REJECT_NO_SUCH_CHANNEL
end

@testset "An unknown command is refused on channel 1" begin
    f = scripted_fixture()
    publish!(command_ring(f.seg), CommandTag(0x7f, 0, 9), ShutdownCommand())
    @test handle_commands!(f.core) == 1
    s = only(statuses(f, 1))
    @test s.code == HLP.STATUS_COMMAND_REJECTED && s.reason == HLP.REJECT_UNKNOWN_COMMAND
    @test s.sequence == 9
    @test f.core.running
    @test handle_commands!(f.core) == 0
end

@testset "Configure leaves the fields it does not set alone" begin
    f = scripted_fixture()
    cfg = f.core.config
    configure(; epoch = 0, lead = 0, blocks = -1, tint = 0.0, rearm = 0, flags = 0x00, backlog = 0) =
        ConfigureCommand(Int64(epoch), Int32(lead), Int32(blocks), Float64(tint), Int32(rearm),
                         UInt8(flags), Int32(backlog), UInt32(0))
    publish!(command_ring(f.seg), CommandTag(HLP.COMMAND_CONFIGURE, 0, 1), configure())
    handle_commands!(f.core)
    @test (cfg.epoch_length, cfg.commit_lead_samples, cfg.coherent_code_blocks) == (CORE_EPOCH, 0, 1)
    @test (cfg.max_integration_time, cfg.noise_rearm_epochs, cfg.max_backlog_epochs) == (20e-3, 1000, 4)
    publish!(command_ring(f.seg), CommandTag(HLP.COMMAND_CONFIGURE, 0, 2),
             configure(; epoch = 8000, lead = 100, blocks = 0, tint = 10e-3, rearm = 50, backlog = 2))
    handle_commands!(f.core)
    @test (cfg.epoch_length, cfg.commit_lead_samples, cfg.coherent_code_blocks) == (8000, 100, 0)
    @test (cfg.max_integration_time, cfg.noise_rearm_epochs, cfg.max_backlog_epochs) == (10e-3, 50, 2)
    @test !cfg.publish_taps
    @test [s.code for s in statuses(f, 1)] == [HLP.STATUS_CONFIGURED, HLP.STATUS_CONFIGURED]
end

@testset "A passenger follows its driver's words, whichever is armed first" begin
    for passenger_first in (false, true)
        f = scripted_fixture()
        driver_arm() = arm!(f, 1, 7; doppler = 100.0, code_phase = 0.0, sequence = 1, group_key = "sat7")
        passenger_arm() = arm!(f, 2, 7; doppler = 100.0, code_phase = 0.0, sequence = 2, group_key = "sat7",
                               signal_index = 2)
        if passenger_first
            passenger_arm(); driver_arm()
        else
            driver_arm(); passenger_arm()
        end
        # Another satellite's passenger is not linked.
        arm!(f, 3, 8; doppler = 100.0, code_phase = 0.0, sequence = 3, group_key = "sat8", signal_index = 2)
        service_pass!(f.core; wait_ms = 0)
        @test f.core.channels.driver_channel[2] == 1
        @test f.core.channels.driver_channel[3] == 0
        feed_epl!(f, 1, 7, 10)
        words = Dict(ch => [(w[3], w[4]) for w in f.dev.words if w[1] == ch] for ch = 1:3)
        @test !isempty(words[1])
        @test words[2] == words[1]
        @test isempty(words[3])
        @test f.core.channels.carrier_doppler[2] == f.core.channels.carrier_doppler[1]
    end
end
