# Commands from the receiver: every refusal, the device's own refusals, noise
# references and passengers, configuration and unknown commands.

statuses(f, channel) = [s for (_, s) in drain_events!(f, channel).statuses]

@testset "Arms the loop cannot serve are refused with the reason" begin
    f = scripted_fixture()
    arm!(f, 0, 7; doppler = 0.0, code_phase = 0.0, sequence = 1)
    arm!(f, 9, 7; doppler = 0.0, code_phase = 0.0, sequence = 2)
    arm!(f, 2, 7; doppler = 0.0, code_phase = 0.0, sequence = 3, band = 2)
    # The keyword constructor refuses these correlator counts; a receiver that
    # fills the record itself can still send them.
    offsets = Int32.(core_correlator_sample_offsets(f.core))
    for (sequence, num_correlators) in ((4, 0), (5, 6))
        cmd = ArmCommand(FixedName(:GPSL1CA), FixedName(:GPSL1CA), 7, 1, 0.0, 0.0, 0.0, 0, offsets, num_correlators, 1,
                         HLP.SECONDARY_CODE_LOOP_WIPES, 0x00, 0x0000)
        publish!(command_ring(f.seg), CommandTag(HLP.COMMAND_ARM, 2, sequence), cmd)
    end
    service_pass!(f.core; wait_ms = 0)
    # A refusal for a channel that does not exist is answered on channel 1.
    loop_wide = statuses(f, 1)
    @test [s.sequence for s in loop_wide] == [1, 2]
    @test all(s.code == HLP.STATUS_ARM_REJECTED for s in loop_wide)
    @test all(s.reason == HLP.REJECT_NO_SUCH_CHANNEL for s in loop_wide)
    on_channel = statuses(f, 2)
    @test [s.sequence for s in on_channel] == [3, 4, 5]
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
    dev = SimulatedDevice(CORE_SYSTEM; sampling_frequency_hz = CORE_FS, num_channels = 2)
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

@testset "Configure changes exactly the fields it sets" begin
    f = scripted_fixture()
    cfg = f.core.config
    current() = (cfg.nco_update_interval_samples, cfg.max_coherent_integration_time_s,
                 cfg.nco_command_delay_samples, cfg.num_code_periods_to_integrate, cfg.noise_rearm_nco_updates,
                 cfg.max_backlog_nco_updates, cfg.publish_correlator_outputs)
    sequence = 0
    function configure!(cmd)
        publish!(command_ring(f.seg), CommandTag(HLP.COMMAND_CONFIGURE, 0, sequence += 1), cmd)
        handle_commands!(f.core)
        s = only(statuses(f, 1))
        (s.code, s.reason)
    end
    configured = (HLP.STATUS_CONFIGURED, HLP.REJECT_NONE)
    rejected = (HLP.STATUS_COMMAND_REJECTED, HLP.REJECT_BAD_CONFIG)
    @test configure!(ConfigureCommand()) == configured
    @test current() == (CORE_INTERVAL, 20e-3, 0, 1, 1000, 4, false)
    # Every field, zeros included.
    @test configure!(ConfigureCommand(; nco_update_interval_samples = 8000,
                                      max_coherent_integration_time_s = 10e-3,
                                      nco_command_delay_samples = 100, num_code_periods_to_integrate = 0,
                                      noise_rearm_nco_updates = 50, max_backlog_nco_updates = 0,
                                      event_flags = HLP.EVENTS_CORRELATOR_OUTPUTS)) == configured
    @test current() == (8000, 10e-3, 100, 0, 50, 0, true)
    # One field: the interval no longer drags the code periods along, and the
    # published events stay as they are.
    @test configure!(ConfigureCommand(; nco_update_interval_samples = CORE_INTERVAL)) == configured
    @test configure!(ConfigureCommand(; max_coherent_integration_time_s = 20e-3)) == configured
    @test current() == (CORE_INTERVAL, 20e-3, 100, 0, 50, 0, true)
    @test configure!(ConfigureCommand(; event_flags = 0)) == configured
    @test !cfg.publish_correlator_outputs
    # One field out of range rejects the whole command; so do a flag or an
    # event this loop does not know.
    before = current()
    @test configure!(ConfigureCommand(; max_coherent_integration_time_s = 5e-3,
                                      nco_update_interval_samples = 0)) == rejected
    @test configure!(ConfigureCommand(; max_coherent_integration_time_s = 5e-3,
                                      noise_rearm_nco_updates = 0)) == rejected
    @test configure!(ConfigureCommand(; max_coherent_integration_time_s = NaN)) == rejected
    @test configure!(ConfigureCommand(; num_code_periods_to_integrate = -1)) == rejected
    @test configure!(ConfigureCommand(; event_flags = 2)) == rejected
    @test configure!(ConfigureCommand(Int64(0), 5e-3, Int32(0), Int32(0), Int32(0), Int32(0), UInt32(0),
                                      HLP.CONFIGURE_SET_MAX_COHERENT_INTEGRATION_TIME | UInt32(128))) == rejected
    @test current() == before
end

@testset "A passenger follows its driver's NCO commands, whichever is armed first" begin
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
        commands = Dict(ch => [(w[3], w[4]) for w in f.dev.nco_commands if w[1] == ch] for ch = 1:3)
        @test !isempty(commands[1])
        @test commands[2] == commands[1]
        @test isempty(commands[3])
        @test f.core.channels.carrier_doppler[2] == f.core.channels.carrier_doppler[1]
    end
end

@testset "A passenger's integrations step its driver's loop state" begin
    # GPS L5: the Q pilot drives, the I data channel is its passenger. Every
    # integration of the satellite goes to `step_loop!` with the state in the
    # driver channel's slot; the scalar loop returns the command in force for the
    # passenger's, so the driver's loop runs exactly as without it.
    pilot, data = GPSL5Q(), GPSL5I()
    function driver_dopplers(with_passenger)
        f = scripted_fixture((pilot, data))
        arm!(f, 1, 7; doppler = 100.0, code_phase = 0.0, sequence = 1, signal = "GPSL5Q", group_key = "L5-7")
        with_passenger && arm!(f, 2, 7; doppler = 100.0, code_phase = 0.0, sequence = 2, signal = "GPSL5I",
                               group_key = "L5-7", signal_index = 2)
        service_pass!(f.core; wait_ms = 0)
        passenger_state = f.core.channels.loop_state[2]
        for k = 1:40
            f.dev.sample_count = Int64(k) * CORE_INTERVAL
            q_chip = GNSSSignals.secondary_value(get_secondary_code(pilot), 7, mod(k - 1, 20))
            i_chip = GNSSSignals.secondary_value(get_secondary_code(data), 7, mod(k - 1, 10))
            push!(f.dev.queue, epl_record(1, 7, f.dev.sample_count, cis(0.01k) * q_chip; code_phase = 0.0))
            with_passenger && push!(f.dev.queue,
                epl_record(2, 7, f.dev.sample_count, im * cis(0.01k) * i_chip; code_phase = 0.0))
            push!(f.dev.queue, strobe_record(f.dev.sample_count))
            service_pass!(f.core; wait_ms = 0)
        end
        if with_passenger
            # The passenger's integrations reached the loop, and left its own slot alone.
            @test f.core.channels.last_record_end_sample_index[2] == f.core.channels.last_record_end_sample_index[1] > 0
            @test f.core.channels.loop_state[2] == passenger_state
        end
        [(w[3], w[4]) for w in f.dev.nco_commands if w[1] == 1], f.core.channels.loop_state[1]
    end
    alone, alone_state = driver_dopplers(false)
    with_passenger, with_passenger_state = driver_dopplers(true)
    @test !isempty(alone)
    @test with_passenger == alone
    @test with_passenger_state == alone_state
end
