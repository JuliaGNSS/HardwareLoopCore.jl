# The driver API's values: records and strobes, arm outcomes, the defaults of
# the optional methods, and the loop-wide configuration.

@testset "A device record keeps what the driver reported" begin
    outputs = ComplexF64[1 + 2im, 3 - 1im, -2 + 0im]
    record = DeviceRecord(3, 17, 12_000, 4000, pack_correlator_outputs(outputs), 3; band = 2, code_phase = 511.5)
    @test isbitstype(DeviceRecord)
    @test record.channel == 3 && record.prn == 17 && record.band == 2
    @test record.sample_index == 12_000 && record.num_integrated_samples == 4000
    @test record.num_correlators == 3 && record.num_antennas == 1
    @test record.code_phase == 511.5
    @test record.correlator_outputs[1:3] == Tuple(outputs)
    @test all(iszero, record.correlator_outputs[4:end])
    @test !is_strobe(record)
    @test isnan(DeviceRecord(1, 1, 0, 0, pack_correlator_outputs(outputs), 3).code_phase)
end

@testset "A timebase strobe is a record on channel 0" begin
    strobe = strobe_record(8000; band = 2)
    @test is_strobe(strobe)
    @test strobe.sample_index == 8000 && strobe.band == 2
    @test strobe.num_correlators == 0 && strobe.num_integrated_samples == 0
    @test is_strobe(strobe_record(0))
end

@testset "pack_correlator_outputs pads with zeros and refuses more than MAX_RECORD_CORRELATOR_OUTPUTS" begin
    @test pack_correlator_outputs(ComplexF32[1im]) == ntuple(i -> i == 1 ? 1.0im : 0.0im, MAX_RECORD_CORRELATOR_OUTPUTS)
    @test pack_correlator_outputs(ComplexF64[]) == ntuple(_ -> 0.0im, MAX_RECORD_CORRELATOR_OUTPUTS)
    @test_throws ArgumentError pack_correlator_outputs(ones(ComplexF64, MAX_RECORD_CORRELATOR_OUTPUTS + 1))
end

@testset "Arm outcomes" begin
    @test ARM_ACCEPTED.accepted
    @test ARM_ACCEPTED.reason == HLP.REJECT_NONE
    rejected = arm_rejected(HLP.REJECT_DEVICE_ERROR)
    @test !rejected.accepted
    @test rejected.reason === UInt32(HLP.REJECT_DEVICE_ERROR)
end

@testset "The optional driver methods default to no-ops" begin
    dev = SimulatedDevice(CORE_SYSTEM; sampling_frequency = CORE_FS)
    @test wait_records(dev, 10) === nothing
    @test overflowed_channels!(dev) == 0
end

@testset "LoopConfig" begin
    cfg = LoopConfig(; nco_update_interval_samples = 4000)
    @test cfg.nco_update_interval_samples == 4000
    @test cfg.nco_command_delay_samples == 0
    @test cfg.num_code_periods_to_integrate == 1
    @test cfg.max_integration_time == 20e-3
    @test cfg.max_backlog_nco_updates == 4
    @test cfg.noise_rearm_nco_updates == 1000
    @test !cfg.publish_correlator_outputs
    # One second's worth of NCO update intervals unless stated.
    @test cfg.max_timebase_advance == 1000 * 4000
    @test LoopConfig(; nco_update_interval_samples = 4000, max_timebase_advance = 123).max_timebase_advance == 123
    @test_throws ArgumentError LoopConfig(; nco_update_interval_samples = 0)
    @test_throws ArgumentError LoopConfig(; nco_update_interval_samples = 4000, max_integration_time = 0)
end

@testset "LoopCore defaults the NCO update interval to the first signal's code period" begin
    f = core_fixture()
    @test f.core.config.nco_update_interval_samples == CORE_INTERVAL
    @test f.core.num_channels == 4
    @test f.core.num_antennas == 1
    @test f.core.running
    @test length(f.core.latency_hist) == length(LATENCY_EDGES_US) + 1
    # Galileo E1B's code period is 4 ms.
    dev = SimulatedDevice(GalileoE1B(); sampling_frequency = CORE_FS)
    seg = create_segment(nothing, SegmentConfig(; channel_count = 6, bands = dev.bands))
    @test LoopCore(dev, (GalileoE1B(),), seg).config.nco_update_interval_samples == 4 * CORE_INTERVAL
end

@testset "LoopCore refuses a driver with another antenna count" begin
    dev = ScriptedDriver(; num_antennas = 2)
    seg = create_segment(nothing, SegmentConfig(; channel_count = 4, bands = dev.bands))
    err = try
        LoopCore(dev, (CORE_SYSTEM,), seg)
    catch e
        e
    end
    @test err isa ArgumentError
    @test occursin("reads 2 antenna block(s) per record but the core was built for 1", err.msg)
end

@testset "Record ages past the last histogram edge land in the overflow bin" begin
    @test HardwareLoopCore._latency_bin(Int64(0)) == 1
    @test HardwareLoopCore._latency_bin(Int64(LATENCY_EDGES_US[end] - 1)) == length(LATENCY_EDGES_US)
    @test HardwareLoopCore._latency_bin(Int64(10^9)) == length(LATENCY_EDGES_US) + 1
end
