# The driver API's values: records and strobes, arm outcomes, the defaults of
# the optional methods, and the loop-wide configuration.

@testset "A device record keeps what the driver reported" begin
    taps = ComplexF64[1 + 2im, 3 - 1im, -2 + 0im]
    record = DeviceRecord(3, 17, 12_000, 4000, pack_taps(taps), 3; band = 2, code_phase = 511.5)
    @test isbitstype(DeviceRecord)
    @test record.channel == 3 && record.prn == 17 && record.band == 2
    @test record.sample_index == 12_000 && record.integrated_samples == 4000
    @test record.num_taps == 3 && record.num_ants == 1
    @test record.code_phase == 511.5
    @test record.taps[1:3] == Tuple(taps)
    @test all(iszero, record.taps[4:end])
    @test !is_strobe(record)
    @test isnan(DeviceRecord(1, 1, 0, 0, pack_taps(taps), 3).code_phase)
end

@testset "An epoch strobe is a record on channel 0" begin
    strobe = strobe_record(8000; band = 2)
    @test is_strobe(strobe)
    @test strobe.sample_index == 8000 && strobe.band == 2
    @test strobe.num_taps == 0 && strobe.integrated_samples == 0
    @test is_strobe(strobe_record(0))
end

@testset "pack_taps pads with zeros and refuses more than MAX_RECORD_TAPS" begin
    @test pack_taps(ComplexF32[1im]) == ntuple(i -> i == 1 ? 1.0im : 0.0im, MAX_RECORD_TAPS)
    @test pack_taps(ComplexF64[]) == ntuple(_ -> 0.0im, MAX_RECORD_TAPS)
    @test_throws ArgumentError pack_taps(ones(ComplexF64, MAX_RECORD_TAPS + 1))
end

@testset "Arm outcomes" begin
    @test ARM_ACCEPTED.accepted
    @test ARM_ACCEPTED.reason == HLP.REJECT_NONE
    rejected = arm_rejected(HLP.REJECT_DEVICE_ERROR)
    @test !rejected.accepted
    @test rejected.reason === UInt32(HLP.REJECT_DEVICE_ERROR)
end

@testset "The optional driver methods default to no-ops" begin
    dev = SimulatedDevice(CORE_SYSTEM; sampling_freq = CORE_FS)
    @test wait_records(dev, 10) === nothing
    @test overflowed_channels!(dev) == 0
end

@testset "LoopConfig" begin
    cfg = LoopConfig(; epoch_length = 4000)
    @test cfg.epoch_length == 4000
    @test cfg.commit_lead_samples == 0
    @test cfg.coherent_code_blocks == 1
    @test cfg.max_integration_time == 20e-3
    @test cfg.max_backlog_epochs == 4
    @test cfg.noise_rearm_epochs == 1000
    @test !cfg.publish_taps
    # One second's worth of epochs unless stated.
    @test cfg.max_epoch_clock_advance == 1000 * 4000
    @test LoopConfig(; epoch_length = 4000, max_epoch_clock_advance = 123).max_epoch_clock_advance == 123
    @test_throws ArgumentError LoopConfig(; epoch_length = 0)
    @test_throws ArgumentError LoopConfig(; epoch_length = 4000, max_integration_time = 0)
end

@testset "LoopCore defaults the epoch to the first signal's code period" begin
    f = core_fixture()
    @test f.core.config.epoch_length == CORE_EPOCH
    @test f.core.num_channels == 4
    @test f.core.num_ants == 1
    @test f.core.running
    @test length(f.core.latency_hist) == length(LATENCY_EDGES_US) + 1
    # Galileo E1B's code period is 4 ms.
    dev = SimulatedDevice(GalileoE1B(); sampling_freq = CORE_FS)
    seg = create_segment(nothing, SegmentConfig(; channel_count = 6, bands = dev.bands))
    @test LoopCore(dev, (GalileoE1B(),), seg).config.epoch_length == 4 * CORE_EPOCH
end

@testset "LoopCore refuses a driver with another antenna count" begin
    dev = ScriptedDriver(; num_ants = 2)
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
