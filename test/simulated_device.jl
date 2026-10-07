# The simulated FPGA on its own: what it refuses, and how it reports.
# (`arm!` alone is the test helper that publishes an `ArmCommand`; the driver
# method is called qualified.)

@testset "SimulatedDevice takes a sampling frequency with units" begin
    dev = SimulatedDevice((CORE_SYSTEM, GalileoE1B()); sampling_freq = 4u"MHz", num_channels = 2)
    @test dev.sampling_freq == CORE_FS
    @test dev.epoch_length == CORE_EPOCH
    caps = driver_capabilities(dev)
    @test caps.num_channels == 2 && caps.num_ants == 1 && caps.max_taps == 5
    @test only(caps.bands).sampling_freq_hz == CORE_FS
end

@testset "SimulatedDevice refuses what it cannot correlate" begin
    dev = SimulatedDevice(CORE_SYSTEM; sampling_freq = CORE_FS, num_channels = 2)
    shifts = ntuple(i -> Int32(i <= 3 ? 2 - i : 0), 5)
    spec(signal; num_taps = 3, band = 1) =
        ArmSpec(signal, 1, 0.0, 0.0, 0.0, 0, shifts, num_taps, band, 1, 1, CORE_FS, 1.0, 1.0)
    @test HardwareLoopCore.arm!(dev, 1, spec(GalileoE1B())).reason == HLP.REJECT_UNSUPPORTED_SIGNAL
    @test HardwareLoopCore.arm!(dev, 1, spec(CORE_SYSTEM; num_taps = 6)).reason == HLP.REJECT_BAD_CONFIG
    @test HardwareLoopCore.arm!(dev, 1, spec(CORE_SYSTEM; band = 2)).reason == HLP.REJECT_BAD_CONFIG
    @test isempty(dev.arms)
    # A word for a channel that is not correlating is refused.
    @test !write_word!(dev, 2, 100.0, 0.1)
    @test HardwareLoopCore.arm!(dev, 2, spec(CORE_SYSTEM)) == ARM_ACCEPTED
    @test assignment_start(dev, 2) == 0
    @test write_word!(dev, 2, 100.0, 0.1)
    @test dev.words == [(2.0, 0.0, 100.0, 0.1)]
    release!(dev, 2)
    @test assignment_start(dev, 2) == typemax(Int64)
end

@testset "SimulatedDevice dumps inside a code period when told to" begin
    dev = SimulatedDevice(CORE_SYSTEM; sampling_freq = CORE_FS, num_channels = 1, dump_interval_samples = 1000)
    shifts = ntuple(i -> Int32(i <= 3 ? 2 - i : 0), 5)
    HardwareLoopCore.arm!(dev, 1, ArmSpec(CORE_SYSTEM, 1, 0.0, 0.0, 0.0, 0, shifts, 3, 1, 1, 1, CORE_FS, 1.0, 1.0))
    # Four dumps and the strobe per code period.
    @test correlate_chunk!(dev, zeros(ComplexF64, CORE_EPOCH)) == 5
    records = DeviceRecord[]
    @test read_records!(dev, records) == 5
    @test [r.integrated_samples for r in records if !is_strobe(r)] == fill(1000, 4)
    @test is_strobe(records[end]) && records[end].sample_index == CORE_EPOCH
    @test read_records!(dev, records) == 0
end

@testset "SimulatedDevice holds records back by the record delay" begin
    dev = SimulatedDevice(CORE_SYSTEM; sampling_freq = CORE_FS, num_channels = 1, record_delay_samples = 2000)
    correlate_chunk!(dev, zeros(ComplexF64, CORE_EPOCH))
    records = DeviceRecord[]
    @test read_records!(dev, records) == 0
    correlate_chunk!(dev, zeros(ComplexF64, 2000))
    @test read_records!(dev, records) == 1
    @test is_strobe(only(records))
end

@testset "SimulatedDevice runs its carrier NCO at the IF plus the Doppler" begin
    shifts = ntuple(i -> Int32(i <= 3 ? 2 - i : 0), 5)
    # PRN 1 at a 1 kHz Doppler on a 1 kHz IF, without noise.
    samples = [get_code(CORE_SYSTEM, 1.023e6 * (k - 1) / CORE_FS, 1) * cis(2π * 2000.0 * (k - 1) / CORE_FS)
               for k = 1:CORE_EPOCH]
    function prompt(intermediate_frequency)
        dev = SimulatedDevice(CORE_SYSTEM; sampling_freq = CORE_FS, num_channels = 1, intermediate_frequency)
        HardwareLoopCore.arm!(dev, 1, ArmSpec(CORE_SYSTEM, 1, 1000.0, 0.0, 0.0, 0, shifts, 3, 1, 1, 1, CORE_FS, 1.0, 1.0))
        correlate_chunk!(dev, samples)
        records = DeviceRecord[]
        read_records!(dev, records)
        abs(first(r for r in records if !is_strobe(r)).taps[2])
    end
    @test prompt(1000.0) ≈ CORE_EPOCH rtol = 1e-3
    # Without the IF the wipe-off is a whole carrier cycle per code period off.
    @test prompt(0.0) < 0.01CORE_EPOCH
end

@testset "The core refuses a segment whose band table is not the driver's" begin
    dev = SimulatedDevice(CORE_SYSTEM; sampling_freq = CORE_FS, num_channels = 2, intermediate_frequency = -83.0)
    @test only(driver_capabilities(dev).bands).intermediate_frequency_hz == -83.0
    other = create_segment(nothing, SegmentConfig(; channel_count = 2, bands = [BandEntry(Symbol(only(dev.bands).band_id), CORE_FS)]))
    @test_throws "band table differs" LoopCore(dev, (CORE_SYSTEM,), other)
    same = create_segment(nothing, SegmentConfig(; channel_count = 2, bands = dev.bands))
    @test band_table(LoopCore(dev, (CORE_SYSTEM,), same).segment) == dev.bands
end
