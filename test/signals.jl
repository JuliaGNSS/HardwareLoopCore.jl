# What depends on the signal and the device's record grid: overlay (secondary)
# codes, records shorter than a code period, dump grids that do not divide the
# code period, coherent accumulation over a whole symbol, several antennas and
# several bands.

# A pilot record of GPS L5Q in lock: the prompt carries the overlay chip of
# code period `k` (1-based) and no data.
function l5q_record(signal, channel, prn, k; samples = CORE_EPOCH, periods = 1)
    chip = GNSSSignals.secondary_value(get_secondary_code(signal), prn, mod(k - 1, get_secondary_code_length(signal)))
    epl_record(channel, prn, k * CORE_EPOCH, chip; samples, code_phase = 0.0)
end

@testset "An overlay code is wiped once its phase is known, and never at a guess" begin
    signal = GPSL5Q()
    f = scripted_fixture((signal,))
    arm!(f, 1, 7; doppler = 0.0, code_phase = 0.0, sequence = 1, signal = "GPSL5Q")
    service_pass!(f.core; wait_ms = 0)
    @test f.core.channels.secondary_wipe[1]
    feed!(k) = begin
        f.dev.sample_count = k * CORE_EPOCH
        push!(f.dev.queue, l5q_record(signal, 1, 7, k), strobe_record(k * CORE_EPOCH))
        service_pass!(f.core; wait_ms = 0)
    end
    foreach(feed!, 1:200)
    ev = drain_events!(f, 1)
    @test ev.states[end][2].flags & HLP.STATE_SYNC_FOUND != 0
    wiped = [r.flags & HLP.RECORD_OVERLAY_WIPED != 0 for (_, r) in ev.records]
    @test !wiped[1] && wiped[end]
    # Wiped, the prompts no longer change sign with the overlay.
    @test all(real(r.prompt) > 0 for ((_, r), w) in zip(ev.records, wiped) if w)
    # Post-sync the code phase wraps at the whole overlay period, not the primary code.
    @test HardwareLoopCore._post_sync_code_length(signal) == 10230 * 20
    @test 0 <= f.core.channels.code_phase[1] < 10230 * 20
    # A record that does not start where the overlay counter stands stops the
    # wipe until sync re-seeds the phase.
    foreach(feed!, 202:203)
    @test f.core.channels.secondary_phase[1] == -1
    ev = drain_events!(f, 1)
    @test !(last(ev.records)[2].flags & HLP.RECORD_OVERLAY_WIPED != 0)
end

@testset "An overlay code is not wiped across a record that spans two blocks" begin
    signal = GPSL5Q()
    f = scripted_fixture((signal,))
    arm!(f, 1, 7; doppler = 0.0, code_phase = 0.0, sequence = 1, signal = "GPSL5Q")
    service_pass!(f.core; wait_ms = 0)
    for k = 1:200
        f.dev.sample_count = k * CORE_EPOCH
        push!(f.dev.queue, l5q_record(signal, 1, 7, k), strobe_record(k * CORE_EPOCH))
        service_pass!(f.core; wait_ms = 0)
    end
    @test f.core.channels.secondary_phase[1] >= 0
    f.dev.sample_count = 202CORE_EPOCH
    push!(f.dev.queue, epl_record(1, 7, 202CORE_EPOCH, 1.0; samples = 2CORE_EPOCH, code_phase = 0.0),
          strobe_record(201CORE_EPOCH), strobe_record(202CORE_EPOCH), strobe_record(203CORE_EPOCH))
    f.dev.sample_count = 203CORE_EPOCH
    drain_events!(f, 1)
    service_pass!(f.core; wait_ms = 0)
    # That record goes to the loop unwiped; sync still stands, so the overlay
    # phase is re-seeded from the bit buffer at the end of the same epoch.
    tag, record = last(drain_events!(f, 1).records)
    @test tag.device_sample == 202CORE_EPOCH
    @test record.integrated_samples == 2CORE_EPOCH
    @test record.flags & HLP.RECORD_OVERLAY_WIPED == 0
    @test f.core.channels.secondary_phase[1] >= 0
end

@testset "An overlay the receiver wipes itself is left alone" begin
    f = scripted_fixture((GPSL5Q(),))
    arm!(f, 1, 7; doppler = 0.0, code_phase = 0.0, sequence = 1, signal = "GPSL5Q",
         secondary_code_mode = HLP.SECONDARY_WIPEOFF)
    service_pass!(f.core; wait_ms = 0)
    @test !f.core.channels.secondary_wipe[1]
end

@testset "A code period longer than max_integration_time is cut into partial records" begin
    # Galileo E1B: a 4 ms code period, here stepped every millisecond.
    signal = GalileoE1B()
    cfg = LoopConfig(; epoch_length = CORE_EPOCH, max_integration_time = 1e-3)
    f = scripted_fixture((signal,); config = cfg)
    arm!(f, 1, 7; doppler = 0.0, code_phase = 0.0, sequence = 1, signal = "GalileoE1B", num_taps = 5)
    service_pass!(f.core; wait_ms = 0)
    for k = 1:40
        f.dev.sample_count = k * CORE_EPOCH
        taps = pack_taps(ComplexF64[1000, 2000, 4000, 2000, 1000])
        push!(f.dev.queue, DeviceRecord(1, 7, k * CORE_EPOCH, CORE_EPOCH, taps, 5; code_phase = mod(1023k, 4092)),
              strobe_record(k * CORE_EPOCH))
        service_pass!(f.core; wait_ms = 0)
    end
    ev = drain_events!(f, 1)
    @test length(ev.records) == 39
    @test all(r.integrated_samples == CORE_EPOCH for (_, r) in ev.records)
    @test f.core.words_committed >= 38
end

@testset "Records shorter than a code period are summed into one" begin
    f = scripted_fixture()
    arm!(f, 1, 7; doppler = 0.0, code_phase = 0.0, sequence = 1)
    service_pass!(f.core; wait_ms = 0)
    for k = 1:20
        f.dev.sample_count = k * CORE_EPOCH
        push!(f.dev.queue,
              epl_record(1, 7, k * CORE_EPOCH - 2000, 1.0; samples = 2000, code_phase = 511.5),
              epl_record(1, 7, k * CORE_EPOCH, 1.0; samples = 2000, code_phase = 0.0),
              strobe_record(k * CORE_EPOCH))
        service_pass!(f.core; wait_ms = 0)
    end
    ev = drain_events!(f, 1)
    @test length(ev.records) == 19
    @test all(r.integrated_samples == CORE_EPOCH for (_, r) in ev.records)
    @test all(r.block_credit == 1 for (_, r) in ev.records)
    @test f.core.channels.records_folded[1] == 38
end

@testset "A dump grid that does not divide the code period still steps the loop" for samples in (3000, 6000)
    f = scripted_fixture()
    arm!(f, 1, 7; doppler = 0.0, code_phase = 0.0, sequence = 1)
    service_pass!(f.core; wait_ms = 0)
    total = 0
    for k = 1:60
        total += samples
        f.dev.sample_count = total
        push!(f.dev.queue, epl_record(1, 7, total, 1.0; samples, code_phase = NaN), strobe_record(total))
        service_pass!(f.core; wait_ms = 0)
    end
    ev = drain_events!(f, 1)
    emitted = sum(r.integrated_samples for (_, r) in ev.records)
    # Nothing is lost: what has not been emitted is still accumulating.
    # Nothing is lost: what has not been emitted is the open accumulation, or
    # the last record, which is folded in the next pass.
    @test emitted + f.core.channels.partial_samples[1] + samples == total
    # A 3000-sample grid lands on a boundary every 12 000 samples, and hands
    # over 9000 samples once a whole period past the target in between.
    @test length(ev.records) >= 25
    @test f.core.words_committed >= 25
end

@testset "coherent_code_blocks = 0 accumulates one whole symbol after sync" begin
    cfg = LoopConfig(; epoch_length = CORE_EPOCH, coherent_code_blocks = 0)
    f = scripted_fixture(; config = cfg)
    arm!(f, 1, 7; doppler = 0.0, code_phase = 0.0, sequence = 1)
    service_pass!(f.core; wait_ms = 0)
    feed_epl!(f, 1, 7, 400)
    ev = drain_events!(f, 1)
    blocks = [r.block_credit for (_, r) in ev.records]
    @test blocks[1] == 1
    @test count(==(20), blocks) >= 5
    @test all(b -> 1 <= b <= 20, blocks)
end

@testset "Two antennas: records, taps events and the loop" begin
    f = scripted_fixture(; num_ants = 2, config = LoopConfig(; epoch_length = CORE_EPOCH, publish_taps = true))
    arm!(f, 1, 7; doppler = 0.0, code_phase = 0.0, sequence = 1)
    service_pass!(f.core; wait_ms = 0)
    for k = 1:30
        f.dev.sample_count = k * CORE_EPOCH
        taps = ComplexF64[2000, 4000, 2000, 1000, 2000, 1000]   # antenna 2 at half the amplitude
        push!(f.dev.queue, DeviceRecord(1, 7, k * CORE_EPOCH, CORE_EPOCH, pack_taps(taps), 3; num_ants = 2,
                                        code_phase = 0.0),
              strobe_record(k * CORE_EPOCH))
        service_pass!(f.core; wait_ms = 0)
    end
    ev = drain_events!(f, 1)
    @test length(ev.records) == 29
    @test length(ev.taps) == 29
    @test ev.taps[end][1].num_taps == 3
    @test ev.taps[end][2].taps[1:6] == (2000, 4000, 2000, 1000, 2000, 1000)
    @test all(iszero, ev.taps[end][2].taps[7:end])
    @test f.core.words_committed >= 28
end

@testset "A channel wants its taps even when the loop does not publish them" begin
    f = scripted_fixture()
    arm!(f, 1, 7; doppler = 0.0, code_phase = 0.0, sequence = 1, want_taps = true)
    arm!(f, 2, 8; doppler = 0.0, code_phase = 0.0, sequence = 2)
    service_pass!(f.core; wait_ms = 0)
    for k = 1:5
        f.dev.sample_count = k * CORE_EPOCH
        push!(f.dev.queue, epl_record(1, 7, k * CORE_EPOCH, 1.0), epl_record(2, 8, k * CORE_EPOCH, 1.0),
              strobe_record(k * CORE_EPOCH))
        service_pass!(f.core; wait_ms = 0)
    end
    @test length(drain_events!(f, 1).taps) == 4
    @test isempty(drain_events!(f, 2).taps)
end

@testset "A channel on a second band counts on that band's counter" begin
    bands = [BandEntry("L1", CORE_FS), BandEntry("L1b", 2CORE_FS)]
    f = scripted_fixture(; bands)
    arm!(f, 1, 7; doppler = 0.0, code_phase = 0.0, sequence = 1, band = 2, sampling_freq_hz = 2CORE_FS)
    service_pass!(f.core; wait_ms = 0)
    for k = 1:10
        f.dev.sample_count = k * CORE_EPOCH
        push!(f.dev.queue, epl_record(1, 7, 2k * CORE_EPOCH, 1.0; samples = 2CORE_EPOCH, band = 2, code_phase = 0.0),
              strobe_record(k * CORE_EPOCH))
        service_pass!(f.core; wait_ms = 0)
    end
    ev = drain_events!(f, 1)
    @test length(ev.records) == 9
    @test all(tag.band == 2 for (tag, _) in ev.states)
    # Epoch boundaries on the reference counter, stated on band 2's.
    @test [Int(tag.device_sample) for (tag, _) in ev.states] == [2k * CORE_EPOCH for k = 2:10]
    @test f.core.channels.phase_ref_sample[1] == 20CORE_EPOCH
    @test f.core.words_committed >= 8
end

@testset "Two antennas: the noise reference measures the per-antenna density" begin
    f = scripted_fixture(; num_ants = 2)
    arm!(f, 1, 7; doppler = 0.0, code_phase = 0.0, sequence = 1)
    arm!(f, 4, 30; doppler = 0.0, code_phase = 0.0, sequence = 2, signal_index = 0)
    service_pass!(f.core; wait_ms = 0)
    rng = Xoshiro(7)
    noise = Vector{ComplexF64}(undef, 6)
    for k = 1:300
        f.dev.sample_count = k * CORE_EPOCH
        # Every tap of every antenna sums CORE_EPOCH unit-variance samples.
        randn!(rng, noise)
        push!(f.dev.queue,
              DeviceRecord(4, 30, k * CORE_EPOCH, CORE_EPOCH, pack_taps(noise .* sqrt(CORE_EPOCH)), 3; num_ants = 2),
              DeviceRecord(1, 7, k * CORE_EPOCH, CORE_EPOCH,
                           pack_taps(ComplexF64[200, 400, 200, 200, 400, 200]), 3; num_ants = 2, code_phase = 0.0),
              strobe_record(k * CORE_EPOCH))
        service_pass!(f.core; wait_ms = 0)
    end
    @test f.core.bands[1].noise_density_ready
    @test 0.9 < f.core.bands[1].noise_density * CORE_FS < 1.1
    # The satellite reads its C/N₀ against it: 0.1² · 4 MS/s is 46 dBHz.
    ev = drain_events!(f, 1)
    @test last(ev.records)[2].flags & HLP.RECORD_HAS_CN0 != 0
    @test 40 < 10log10(last(ev.states)[2].cn0_linear_hz) < 52
end

@testset "A data passenger is read in its pilot driver's carrier frame" begin
    # GPS L5: the Q pilot drives, the I data channel rides on its words. In the
    # pilot's frame (offset -π/2) the data sits in quadrature to the pilot.
    pilot, data = GPSL5Q(), GPSL5I()
    f = scripted_fixture((pilot, data))
    arm!(f, 1, 7; doppler = 0.0, code_phase = 0.0, sequence = 1, signal = "GPSL5Q", group_key = "L5-7")
    arm!(f, 2, 7; doppler = 0.0, code_phase = 0.0, sequence = 2, signal = "GPSL5I", group_key = "L5-7",
         signal_index = 2)
    service_pass!(f.core; wait_ms = 0)
    @test f.core.channels.driver_channel[2] == 1
    symbols = [1.0, -1.0, -1.0, 1.0, -1.0, 1.0, 1.0, 1.0, -1.0, -1.0]
    for k = 1:400
        f.dev.sample_count = k * CORE_EPOCH
        q_chip = GNSSSignals.secondary_value(get_secondary_code(pilot), 7, mod(k - 1, 20))
        i_chip = GNSSSignals.secondary_value(get_secondary_code(data), 7, mod(k - 1, 10))
        symbol = symbols[mod(div(k - 1, 10), length(symbols))+1]
        push!(f.dev.queue,
              epl_record(1, 7, k * CORE_EPOCH, q_chip; code_phase = 0.0),
              epl_record(2, 7, k * CORE_EPOCH, im * symbol * i_chip; code_phase = 0.0),
              strobe_record(k * CORE_EPOCH))
        service_pass!(f.core; wait_ms = 0)
    end
    ev = drain_events!(f, 2)
    @test ev.states[end][2].flags & HLP.STATE_SYNC_FOUND != 0
    @test length(ev.bits) >= 20
    # The decoded soft bits follow the symbol sequence (up to the polarity).
    signs = [sign(b.soft_bit) for (_, b) in ev.bits]
    n = length(symbols)
    @test any(all(signs[i] == s * symbols[mod(i + shift - 1, n)+1] for i in eachindex(signs))
              for shift = 0:n-1, s in (1, -1))
end
