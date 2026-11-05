# What depends on the signal and the device's record grid: secondary
# codes, records shorter than a code period, dump grids that do not divide the
# code period, coherent integration over a whole bit, several antennas and
# several bands.

# A pilot record of GPS L5Q in lock: the prompt carries the secondary code chip of
# code period `k` (1-based) and no data.
function l5q_record(signal, channel, prn, k; samples = CORE_INTERVAL, periods = 1)
    chip = GNSSSignals.secondary_value(get_secondary_code(signal), prn, mod(k - 1, get_secondary_code_length(signal)))
    epl_record(channel, prn, k * CORE_INTERVAL, chip; samples, code_phase = 0.0)
end

@testset "A secondary code is wiped once its phase is known, and never at a guess" begin
    signal = GPSL5Q()
    f = scripted_fixture((signal,))
    arm!(f, 1, 7; doppler = 0.0, code_phase = 0.0, sequence = 1, signal = "GPSL5Q")
    service_pass!(f.core; wait_ms = 0)
    @test f.core.channels.secondary_wipe[1]
    feed!(k) = begin
        f.dev.sample_count = k * CORE_INTERVAL
        push!(f.dev.queue, l5q_record(signal, 1, 7, k), strobe_record(k * CORE_INTERVAL))
        service_pass!(f.core; wait_ms = 0)
    end
    polarity() = f.core.banks[1].states[1].last_carrier_phase_polarity
    # Before sync the record's sign is unknown to the loop; once the secondary code is
    # wiped it is the prompt's, and the loop reads it with the four-quadrant
    # discriminators.
    feed!(1)
    @test polarity() == 0
    foreach(feed!, 2:200)
    @test polarity() == 1
    ev = drain_events!(f, 1)
    @test ev.states[end][2].flags & HLP.STATE_BIT_OR_SECONDARY_CODE_SYNCED != 0
    wiped = [r.flags & HLP.INTEGRATION_SECONDARY_CODE_WIPED != 0 for (_, r) in ev.integrations]
    @test !wiped[1] && wiped[end]
    # Wiped, the prompts no longer change sign with the secondary code.
    @test all(real(r.prompt) > 0 for ((_, r), w) in zip(ev.integrations, wiped) if w)
    # Post-sync the code phase wraps at the whole secondary code period, not the primary code.
    @test HardwareLoopCore._post_sync_code_length(signal) == 10230 * 20
    @test 0 <= f.core.channels.code_phase[1] < 10230 * 20
    # A record that does not start where the secondary code phase stands stops the
    # wipe until sync re-seeds the phase.
    foreach(feed!, 202:203)
    @test f.core.channels.secondary_code_phase[1] == -1
    @test polarity() == 0
    ev = drain_events!(f, 1)
    @test !(last(ev.integrations)[2].flags & HLP.INTEGRATION_SECONDARY_CODE_WIPED != 0)
end

@testset "A secondary code is not wiped across a record that spans two code periods" begin
    signal = GPSL5Q()
    f = scripted_fixture((signal,))
    arm!(f, 1, 7; doppler = 0.0, code_phase = 0.0, sequence = 1, signal = "GPSL5Q")
    service_pass!(f.core; wait_ms = 0)
    for k = 1:200
        f.dev.sample_count = k * CORE_INTERVAL
        push!(f.dev.queue, l5q_record(signal, 1, 7, k), strobe_record(k * CORE_INTERVAL))
        service_pass!(f.core; wait_ms = 0)
    end
    @test f.core.channels.secondary_code_phase[1] >= 0
    f.dev.sample_count = 202CORE_INTERVAL
    push!(f.dev.queue, epl_record(1, 7, 202CORE_INTERVAL, 1.0; samples = 2CORE_INTERVAL, code_phase = 0.0),
          strobe_record(201CORE_INTERVAL), strobe_record(202CORE_INTERVAL), strobe_record(203CORE_INTERVAL))
    f.dev.sample_count = 203CORE_INTERVAL
    drain_events!(f, 1)
    service_pass!(f.core; wait_ms = 0)
    # That record goes to the loop unwiped; sync still stands, so the secondary
    # code phase is re-seeded from the bit synchronizer at the end of the same interval.
    tag, integration = last(drain_events!(f, 1).integrations)
    @test tag.sample_index == 202CORE_INTERVAL
    @test integration.num_integrated_samples == 2CORE_INTERVAL
    @test integration.flags & HLP.INTEGRATION_SECONDARY_CODE_WIPED == 0
    @test f.core.channels.secondary_code_phase[1] >= 0
end

@testset "A secondary code the receiver wipes itself is left alone" begin
    f = scripted_fixture((GPSL5Q(),))
    arm!(f, 1, 7; doppler = 0.0, code_phase = 0.0, sequence = 1, signal = "GPSL5Q",
         secondary_code_mode = HLP.SECONDARY_CODE_DEVICE_WIPES)
    service_pass!(f.core; wait_ms = 0)
    @test !f.core.channels.secondary_wipe[1]
end

@testset "A code period longer than max_integration_time is cut into partial records" begin
    # Galileo E1B: a 4 ms code period, here stepped every millisecond.
    signal = GalileoE1B()
    cfg = LoopConfig(; nco_update_interval_samples = CORE_INTERVAL, max_integration_time = 1e-3)
    f = scripted_fixture((signal,); config = cfg)
    arm!(f, 1, 7; doppler = 0.0, code_phase = 0.0, sequence = 1, signal = "GalileoE1B", num_correlators = 5)
    service_pass!(f.core; wait_ms = 0)
    for k = 1:40
        f.dev.sample_count = k * CORE_INTERVAL
        outputs = pack_correlator_outputs(ComplexF64[1000, 2000, 4000, 2000, 1000])
        push!(f.dev.queue, DeviceRecord(1, 7, k * CORE_INTERVAL, CORE_INTERVAL, outputs, 5; code_phase = mod(1023k, 4092)),
              strobe_record(k * CORE_INTERVAL))
        service_pass!(f.core; wait_ms = 0)
    end
    ev = drain_events!(f, 1)
    @test length(ev.integrations) == 39
    @test all(r.num_integrated_samples == CORE_INTERVAL for (_, r) in ev.integrations)
    @test f.core.nco_commands_committed >= 38
end

@testset "Records shorter than a code period are summed into one" begin
    f = scripted_fixture()
    arm!(f, 1, 7; doppler = 0.0, code_phase = 0.0, sequence = 1)
    service_pass!(f.core; wait_ms = 0)
    for k = 1:20
        f.dev.sample_count = k * CORE_INTERVAL
        push!(f.dev.queue,
              epl_record(1, 7, k * CORE_INTERVAL - 2000, 1.0; samples = 2000, code_phase = 511.5),
              epl_record(1, 7, k * CORE_INTERVAL, 1.0; samples = 2000, code_phase = 0.0),
              strobe_record(k * CORE_INTERVAL))
        service_pass!(f.core; wait_ms = 0)
    end
    ev = drain_events!(f, 1)
    @test length(ev.integrations) == 19
    @test all(r.num_integrated_samples == CORE_INTERVAL for (_, r) in ev.integrations)
    @test all(r.num_code_periods == 1 for (_, r) in ev.integrations)
    @test f.core.channels.records_ingested[1] == 38
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
    emitted = sum(r.num_integrated_samples for (_, r) in ev.integrations)
    # Nothing is lost: what has not been emitted is still accumulating.
    # Nothing is lost: what has not been emitted is the open accumulation, or
    # the last record, which is ingested in the next pass.
    @test emitted + f.core.channels.partial_samples[1] + samples == total
    # A 3000-sample grid lands on a boundary every 12 000 samples, and hands
    # over 9000 samples once a whole period past the target in between.
    @test length(ev.integrations) >= 25
    @test f.core.nco_commands_committed >= 25
end

@testset "num_code_periods_to_integrate = 0 integrates one whole bit after sync" begin
    cfg = LoopConfig(; nco_update_interval_samples = CORE_INTERVAL, num_code_periods_to_integrate = 0)
    f = scripted_fixture(; config = cfg)
    arm!(f, 1, 7; doppler = 0.0, code_phase = 0.0, sequence = 1)
    service_pass!(f.core; wait_ms = 0)
    feed_epl!(f, 1, 7, 400)
    ev = drain_events!(f, 1)
    blocks = [r.num_code_periods for (_, r) in ev.integrations]
    @test blocks[1] == 1
    @test count(==(20), blocks) >= 5
    @test all(b -> 1 <= b <= 20, blocks)
end

@testset "Two antennas: records, correlator output events and the loop" begin
    f = scripted_fixture(; num_antennas = 2, config = LoopConfig(; nco_update_interval_samples = CORE_INTERVAL, publish_correlator_outputs = true))
    arm!(f, 1, 7; doppler = 0.0, code_phase = 0.0, sequence = 1)
    service_pass!(f.core; wait_ms = 0)
    for k = 1:30
        f.dev.sample_count = k * CORE_INTERVAL
        outputs = ComplexF64[2000, 4000, 2000, 1000, 2000, 1000]   # antenna 2 at half the amplitude
        push!(f.dev.queue, DeviceRecord(1, 7, k * CORE_INTERVAL, CORE_INTERVAL, pack_correlator_outputs(outputs), 3; num_antennas = 2,
                                        code_phase = 0.0),
              strobe_record(k * CORE_INTERVAL))
        service_pass!(f.core; wait_ms = 0)
    end
    ev = drain_events!(f, 1)
    @test length(ev.integrations) == 29
    @test length(ev.correlator_outputs) == 29
    @test ev.correlator_outputs[end][1].num_correlators == 3
    @test ev.correlator_outputs[end][2].correlator_outputs[1:6] == (2000, 4000, 2000, 1000, 2000, 1000)
    @test all(iszero, ev.correlator_outputs[end][2].correlator_outputs[7:end])
    @test f.core.nco_commands_committed >= 28
end

@testset "A channel wants its correlator outputs even when the loop does not publish them" begin
    f = scripted_fixture()
    arm!(f, 1, 7; doppler = 0.0, code_phase = 0.0, sequence = 1, want_correlator_outputs = true)
    arm!(f, 2, 8; doppler = 0.0, code_phase = 0.0, sequence = 2)
    service_pass!(f.core; wait_ms = 0)
    for k = 1:5
        f.dev.sample_count = k * CORE_INTERVAL
        push!(f.dev.queue, epl_record(1, 7, k * CORE_INTERVAL, 1.0), epl_record(2, 8, k * CORE_INTERVAL, 1.0),
              strobe_record(k * CORE_INTERVAL))
        service_pass!(f.core; wait_ms = 0)
    end
    @test length(drain_events!(f, 1).correlator_outputs) == 4
    @test isempty(drain_events!(f, 2).correlator_outputs)
end

@testset "A channel on a second band counts on that band's counter" begin
    bands = [BandEntry("L1", CORE_FS), BandEntry("L1b", 2CORE_FS)]
    f = scripted_fixture(; bands)
    arm!(f, 1, 7; doppler = 0.0, code_phase = 0.0, sequence = 1, band = 2)
    service_pass!(f.core; wait_ms = 0)
    for k = 1:10
        f.dev.sample_count = k * CORE_INTERVAL
        push!(f.dev.queue, epl_record(1, 7, 2k * CORE_INTERVAL, 1.0; samples = 2CORE_INTERVAL, band = 2, code_phase = 0.0),
              strobe_record(k * CORE_INTERVAL))
        service_pass!(f.core; wait_ms = 0)
    end
    ev = drain_events!(f, 1)
    @test length(ev.integrations) == 9
    @test all(tag.band == 2 for (tag, _) in ev.states)
    # Interval boundaries on the reference counter, stated on band 2's.
    @test [Int(tag.sample_index) for (tag, _) in ev.states] == [2k * CORE_INTERVAL for k = 2:10]
    @test f.core.channels.phase_ref_sample[1] == 20CORE_INTERVAL
    @test f.core.nco_commands_committed >= 8
end

@testset "Two antennas: the noise reference measures the per-antenna density" begin
    f = scripted_fixture(; num_antennas = 2)
    arm!(f, 1, 7; doppler = 0.0, code_phase = 0.0, sequence = 1)
    arm!(f, 4, 30; doppler = 0.0, code_phase = 0.0, sequence = 2, signal_index = 0)
    service_pass!(f.core; wait_ms = 0)
    rng = Xoshiro(7)
    noise = Vector{ComplexF64}(undef, 6)
    for k = 1:300
        f.dev.sample_count = k * CORE_INTERVAL
        # Every correlator of every antenna sums CORE_INTERVAL unit-variance samples.
        randn!(rng, noise)
        push!(f.dev.queue,
              DeviceRecord(4, 30, k * CORE_INTERVAL, CORE_INTERVAL, pack_correlator_outputs(noise .* sqrt(CORE_INTERVAL)), 3; num_antennas = 2),
              DeviceRecord(1, 7, k * CORE_INTERVAL, CORE_INTERVAL,
                           pack_correlator_outputs(ComplexF64[200, 400, 200, 200, 400, 200]), 3; num_antennas = 2, code_phase = 0.0),
              strobe_record(k * CORE_INTERVAL))
        service_pass!(f.core; wait_ms = 0)
    end
    @test f.core.bands[1].noise_density_ready
    @test 0.9 < f.core.bands[1].noise_density * CORE_FS < 1.1
    # The satellite reads its C/N₀ against it: 0.1² · 4 MS/s is 46 dBHz.
    ev = drain_events!(f, 1)
    @test last(ev.integrations)[2].flags & HLP.INTEGRATION_HAS_CN0 != 0
    @test 40 < 10log10(last(ev.states)[2].cn0_hz) < 52
end

@testset "A data passenger is read in its pilot driver's carrier frame" begin
    # GPS L5: the Q pilot drives, the I data channel rides on its NCO commands. In the
    # pilot's frame (offset -π/2) the data sits in quadrature to the pilot.
    pilot, data = GPSL5Q(), GPSL5I()
    f = scripted_fixture((pilot, data))
    arm!(f, 1, 7; doppler = 0.0, code_phase = 0.0, sequence = 1, signal = "GPSL5Q", group_key = "L5-7")
    arm!(f, 2, 7; doppler = 0.0, code_phase = 0.0, sequence = 2, signal = "GPSL5I", group_key = "L5-7",
         signal_index = 2)
    service_pass!(f.core; wait_ms = 0)
    @test f.core.channels.driver_channel[2] == 1
    bits = [1.0, -1.0, -1.0, 1.0, -1.0, 1.0, 1.0, 1.0, -1.0, -1.0]
    for k = 1:400
        f.dev.sample_count = k * CORE_INTERVAL
        q_chip = GNSSSignals.secondary_value(get_secondary_code(pilot), 7, mod(k - 1, 20))
        i_chip = GNSSSignals.secondary_value(get_secondary_code(data), 7, mod(k - 1, 10))
        bit = bits[mod(div(k - 1, 10), length(bits))+1]
        push!(f.dev.queue,
              epl_record(1, 7, k * CORE_INTERVAL, q_chip; code_phase = 0.0),
              epl_record(2, 7, k * CORE_INTERVAL, im * bit * i_chip; code_phase = 0.0),
              strobe_record(k * CORE_INTERVAL))
        service_pass!(f.core; wait_ms = 0)
    end
    ev = drain_events!(f, 2)
    @test ev.states[end][2].flags & HLP.STATE_BIT_OR_SECONDARY_CODE_SYNCED != 0
    @test length(ev.bits) >= 20
    # The decoded soft bits follow the bit sequence (up to the polarity).
    signs = [sign(b.soft_bit) for (_, b) in ev.bits]
    n = length(bits)
    @test any(all(signs[i] == s * bits[mod(i + shift - 1, n)+1] for i in eachindex(signs))
              for shift = 0:n-1, s in (1, -1))
end
