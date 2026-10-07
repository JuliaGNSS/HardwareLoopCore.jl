# Vector tracking closed through the simulated device on a real recording: the
# ION RTL-SDR capture (GPS L1 C/A, 60 s, 2.048 MS/s, 8-bit offset-binary I/Q at
# zero IF), correlated by `SimulatedDevice`, the loops closed by the core's
# `VectorPLLAndDLL`, and everything asserted from the segment's nav ring and
# snapshot, as a receiver would read it. GNSSReceiver's
# `test/ion_rtlsdr_integration.jl` runs the same recording through the software
# receiver; its regression fix is the reference here.
#
# Source: https://sdr.ion.org/api-sample-data.html. Recorded 2017-09-10 in
# Oegstgeest, NL. Downloaded once into a Scratch.jl space, which CI's
# julia-actions/cache keeps between runs.

const ION_URL = "https://sdr.ion.org/RTL_SDR/RTLSDR_Bands-L1.uint8"
const ION_FS = 2.048e6
const ION_EPOCH = 2048                   # one C/A code period
# The healthy satellites GNSSReceiver's test locks on this recording.
const ION_PRNS = [5, 7, 8, 13, 15, 18, 20, 21, 24, 28, 30]
# GNSSReceiver's final scalar fix (ECEF, m): a pipeline baseline, not a surveyed point.
const ION_POSITION = [3.9074087926e6, 3.0683836901e5, 5.0149608655e6]
# East/north/up at the baseline (52.177° N, 4.490° E).
const ION_ENU = let lat = deg2rad(52.177), lon = deg2rad(4.490)
    [-sin(lon) cos(lon) 0; -sin(lat)*cos(lon) -sin(lat)*sin(lon) cos(lat); cos(lat)*cos(lon) cos(lat)*sin(lon) sin(lat)]
end
# Its final-fix epoch, 2017-09-10T22:57:20.697 TAI, in seconds since J2000.
const ION_TIME_TAI_S = 5.58356240697e8
# Where the samples' zero Doppler sits. The RTL-SDR's tuned frequency is off by a fixed
# ~83 Hz (its LO synthesizer's resolution), which the code does not share: scalar loops
# hold code − carrier/1540 ≈ +0.054 Hz on every satellite. A driver computes this from
# its tuner settings; the recording does not say which settings it was made with, so
# the value here is the measured one. Uncompensated, the vector
# filter's single clock drift cannot satisfy both the carrier rates and the code, and
# the conflict leaks into the height (JuliaGNSS/TrackingLoops.jl#35).
const ION_INTERMEDIATE_FREQUENCY = -83.0

function ion_recording()
    file = joinpath(@get_scratch!("rtl_sdr_test_data"), "RTLSDR_Bands-L1.uint8")
    if !isfile(file)
        @info "Downloading the ION RTL-SDR recording (~246 MB) …"
        partial = file * ".partial"
        run(`curl -sfL -o $partial $ION_URL`)
        mv(partial, file; force = true)
    end
    file
end

# Offset binary, recentred on the exact midscale.
function read_ion_samples!(buf::Vector{ComplexF64}, raw::Vector{UInt8}, io)
    readbytes!(io, raw) == length(raw) || error("the recording ended early")
    @inbounds for i in eachindex(buf)
        buf[i] = ComplexF64(raw[2i-1] - 127.5, raw[2i] - 127.5)
    end
    buf
end

function drain_ring!(ring)
    while true
        status, view, _ = peek!(ring, EventTag)
        status === :empty && return nothing
        commit!(ring, view)
    end
end

# One run over the whole recording: acquire the healthy satellites in the first
# 10 ms, arm them and a noise reference, then one service pass per code period.
function run_ion_vector(file; record_delay_epochs)
    num_channels = length(ION_PRNS) + 1
    dev = SimulatedDevice(GPSL1CA(); sampling_freq = ION_FS, num_channels,
                          record_delay_samples = record_delay_epochs * ION_EPOCH,
                          intermediate_frequency = ION_INTERMEDIATE_FREQUENCY)
    seg = create_segment(nothing, SegmentConfig(; channel_count = num_channels, bands = dev.bands))
    # The receiver's side: acquire around the IF the band table publishes.
    initial = open(io -> read_ion_samples!(zeros(ComplexF64, 10ION_EPOCH), Vector{UInt8}(undef, 20ION_EPOCH), io), file)
    acquired = acquire(GPSL1CA(), initial, ION_FS * Hz, ION_PRNS;
                       interm_freq = band_table(seg)[1].intermediate_frequency_hz * Hz,
                       num_coherently_integrated_code_periods = 2, num_noncoherent_accumulations = 5,
                       subsample_interpolation = true)
    estimator = VectorPLLAndDLL(GPSL1CA(); inner = NCOReferencedPLLAndDLL(), approximate_year = 2017)
    core = LoopCore(dev, (GPSL1CA(),), seg; estimator)
    shifts = HardwareLoopCore._template_tap_shifts(core.banks[1].template, ION_FS, GPSL1CA())
    commands = command_ring(seg)
    for (ch, acq) in enumerate(acquired)
        doppler = ustrip(Hz, acq.carrier_doppler)
        publish!(commands, CommandTag(HLP.COMMAND_ARM, ch, ch), ArmCommand(;
            signal = :GPSL1CA, prn = acq.prn, carrier_doppler_hz = doppler, code_doppler_hz = doppler / 1540,
            code_phase_chips = acq.code_phase, valid_at_sample = 0, tap_sample_shifts = shifts, num_taps = 3,
            sampling_freq_hz = ION_FS))
    end
    publish!(commands, CommandTag(HLP.COMMAND_ARM, num_channels, num_channels), ArmCommand(;
        signal = :GPSL1CA, prn = 1, signal_index = 0, carrier_doppler_hz = 3000.0, code_doppler_hz = 0.0,
        code_phase_chips = 100.0, valid_at_sample = 0, tap_sample_shifts = shifts, num_taps = 3,
        sampling_freq_hz = ION_FS))
    service_pass!(core; wait_ms = 0)

    nav = nav_ring(seg)
    solutions = Tuple{EventTag,NavSolutionEvent}[]
    satellites = Tuple{EventTag,NavSatelliteEvent}[]
    allocated_warm = 0
    filter_cycles = 0
    buf = zeros(ComplexF64, ION_EPOCH)
    raw = Vector{UInt8}(undef, 2ION_EPOCH)
    num_passes = filesize(file) ÷ (2ION_EPOCH)
    open(file) do io
        for _ = 1:num_passes
            read_ion_samples!(buf, raw, io)
            correlate_chunk!(dev, buf)
            allocated = @allocated service_pass!(core; wait_ms = 0)
            # The device's log of every word it took, which would grow.
            empty!(dev.words)
            # Warm once the filter has run a cycle past the one that seeded it
            # (the engine sizes its member report on those two): from then on
            # every pass — the cycles, the UKF and the nav publish included —
            # allocates nothing.
            filter_cycles >= 2 && (allocated_warm += allocated)
            for ch = 1:num_channels
                drain_ring!(event_ring(seg, ch))
            end
            while true
                status, view, _ = peek!(nav, EventTag)
                status === :empty && break
                if view.tag.kind == HLP.EVENT_NAV_SOLUTION
                    solution = payload(NavSolutionEvent, nav, view)
                    push!(solutions, (view.tag, solution))
                    solution.flags & HLP.NAV_RUNNING != 0 && (filter_cycles += 1)
                else
                    push!(satellites, (view.tag, payload(NavSatelliteEvent, nav, view)))
                end
                commit!(nav, view)
            end
        end
    end
    (; core, seg, solutions, satellites, allocated_warm)
end

@testset "Vector tracking on the ION RTL-SDR recording ($delay-epoch record delay)" for delay in (0, 2)
    r = run_ion_vector(ion_recording(); record_delay_epochs = delay)
    @test !isempty(r.solutions)
    @test r.core.words_late == 0
    @test r.core.words_rejected == 0
    @test consumer_lost(nav_ring(r.seg)) == 0
    # The filter seeds from the engine's first scalar fix (the navigation data
    # takes some 30 s to decode) and runs to the end.
    first_seeded = findfirst(((_, s),) -> s.flags & HLP.NAV_SEEDED != 0, r.solutions)
    @test !isnothing(first_seeded)
    @test all(s.flags & HLP.NAV_RUNNING != 0 for (_, s) in r.solutions[first_seeded:end])
    @test r.allocated_warm == 0
    tag, solution = r.solutions[end]
    @test solution.flags & (HLP.NAV_VALID | HLP.NAV_TIME_VALID) == HLP.NAV_VALID | HLP.NAV_TIME_VALID
    @test solution.num_members >= 4
    @test solution.num_sats >= 4
    # Within a few metres of the baseline. Its height is the least certain part: no
    # ionospheric coefficients are decoded from 60 s, so nothing corrects for the
    # ionosphere (the scalar fixes sit some 6 m high as well).
    @test norm(collect(solution.position_ecef_m) .- ION_POSITION) < 10.0
    enu = [ION_ENU * (collect(s.position_ecef_m) .- ION_POSITION) for (_, s) in r.solutions[first_seeded:end]]
    @test norm(sum(e -> e[1:2], enu) / length(enu)) < 5.0
    # Code and carrier agree once the front end's offset is the IF: no common-mode
    # code residual is left for the single clock drift to fight (-30 m without it).
    last_cycle_residuals = [s.residual_m for (_, s) in r.satellites if s.cycle == solution.cycle]
    @test abs(sum(last_cycle_residuals) / length(last_cycle_residuals)) < 5.0
    @test norm(collect(solution.velocity_ecef_mps)) < 5.0           # a static receiver
    @test abs(solution.time_tai_s + solution.time_tai_frac - ION_TIME_TAI_S) < 1.0
    @test all(isfinite, solution.dop) && solution.position_std_m < 10.0
    @test read_nav_snapshot(r.seg) == (tag, solution)
    # The last cycle reports every healthy satellite tracked, in lock and in the
    # solution, each on the channel it was armed on.
    last_cycle = [(t, s) for (t, s) in r.satellites if s.cycle == solution.cycle]
    @test sort([Int(t.prn) for (t, _) in last_cycle]) == ION_PRNS
    for (t, s) in last_cycle
        @test s.flags & (HLP.NAV_SAT_TRACKED | HLP.NAV_SAT_IN_LOCK | HLP.NAV_SAT_IN_SOLUTION) ==
              HLP.NAV_SAT_TRACKED | HLP.NAV_SAT_IN_LOCK | HLP.NAV_SAT_IN_SOLUTION
        @test ION_PRNS[s.channel] == t.prn
        @test isfinite(s.residual_m) && s.cn0_dbhz > 35
    end
    @test r.core.nav_events_published == length(r.solutions) + length(r.satellites)
end
