# Usage

This page closes a tracking loop end to end in one process: the
[`SimulatedDevice`](@ref) stands in for the FPGA, a heap-backed segment stands
in for the shared memory, and the test code plays the receiver, sending an arm
command and reading the events.

## Building the core

A [`LoopCore`](@ref) needs a driver, the signals it can track, and the segment
it publishes into. The segment needs one event ring per hardware channel, and
its band table should match the driver's.

```@example usage
using HardwareLoopCore, HardwareLoopProtocol, GNSSSignals
using Random: Xoshiro
const HLP = HardwareLoopProtocol

signal = GPSL1CA()
fs = 4e6
dev = SimulatedDevice(signal; sampling_freq = fs, num_channels = 2)
seg = create_segment(nothing, SegmentConfig(; channel_count = 2, bands = [BandEntry(:L1, fs)]))
core = LoopCore(dev, (signal,), seg)
core.config.epoch_length
```

The epoch defaults to one primary code period of the first signal, here 1 ms
or 4000 samples. A [`LoopConfig`](@ref) passed as `config` overrides it and the
other loop-wide settings; the receiver can change them later with a
`ConfigureCommand`.

## Arming a channel

The receiver hands over a satellite with an `ArmCommand` on the command ring:
the signal, the PRN, the Dopplers and the code phase it measured at
`valid_at_sample`, and the tap offsets in samples. The core programs the
device, and answers once the device has confirmed. Here channel 1 tracks the
satellite and channel 2 becomes the band's noise reference (`signal_index = 0`),
which the C/N₀ estimate is measured against.

```@example usage
prn, doppler, code_phase = 7, 1200.0, 100.0
commands = command_ring(seg)
satellite = ArmCommand(;
    signal = :GPSL1CA,
    prn,
    carrier_doppler_hz = doppler - 10,   # a handover is never exact
    code_doppler_hz = doppler / 1540,
    code_phase_chips = code_phase,
    valid_at_sample = 0,
    tap_sample_shifts = (-2, 0, 2),      # early, prompt, late
    num_taps = 3,
    sampling_freq_hz = fs,
)
noise = ArmCommand(;
    signal = :GPSL1CA,
    prn = 30,
    signal_index = 0,
    carrier_doppler_hz = 3000.0,
    code_doppler_hz = 0.0,
    code_phase_chips = 0.0,
    valid_at_sample = 0,
    tap_sample_shifts = (-2, 0, 2),
    num_taps = 3,
    sampling_freq_hz = fs,
)
try_publish!(commands, CommandTag(HLP.COMMAND_ARM, 1, 1), satellite)
try_publish!(commands, CommandTag(HLP.COMMAND_ARM, 2, 2), noise)
```

## Running the loop

In a loop process, [`run!`](@ref) calls [`service_pass!`](@ref) until a
shutdown command arrives, and the driver's [`wait_records`](@ref) paces it.
Here the samples are synthesised one epoch at a time — the satellite's code
with a navigation bit that flips every 20 ms, in noise — the device correlates
them with [`correlate_chunk!`](@ref), and one pass folds what it produced:

```@example usage
rng = Xoshiro(1)
function synthesize!(buf, n0)
    for k in eachindex(buf)
        t = (n0 + k - 1) / fs
        code = get_code(signal, code_phase + (1.023e6 + doppler / 1540) * t, prn)
        bit = isodd(div(n0 + k - 1, 80_000)) ? -1 : 1
        buf[k] = 0.15 * bit * code * cis(2π * doppler * t) + randn(rng, ComplexF64)
    end
    buf
end

buf = Vector{ComplexF64}(undef, 4000)
for c in 0:1499
    correlate_chunk!(dev, synthesize!(buf, 4000c))
    service_pass!(core; wait_ms = 0)
end
(dev.channels[1].carrier_doppler, core.words_committed)
```

The loop has pulled the device's carrier NCO from the handover's 1190 Hz to
the true 1200 Hz. Once bit synchronisation is found, the loop steps once per
primary code period as before, and the bit buffer integrates the whole bit.

## Reading the events

Each channel has an event ring: status events answer commands, a record event
follows every loop step, a bit event every navigation bit, and an epoch state
closes every epoch. The receiver drains it:

```@example usage
function drain!(events)
    counts = Dict{UInt8,Int}()
    armed_at = nothing
    while true
        status, view, lost = peek!(events, EventTag)
        status === :empty && break
        if view.tag.kind == HLP.EVENT_STATUS
            ev = payload(StatusEvent, events, view)
            ev.code == HLP.STATUS_ARMED && (armed_at = ev.sample)
        end
        counts[view.tag.kind] = get(counts, view.tag.kind, 0) + 1
        commit!(events, view)
    end
    (armed_at, records = counts[HLP.EVENT_RECORD], bits = get(counts, HLP.EVENT_BIT, 0),
     epoch_states = counts[HLP.EVENT_EPOCH_STATE])
end

drain!(event_ring(seg, 1))
```

The newest epoch state is also kept in the channel's snapshot slot, for a
receiver that only wants the current state:

```@example usage
tag, state = read_snapshot(snapshot_slot(seg, 1))
(doppler = state.carrier_doppler_hz,
 cn0_dBHz = 10log10(state.cn0_linear_hz),
 synchronised = state.flags & HLP.STATE_SYNC_FOUND != 0)
```

## Diagnostics

The core counts what went wrong instead of throwing on the service path. Among
the counters on a [`LoopCore`](@ref): `stale_dumps` (records for a channel that
was free, re-armed or not yet confirmed), `lost_record_gaps` (records that
never arrived), `words_late` and `words_rejected`, `dropped_records` (ingest
buffer full), and `latency_hist`, a histogram of the record-to-word latency over
[`LATENCY_EDGES_US`](@ref).

```@example usage
(core.stale_dumps, core.lost_record_gaps, core.words_late, core.max_record_age_us)
```
