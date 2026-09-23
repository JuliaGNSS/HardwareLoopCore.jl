[![CI](https://github.com/JuliaGNSS/HardwareLoopCore.jl/actions/workflows/ci.yml/badge.svg)](https://github.com/JuliaGNSS/HardwareLoopCore.jl/actions/workflows/ci.yml)
[![codecov](https://codecov.io/gh/JuliaGNSS/HardwareLoopCore.jl/branch/main/graph/badge.svg)](https://codecov.io/gh/JuliaGNSS/HardwareLoopCore.jl)

# HardwareLoopCore.jl

The engine of a hardware correlator's dedicated, allocation-free loop process
(GNSSReceiver.jl, `docs/plans/2026-09-22-loop-process.md`).

- `AbstractLoopDriver` — what the core asks of a device: `read_records!`,
  `write_word!`, `arm!`, `release!`, `assignment_start`, `sample_count`,
  `driver_capabilities`, optionally `wait_records` / `overflowed_channels!`.
- `LoopCore` — the loop process's whole state: per-signal channel banks, the
  channel table, the epoch clock, the noise references, the
  `HardwareLoopProtocol` segment it publishes into. `service_pass!` is one pass
  (wait → read → fold closed epochs → commit words → commands → confirm arms →
  heartbeat); `run!` loops it. A warm pass allocates nothing (`test/core.jl`).
- `SimulatedDevice` — the software stand-in for the FPGA, the first driver, with
  a configurable record delay for the delay-tolerance tests.

The per-record arithmetic (discriminators, loop filters, bit buffer, C/N₀
estimators, the delay-aware `NCOReferencedPLLAndDLL` and its NCO timelines) is
[TrackingLoops.jl](https://github.com/JuliaGNSS/TrackingLoops.jl)'s, shared with Tracking.jl's software
receiver. The wire contract is [HardwareLoopProtocol.jl](https://github.com/JuliaGNSS/HardwareLoopProtocol.jl).
The LiteX-M2SDR driver and the `gnss_loop` executable are GNSSM2SDR.jl's
`M2SDRLoop/`; the receiver side is GNSSReceiver.jl's `RemoteHardwareLoop`.

```julia
using HardwareLoopCore, HardwareLoopProtocol, GNSSSignals
dev = SimulatedDevice(GPSL1CA(); sampling_freq = 4e6, num_channels = 4)
seg = create_segment(nothing, SegmentConfig(; channel_count = 4, bands = [BandEntry("L1", 4e6)]))
core = LoopCore(dev, (GPSL1CA(),), seg)
# ... publish an ArmCommand on command_ring(seg), then per chunk:
correlate_chunk!(dev, samples)
service_pass!(core; wait_ms = 0)
```
