[![Docs: stable](https://img.shields.io/badge/docs-stable-blue.svg)](https://JuliaGNSS.github.io/HardwareLoopCore.jl/stable)
[![Docs: dev](https://img.shields.io/badge/docs-dev-blue.svg)](https://JuliaGNSS.github.io/HardwareLoopCore.jl/dev)
[![Tests](https://github.com/JuliaGNSS/HardwareLoopCore.jl/actions/workflows/ci.yml/badge.svg)](https://github.com/JuliaGNSS/HardwareLoopCore.jl/actions/workflows/ci.yml)
[![Documentation](https://github.com/JuliaGNSS/HardwareLoopCore.jl/actions/workflows/Documentation.yml/badge.svg)](https://github.com/JuliaGNSS/HardwareLoopCore.jl/actions/workflows/Documentation.yml)
[![codecov](https://codecov.io/gh/JuliaGNSS/HardwareLoopCore.jl/branch/main/graph/badge.svg)](https://codecov.io/gh/JuliaGNSS/HardwareLoopCore.jl)
[![semantic-release](https://img.shields.io/badge/%20%20%F0%9F%93%A6%F0%9F%9A%80-semantic--release-e10079.svg)](https://github.com/semantic-release/semantic-release)

# HardwareLoopCore.jl

The engine of a hardware correlator's loop process: the dedicated,
allocation-free process that sits next to an FPGA correlator, reads every
correlator record it produces, closes the carrier and code loops, and writes
the new NCO words back.

- `AbstractLoopDriver` — what the core asks of a device: `read_records!`,
  `write_word!`, `arm!`, `release!`, `assignment_start`, `sample_count`,
  `driver_capabilities`, optionally `wait_records` / `overflowed_channels!`.
- `LoopCore` — the loop process's whole state: per-signal channel banks, the
  channel table, the epoch clock, the noise references, the
  `HardwareLoopProtocol` segment it publishes into. `service_pass!` is one pass
  (wait → read → fold closed epochs → commit words → commands → confirm arms →
  heartbeat); `run!` loops it. A warm pass allocates nothing. Built with
  TrackingLoops' `VectorPLLAndDLL`, it runs vector tracking and publishes the
  navigation solution on the segment's nav ring.
- `SimulatedDevice` — a software correlator behind the driver API, for tests
  and for trying the core without hardware.

The per-record arithmetic (discriminators, loop filters, bit buffer, C/N₀
estimators, the delay-aware `NCOReferencedPLLAndDLL` and its NCO timelines) is
[TrackingLoops.jl](https://github.com/JuliaGNSS/TrackingLoops.jl)'s. The wire contract between the loop process and the receiver is
[HardwareLoopProtocol.jl](https://github.com/JuliaGNSS/HardwareLoopProtocol.jl).

## Installation

```julia
using Pkg
Pkg.add(["HardwareLoopCore", "HardwareLoopProtocol", "GNSSSignals"])
```

## Example

```julia
using HardwareLoopCore, HardwareLoopProtocol, GNSSSignals
const HLP = HardwareLoopProtocol

signal = GPSL1CA()
dev = SimulatedDevice(signal; sampling_freq = 4e6, num_channels = 4)
# `nothing` backs the segment with a heap buffer; a loop process passes a file
# path under /dev/shm, which the receiver attaches to. It publishes the driver's
# band table, intermediate frequencies included.
seg = create_segment(nothing, SegmentConfig(; channel_count = 4, bands = driver_capabilities(dev).bands))
core = LoopCore(dev, (signal,), seg)

# The receiver arms channel 1 on PRN 7 …
arm = ArmCommand(; signal = :GPSL1CA, prn = 7, carrier_doppler_hz = 1200.0,
                 code_doppler_hz = 1200.0 / 1540, code_phase_chips = 100.0,
                 valid_at_sample = 0, tap_sample_shifts = (-2, 0, 2), num_taps = 3,
                 sampling_freq_hz = 4e6)
try_publish!(command_ring(seg), CommandTag(HLP.COMMAND_ARM, 1, 1), arm)

# … and per chunk of samples the device correlates, one pass closes the loop
# and publishes the events on channel 1's event ring.
samples = zeros(ComplexF64, 4000)
correlate_chunk!(dev, samples)
service_pass!(core; wait_ms = 0)
```

## Limitations

- The signal types, the antenna count and the hardware channel count are fixed
  when the core is constructed.
- One estimator per core, by default TrackingLoops' `NCOReferencedPLLAndDLL`.
  Its bandwidths are set loop-wide; the per-arm bandwidths of an `ArmCommand`
  are not applied yet.
- Vector tracking (`VectorPLLAndDLL`): data signals only, one shared engine,
  and the navigation cycle runs inside the service pass.
- A record carries at most five taps and ten tap × antenna values.
- With several antennas there is no beamforming, and the antennas are taken to
  see equal, uncorrelated noise.
- C/N₀ needs a noise reference: one hardware channel per band.
- `overflowed_channels!` is part of the driver API but not read by the core yet.
- `SimulatedDevice` is for tests: one band, one antenna, not real-time.

See the [documentation](https://JuliaGNSS.github.io/HardwareLoopCore.jl/stable)
for a closed-loop walk-through, the driver contract, and the API reference.
