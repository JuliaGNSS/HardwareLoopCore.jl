# HardwareLoopCore.jl

```@docs
HardwareLoopCore
```

## Where it sits

A receiver built on a hardware correlator runs two processes. The receiver
process acquires satellites, decides which channel tracks which signal, decodes
the navigation data and computes positions. The *loop process* sits next to the
device: it reads every correlator record the device produces, closes the
carrier and code loops, and writes the new NCO words back within a fraction of
a millisecond. This package is the engine of that loop process.

- [TrackingLoops.jl](https://github.com/JuliaGNSS/TrackingLoops.jl) provides
  the per-record arithmetic: discriminators, loop filters, bit
  synchronisation, C/N₀ estimation and the delay-aware estimator.
- [HardwareLoopProtocol.jl](https://github.com/JuliaGNSS/HardwareLoopProtocol.jl)
  is the wire contract between the two processes: a shared-memory segment with
  a command ring (receiver → loop) and per channel an event ring and an epoch
  state snapshot (loop → receiver).
- A device plugs in through an [`AbstractLoopDriver`](@ref). The package ships
  one, [`SimulatedDevice`](@ref), a software correlator for tests and for
  trying the core without hardware. See [Writing a driver](@ref) for the
  contract a real device implements.

The core is built so that the loop process can be compiled with
`juliac --trim` into a small executable: everything is sized at construction,
the driver and the signals are type parameters, and a warm
[`service_pass!`](@ref) allocates nothing.

## Why it is not part of HardwareLoopProtocol

HardwareLoopProtocol.jl is the contract both processes link against; this
package is one implementation of the loop side of it. Keeping them apart keeps
the contract small and stable:

- The receiver process needs the protocol but not the engine. The protocol
  depends on Base only; this package pulls in TrackingLoops, GNSSSignals,
  StaticArrays and Unitful, none of which a receiver needs just to read a
  segment.
- The segment layout is versioned and checked by a layout hash on attach, so
  both processes must agree on it. Changes to the loop arithmetic, the fold
  or a driver should not force a new protocol release, and a new protocol
  release should not be mixed up with loop changes.
- The protocol does not prescribe how the loop is closed. Another loop
  process — for another device, or not written in Julia at all — can speak the
  same protocol without this package.

## Signals and limitations

The core tracks any signal type that GNSSSignals defines and TrackingLoops has
a default correlator and bit/secondary-code synchronisation for (GPS L1 C/A, L1C, L2C, L5; Galileo E1, E5a, E5b, E6;
BeiDou B1I, B1C, B2a, B2b, B3I). A loop serves the signal types passed to
[`LoopCore`](@ref) at construction, mixed freely across channels. Overlay
(secondary) codes are removed once their phase is known, code periods longer
than `max_integration_time` are stepped in partial records, and a data
component can ride on its pilot's NCO words as a passenger.

The test suite closes the loop through [`SimulatedDevice`](@ref) on GPS L1 C/A
and checks GPS L5 (pilot with overlay, data passenger) and Galileo E1B (partial
records) with scripted records. The other signals go through the same code
paths but have not been run against a device.

Current limitations:

- The signal types, the antenna count and the hardware channel count are
  fixed when the core is constructed.
- One estimator per core, chosen when it is built (the `estimator` keyword of
  [`LoopCore`](@ref)): by default TrackingLoops' delay-aware
  `NCOReferencedPLLAndDLL`. Its bandwidths are set for the whole loop; the
  per-arm `carrier_loop_bandwidth_hz` and `code_loop_bandwidth_hz` of an
  `ArmCommand` are not applied yet.
- Vector tracking (a `VectorPLLAndDLL` with an `NCOReferencedPLLAndDLL` inner
  loop): data signals only — a pilot is refused as a driver and pilot/data
  pairs (passengers) are refused altogether. All satellites share one
  navigation engine, and its cycle (the scalar PVT or the filter update) runs
  inside the service pass, in the `step_loop` of one record. Inter-system and
  inter-frequency biases are not published.
- A record carries at most five taps and ten tap × antenna values
  ([`MAX_RECORD_TAPS`](@ref)).
- With several antennas, the core does no beamforming. The noise reference
  measures one density, and the antennas are taken to see equal, uncorrelated
  noise.
- C/N₀ needs a noise reference: one hardware channel per band armed on a
  decoy PRN. Without one, no C/N₀ is published.
- [`overflowed_channels!`](@ref) is part of the driver API but the core does
  not read it yet. Lost records are detected from gaps in each channel's
  records instead.
- [`SimulatedDevice`](@ref) correlates sample by sample in Julia, on one band
  with one antenna. It is meant for tests, not for real-time use.

## Installation

```julia
using Pkg
Pkg.add("HardwareLoopCore")
```

The core publishes into a `HardwareLoopProtocol` segment and its signals are
`GNSSSignals` objects, so a loop process usually loads both as well:

```julia
Pkg.add(["HardwareLoopProtocol", "GNSSSignals"])
```

The segment is mapped with POSIX `mmap`, so the loop process runs on Linux and
macOS. A heap-backed segment (as on the [Usage](@ref) page) works anywhere.

## Versioning

The public API is every exported symbol, listed in the
[API Reference](@ref). It follows [semantic versioning](https://semver.org):
breaking changes to these symbols bump the major version. The fields of
[`LoopCore`](@ref) and the counters on it are readable for diagnostics but are
not part of that promise, and neither are unexported names.
