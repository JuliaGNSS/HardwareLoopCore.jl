# Writing a driver

A device plugs into the core as a subtype of [`AbstractLoopDriver`](@ref). The
core calls the driver directly; it is a type parameter of
[`LoopCore`](@ref), so every call resolves at compile time and a trimmed
binary carries no dynamic dispatch. [`SimulatedDevice`](@ref) is a complete
driver and a good template.

## The contract

A driver implements these methods:

| Method | Called | Does |
|---|---|---|
| [`driver_capabilities`](@ref)`(d)` | once, when the core is built | the channel count, the widest tap layout, the antennas and the band table |
| [`read_records!`](@ref)`(d, records)` | every pass | appends every new [`DeviceRecord`](@ref) and returns how many; never blocks |
| [`write_word!`](@ref)`(d, ch, carrier_hz, code_hz)` | after a fold that stepped `ch` | commits the NCO word, effective on the next sample |
| [`arm!`](@ref)`(d, ch, spec)` | on an arm command, and on a noise-reference re-arm | programs the replica in the [`ArmSpec`](@ref); returns [`ARM_ACCEPTED`](@ref) or [`arm_rejected`](@ref)`(reason)` |
| [`release!`](@ref)`(d, ch)` | on a release command, or a failed arm | stops the channel |
| [`assignment_start`](@ref)`(d, ch)` | every pass while an arm is unconfirmed, and per record | the sample the current assignment took effect at |
| [`sample_count`](@ref)`(d, band)` | several times per pass | the device's sample counter on `band` now |

and optionally:

| Method | Default |
|---|---|
| [`wait_records`](@ref)`(d, timeout_ms)` | returns at once (the service loop polls) |
| [`overflowed_channels!`](@ref)`(d)` | `0` (not read by the core yet) |

## Records

[`read_records!`](@ref) hands the core two kinds of [`DeviceRecord`](@ref):

- A **correlator dump** of one channel: the PRN the channel ran, the sample
  index at the *end* of the integration, the number of samples integrated, and
  the raw accumulator sums, latest tap first and antenna-major (pack them with
  [`pack_taps`](@ref)). If the device knows the replica's code phase at the end
  of the integration, report it in chips; it anchors the absolute code phase
  the receiver forms pseudoranges from. Otherwise leave it `NaN`.
- An **epoch strobe** ([`strobe_record`](@ref)): a marker on the reference
  band's counter. The core folds an epoch once a record past its boundary has
  arrived; the strobes keep the epoch clock moving when no channel is armed.

The records must tile each channel's sample axis: a dump starts where the
previous one ended. The core infers a lost record from a gap, restarts the
navigation bit clock, and counts it. A dump normally ends on a primary-code
boundary; a device may also cut dumps inside a code period, and the core
accumulates them up to the boundary.

The tap layout of a dump must match the layout the core's correlator expects
for the signal (three taps for the default early–prompt–late correlator);
dumps with another tap count are counted in `tap_layout_mismatches` and
dropped.

## Timing

The sample index of a record, the `valid_at_sample` of an [`ArmSpec`](@ref),
[`assignment_start`](@ref) and [`sample_count`](@ref) all count the same
free-running device samples on the channel's band. Band 1 is the reference:
its counter is the receiver timebase, and other bands are scaled onto it by
their sampling frequencies.

An arm describes the satellite at `valid_at_sample`, which may lie in the
past: the device propagates the code phase to the sample it actually starts
at, and reports that sample through [`assignment_start`](@ref). Until then it
returns `typemax(Int64)`, and the core drops the channel's records as stale. If
the device cannot start the assignment after all, it returns `typemin(Int64)`;
the core rejects the arm towards the receiver and releases the channel.

A word written with [`write_word!`](@ref) applies from the device's next
sample. The core reads [`sample_count`](@ref) right after the write and
corrects the channel's NCO timeline to where the word really landed, so a
record that straddles a word change is credited with the weighted mean of both
words. Records may arrive late — the delay-aware estimator accounts for the
words committed since — but the shorter the record-to-word latency, the
tighter the loop.

## Amplitudes

The [`ArmSpec`](@ref) carries the replica and code amplitudes the receiver
declared. A device whose accumulators carry a gain (e.g. a ±127 carrier
table) reports the raw sums; the core divides them by
`replica_amplitude × code_amplitude / code amplitude of the signal`, the same
for satellites and the noise reference, so the C/N₀ is independent of the
device's scale.
