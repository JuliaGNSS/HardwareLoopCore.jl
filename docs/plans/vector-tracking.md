# Vector tracking in the loop process

Status: Phases 1–3 implemented (HardwareLoopProtocol 2.0.0 and #11; HardwareLoopCore#15) · 2026-10-07

## Goal

The loop process closes every satellite's loops with TrackingLoops'
`VectorPLLAndDLL` (TrackingLoops ≥ 3), and it publishes the navigation solution
the estimator computes into the HardwareLoopProtocol segment. The receiver
(GNSSReceiver's `RemoteHardwareLoop`) reads that solution in place of decoding
the bits and solving the PVT itself.

## Decisions

| Question | Decision |
|---|---|
| How the solution reaches the receiver | A new loop-wide event ring for solution and per-satellite events, plus a seqlocked snapshot of the latest solution. This changes the protocol layout (HardwareLoopProtocol 2.0). |
| What the receiver does in vector mode | It uses the loop's PVT and satellite reports, and skips its own decoding and `calc_pvt`. BIT events keep flowing. |
| Pilot/data pairs | Data-only signals at first. A vector-mode arm whose driving signal the estimator does not list (any dataless pilot) is rejected. Pairs come later. |
| When the estimator is chosen | At construction (`LoopCore(...; estimator)`). A trimmed binary needs the type known at build time, and the protocol has no command to switch it. |

## What exists today

- `LoopCore` and `ChannelTable` are typed to `NCOReferencedPLLAndDLL` with the
  default filters (`src/state.jl:175`, `:261`, `:316`).
- `_emit_partial!` builds `CorrelatorOutput(correlator, samples, sample_end)`
  and `LoopRecord(signal, filtered, previous_prompt, output, blocks, fs)`
  without `code_phase`, `prn` or `sample_offset` (`src/fold.jl:37`, `:67`).
- The `step_loop(estimator, state, record, timeline, landing)` call at
  `src/fold.jl:69` already has the shape the vector estimator takes.
- `DeviceRecord` carries the device's `code_phase` (NaN when the device does
  not report it, `src/driver.jl:52`). The core also keeps an absolute code
  phase per channel at each epoch boundary (`_advance_code_phase!`,
  `src/fold.jl:180`).
- Band counters are related to the reference band by `timebase_scale` alone
  (`src/state.jl:389`), so they are assumed to share an origin.
- Every ring and snapshot in the segment is per channel. There is no
  loop → receiver area for loop-wide data.

## What TrackingLoops 3 requires

- One shared `VectorPLLAndDLL(signals...; inner, config, cycle_time, ...)`
  instance for all satellites. It holds a mutable `VectorNavigation`, and each
  satellite's state is the immutable `SatVectorPLLAndDLL`.
- `inner = NCOReferencedPLLAndDLL()` keeps the delay-aware loop the core runs
  today.
- Every record carries:
  - `prn` (non-zero);
  - `code_phase` at `sample_index` (NaN only when the record ends on a code
    block boundary);
  - a `sample_index / fs` on a time grid shared by all satellites.
- Every satellite is stepped at least every `cycle_time / 2`. A satellite
  without records for two cycles is dropped.
- The navigation cycle (scalar PVT or UKF iteration) runs inside the
  `step_loop` call that brings the last satellite past the epoch.
- Readers: `navigation_solution`, `navigation_status`, `navigation_cycle`,
  `navigation_epoch` and `satellite_report`. All of them return reused objects.
  The scalar estimators return `nothing`.

## Phase 1: HardwareLoopProtocol 2.0 (`feat!`)

**Layout**
- Add a loop-wide block after the command ring:
  - a nav event ring (SPSC, 192 B slots, default capacity 1024);
  - one seqlocked snapshot slot holding the latest `NavSolutionEvent`.
- Add `nav_ring_offset`, `nav_capacity` and `nav_snapshot_offset` to the header.
- Add `navigation_mode::UInt8` to the header: `NAV_NONE` or `NAV_VECTOR`. The loop writes it at creation, so the receiver knows which
  path to take before the first event.
- Add every new struct to `layout_hash` and `_check_geometry`. Bump
  `PROTOCOL_VERSION` to 2, so that old and new builds refuse each other.

**Events.** The tags use `channel = 0` (loop-wide) and `device_sample` is the
cycle epoch on the reference counter. Every payload is isbits and at most
168 B.

- `EVENT_NAV_SOLUTION = 6` → `NavSolutionEvent` (about 152 B):
  - `cycle::Int64`;
  - `position_ecef_m::NTuple{3,Float64}` and `velocity_ecef_mps::NTuple{3,Float64}`;
  - `clock_bias_m` and `clock_drift`;
  - `time_tai_s::Int64` and `time_tai_frac::Float64` (with a valid flag);
  - `dop::NTuple{5,Float32}` (G/P/V/H/T, NaN when absent);
  - `position_std_m`, `clock_std_m` and `time_with_insufficient_meas_s`;
  - `num_members::Int32` and `num_sats::Int32`;
  - `flags::UInt32`: `RUNNING`, `SEEDED`, `FELL_BACK`, `RELEASED`, `VALID`.
- `EVENT_NAV_SATELLITE = 7` → `NavSatelliteEvent`, one per satellite the cycle
  knows. `tag.prn` and `tag.band` are filled.
  - `cycle::Int64` and `channel::UInt16` (the loop channel driving the
    satellite);
  - `signal::FixedName` (the bank's signal name);
  - `sat_position_ecef_m::NTuple{3,Float64}` and `sat_time_s`;
  - `residual_m`, `rate_residual_mps` and `cn0_dbhz`;
  - `flags`: `TRACKED`, `BIT_SYNCED`, `IN_LOCK`, `PVT_READY`, `IN_VECTOR_LOOP`
    and `IN_SOLUTION`;
  - `release_reason::UInt8` (mirrors `VTReleaseReason`).
- Ordering: a cycle's satellite events come first and its solution event
  last. A reader treats the solution as the commit marker for that cycle.
- Inter-system and inter-frequency biases are not published. `clock_bias_m`
  is the receiver clock bias of the solution's reference system.

**API.** `nav_ring(segment)`, `publish_nav_solution!` (which writes the ring and
the snapshot) and `read_nav_snapshot`, with the same seqlock idiom as the
channel snapshots.

## Phase 2: HardwareLoopCore, generic estimator

1. Make `LoopCore` and `ChannelTable` take the estimator type as a parameter:
   - `LoopCore{D,Banks,E}`;
   - `ChannelTable` typed through `estimator_state_type(E, ...)`;
   - the keyword relaxed to any estimator that implements `step_loop` with a
     timeline and a landing.

   The scalar path must still allocate nothing (`test/service.jl`) and still
   build with trim.
2. Make the records name their satellite. `_emit_partial!` passes:
   - `prn = T.prn[ch]`;
   - `code_phase`: the last device record's `code_phase` when it is reported,
     otherwise the core's own code phase advanced to `sample_end`;
   - `sample_offset`: 0 while band counters share an origin, as
     `_to_reference` assumes, behind a helper so that a per-band offset can be
     added without touching the fold.

   The scalar estimators ignore these fields, so this step changes nothing for
   them and can be released on its own (`feat`).
3. Add vector-mode arm rules:
   - In `_handle_arm!`, a `signal_index == 1` arm for a signal the estimator
     does not list is rejected with `REJECT_UNSUPPORTED_SIGNAL`. That covers
     every dataless pilot.
   - Passengers (`signal_index ≥ 2`) are rejected in vector mode until pairs
     are designed.
   - At construction, check that every bank whose signal can drive is listed
     by the estimator, and that `inner` is `NCOReferencedPLLAndDLL`.
4. Keep the engine fed:
   - Observation-only epochs (stale backlog) skip `step_loop` today. Fold them
     through `step_loop` in vector mode as well; otherwise the engine starves
     and falls back after `max_backlog_epochs`.
   - Alternatively, cap the backlog below `cycle_time / 2` and document the
     cap. This is decided during implementation, with a test either way.
5. Re-arm: an in-place re-arm to another PRN needs nothing extra, because the
   engine drops the old PRN after two cycles. Add a test that the slot is
   reused and that the old PRN never appears as a member.

## Phase 3: HardwareLoopCore publishes the solution (`feat!`, protocol 2)

1. Construction writes `navigation_mode` into the header:
   - `NAV_VECTOR` for `VectorPLLAndDLL` with a config;
   - `NAV_NONE` for the scalar loops, which publish no solution: the nav ring
     stays empty and the snapshot invalid.

   A `VectorPLLAndDLL` with `config = nothing` (scalar PVT only) is rejected at
   construction. A loop running scalar loops never computes a PVT. In vector
   mode, the solutions published before the filter is seeded are the engine's
   own scalar fixes, flagged without `SEEDED`.
2. At the end of `fold_closed_epochs!`, if
   `navigation_cycle(core.estimator) > core.last_nav_cycle`, then:
   - for each armed driving channel, read
     `satellite_report(est, signal, prn)` and its `pvt.sats` entry, and
     publish `NavSatelliteEvent`;
   - then publish `NavSolutionEvent` and the snapshot.

   All of this runs without allocation: no iteration that allocates over the
   `Dictionary`, and lookups by `(system, prn)` only.
3. Add counters `nav_events_published` and `max_nav_cycle_ns`. The time of the
   pass that runs the cycle is measured separately, because the UKF and PVT
   land on one record's `step_loop`. See the risks.
4. README limitations: remove "one estimator", and add "vector tracking:
   data signals only, one shared engine, the cycle runs inside the pass".

## Phase 4: GNSSReceiver (`RemoteHardwareLoop`)

1. Remove the `vector_tracking` throw (`src/remote_hardware_loop.jl:721`).
   Vector mode is now a property of the loop process, read from
   `navigation_mode`, not a receiver option.
2. When `navigation_mode != NAV_NONE`:
   - drain the nav ring;
   - build the receiver's PVT output from `NavSolutionEvent` plus that cycle's
     `NavSatelliteEvent`s;
   - skip decoding and `calc_pvt`;
   - take C/N₀ and lock per satellite from the reports;
   - on `:lost`, fall back to `read_nav_snapshot`.
3. When `navigation_mode == NAV_NONE` (scalar loops), nothing changes: the
   receiver decodes the mirrored bits and solves the PVT itself with
   PositionVelocityTime's `calc_pvt`, the same solver TrackingLoops' engine
   uses for its scalar fix.
4. BIT events keep being mirrored for data output and logging.
5. Arming policy stays on the receiver, and it must not arm pilots as drivers
   in vector mode.

## Tests

- **Unit:**
  - records carry `prn` and `code_phase` (device-reported and derived);
  - vector-mode arm rejections;
  - the type-parameterised core with the scalar estimator is unchanged.
- **Protocol:**
  - the new event layouts and the hash;
  - ring and snapshot round-trips;
  - a v1 segment is refused.
- **Closed loop on recorded samples (vector):** feed the ION RTL-SDR
  recording through `SimulatedDevice`, which already correlates any raw
  samples it is handed (`correlate_chunk!`) and reports each record's code
  phase. No synthetic geometry or LNAV encoder is needed.
  - Data: `https://sdr.ion.org/RTL_SDR/RTLSDR_Bands-L1.uint8`, GPS L1 C/A
    only (which matches the data-only scope), 60 s, 2.048 MS/s, 8-bit
    offset-binary I/Q at zero IF, about 246 MB. GNSSReceiver's
    `test/ion_rtlsdr_integration.jl` already uses it, and its `_ion_produce!`
    shows the sample conversion.
  - It runs on every PR as part of the normal suite, with no gate, as in
    GNSSReceiver. The file is downloaded once with `curl` into a Scratch.jl
    space (`@get_scratch!("rtl_sdr_test_data")`), and `julia-actions/cache`
    keeps it between CI runs.
  - The recording carries one signal, GPS L1 C/A, so the test covers exactly
    that. Pilot/data pairs, other systems and other bands get no
    recorded-data test until a recording with them is chosen.
  - Arm the channels from an acquisition of the first milliseconds
    (Acquisition.jl as a test dependency), with the noise reference on a
    spare channel. Build `VectorPLLAndDLL(GPSL1(); inner =
    NCOReferencedPLLAndDLL(), approximate_year = 2017)`.
  - Assert, through the segment's nav ring and snapshot only:
    - the healthy PRNs GNSSReceiver's test expects are tracked;
    - the filter seeds, and at least four members are in the vector loop by
      the end;
    - the final position is within 10 m of GNSSReceiver's regression fix,
      ECEF `[3.9074087926e6, 3.0683836901e5, 5.0149608655e6]`, and the mean
      horizontal error under 5 m. That fix is a pipeline baseline, not a
      surveyed point;
    - the common-mode code residual stays under 5 m, with the device at the
      front end's IF (see below);
    - no word lands late, with a record delay of 0 and 2 epochs.
  - Cost: `SimulatedDevice` correlates sample by sample, about 3×10⁹ tap
    operations for 60 s and 8 channels. Because this runs on every PR,
    measure it first. If it takes more than a few minutes:
    - stop at the shortest span that still seeds the filter (about 35 s to
      the first fix in GNSSReceiver's benchmark);
    - give `correlate_chunk!` a vectorised path that correlates a whole
      chunk per channel.
- **Closed loop, synthetic (scalar):** the existing `SimulatedDevice` tests
  stay as they are and keep covering the scalar path offline.
- **Allocation:** a warm pass that includes a navigation cycle and the nav
  publish allocates nothing.
- **Trim:** the core with `VectorPLLAndDLL` builds with `juliac --trim=safe`.

## The front end's carrier offset (found while testing)

On the ION recording the vector filter first settled about 12 m too high. The
cause is the RTL-SDR, not the vector path. Its tuned frequency is about 83 Hz
off, and the code does not share that offset:
- scalar loops hold `code − carrier/1540 ≈ +0.054 Hz` on every satellite;
- a ppm error of the shared crystal would leave this at 0.

One clock-drift state cannot satisfy both the carrier rates and the code. The
conflict leaves a −30 m common-mode code residual, part of which leaks into
the height (JuliaGNSS/TrackingLoops.jl#35).

The offset is deterministic from the SDR's settings, so it is the driver's to
report, not the core's to estimate. HardwareLoopProtocol 3.0 carries each
band's `intermediate_frequency_hz` in the band table, and drivers run their
carrier NCOs at IF + Doppler; `SimulatedDevice` takes it as a keyword. The
recorded-data test runs at IF = −83 Hz and holds the final position within
10 m of the baseline. The rest of its error (about +6 m up) is the
uncorrected ionosphere: no Klobuchar coefficients are in 60 s of data.

GNSSReceiver looked unaffected because its filter assumes 100 ms cycles that
last 104 ms (4 ms chunks). That scales its velocity and clock drift by 1.04
and happened to cancel half of the offset.

## Releases

| Package | Change | Type |
|---|---|---|
| HardwareLoopCore | allow TrackingLoops 2 and 3 | `fix(deps)` (#13, released in 1.0.2) |
| HardwareLoopProtocol 2.0 | loop-wide nav ring, snapshot, header fields | `feat!` (released) |
| HardwareLoopProtocol 3.0 | each band's intermediate frequency in the band table | `feat!` (#11, merged) |
| HardwareLoopCore 2.0 | generic estimator, vector mode, nav publishing, band IF; protocol 3, TrackingLoops 3 | `feat!` (#15, one release) |
| GNSSReceiver | consumes the loop's solution | `feat` |

## Risks

- **Latency of the cycle pass.** The scalar PVT and the UKF run inside one
  `step_loop`. If that exceeds the commit lead, words land late (`words_late`).
  Measure first. If needed, run the cycle at the end of the pass after the
  words are committed. That would be a TrackingLoops API change to defer the
  cycle.
- **A shared, non-thread-safe engine.** This is fine while the core is single
  threaded, and it rules out a threaded fold later.
- **Load time.** TrackingLoops 3 adds about 1 s to `using`. This does not
  matter for a long-running process, but it does for tests.
- **Band time origins.** If a device's band counters do not share an origin,
  both the vector grid and `_to_reference` are wrong. Ask the driver for a
  per-band offset at that point.

## Settled

- Inter-system and inter-frequency biases are not published (2026-10-06).
- Scalar loops publish no PVT; the receiver solves it from the bits
  (2026-10-06).
- The vector closed-loop test runs on a downloaded recording, not on
  synthetic satellites (2026-10-06).
- That test runs on every PR, and on GPS L1 C/A only, the one signal in the
  recording (2026-10-06).

## Open questions

None at the moment.
