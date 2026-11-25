# Glossary

The core's own terms. It shares the tracking terms with TrackingLoops, whose
[glossary](https://JuliaGNSS.github.io/TrackingLoops.jl/dev/glossary/) ties them to
the textbooks (Kaplan & Hegarty, Misra & Enge, Spilker et al.), and the wire terms
with HardwareLoopProtocol, whose
[glossary](https://JuliaGNSS.github.io/HardwareLoopProtocol.jl/dev/glossary/) covers
the records the core publishes.

## Device and driver

- **Loop process, receiver**: the loop process runs the core next to the device and
  closes the tracking loops; the receiver arms channels, decodes and displays. They
  share one HardwareLoopProtocol segment.
- **Driver**: the [`AbstractLoopDriver`](@ref) a device implements, the core's only
  view of it.
- **Channel**: a hardware channel, one signal of one satellite on one band. A
  *driver channel* closes its satellite's loops; a *passenger channel* (the data
  component of a pilot/data pair) follows its driver's NCO commands; a *noise
  reference* (`signal_index = 0`) despreads noise for the C/N₀.
- **Arm, confirm**: an arm programs a channel; it is *confirmed* once the driver
  reports the sample it took effect at ([`assignment_start`](@ref)). Records before
  that are *stale*.
- **Record**: what the driver hands the core ([`DeviceRecord`](@ref)): one channel's
  *dump*, its correlator outputs integrated over one primary code period or part of
  one, or a *timebase strobe*. A *lost record* is a gap in a channel's dumps.
- **Band, reference band**: an RF band of the band table, each with its own sample
  counter; the first band is the reference band, onto whose counter the core scales
  every other band's sample indices.
- **Timebase**: the core's reading of the device sample counter on the reference
  band, moved by every record's sample index. A *timebase strobe* moves it while no
  channel is armed.

## The loop

- **Integration** (coherent integration): a channel's dumps summed until it is
  handed to TrackingLoops, Kaplan and Hegarty's predetection integration. The
  *open integration* is the one still being summed.
- **NCO update span, interval and boundary**: the core updates every channel's
  NCO command once per *NCO update span*, the samples between two *NCO update
  boundaries* on a common grid of the reference band's counter; the *NCO update
  interval* is a span's length (`nco_update_interval_samples`), as in Tracking.jl.
  The core *closes* a span once every record before its boundary is in: it hands
  the completed integrations to the tracking loop, advances the code phases to the
  boundary, publishes each channel's loop state and schedules the NCO commands.
- **Service pass**: one turn of the loop process ([`service_pass!`](@ref)): read the
  records, close the NCO update spans that are complete, commit the NCO commands,
  handle the receiver's commands, confirm arms, re-arm the noise references.
- **NCO command, application sample index**: the carrier and code Doppler a channel
  runs at, and the sample it takes effect at. The core predicts that sample as the
  counter at the start of the pass plus the *NCO command delay*, writes the command
  in the same pass, and *confirms* it at the sample the device reports, or *cancels*
  it if the device refuses. A command that takes effect more than an NCO update
  interval after its prediction is *late*.
- **Stale backlog, open loop**: a span more than `max_backlog_nco_updates` NCO
  updates behind the device is closed with the *loop open*: its integrations update
  the bits and the C/N₀ but do not step the loop filters.
- **Absolute code phase**: the code phase the core keeps for each driver channel
  at the latest NCO update boundary, counted from the last bit edge (or secondary
  code start) once the bit edge is known, and pinned to the device's reported code
  phase where the device gives one.
- **Accumulated carrier phase**: the replica's carrier phase in cycles, the
  carrier Doppler accumulated over a channel's NCO commands since the arm,
  unfolded, plus TrackingLoops' `calc_carrier_phase_correction` while the vector
  loop steers the channel: the carrier-phase measurement (RINEX `L`). Not
  Tracking.jl's replica carrier phase, which is folded to `[-0.5, 0.5)`.
- **Noise probe**: the PRN, code phase and carrier a noise reference replicates,
  drawn by TrackingLoops; the core re-arms the noise reference onto a new probe every
  `noise_rearm_nco_updates` NCO updates.
- **Navigation update, measurement epoch**: in vector tracking, the navigation
  processor's update at a measurement epoch (TrackingLoops' terms); the core
  publishes each one on the nav ring.

## Units

- **Units in names**: a public field or keyword with a unit ends in it, as on the
  wire: `_hz`, `_s`, `_samples`, `_chips`, `_ns`, `_us`. A sample index or a count
  carries none. Values the core takes from or hands to TrackingLoops carry Unitful
  units instead, as TrackingLoops' API does.
- **C/N₀**: in dB-Hz, `-Inf` for no estimate, as TrackingLoops gives it and the
  protocol carries it.
