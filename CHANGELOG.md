# Changelog

# [2.0.0](https://github.com/JuliaGNSS/HardwareLoopCore.jl/compare/v1.2.0...v2.0.0) (2026-10-19)


* feat!: run vector tracking and publish its solution on protocol 2 ([e97f877](https://github.com/JuliaGNSS/HardwareLoopCore.jl/commit/e97f8774f3aa0e8fc5b7568111201a0ef0efd661))
* feat!: take each band's intermediate frequency from the band table ([76f6772](https://github.com/JuliaGNSS/HardwareLoopCore.jl/commit/76f677202f9f63844714bf3c2c9b2a5b8a206a1b)), closes [JuliaGNSS/TrackingLoops.jl#35](https://github.com/JuliaGNSS/TrackingLoops.jl/issues/35)


### Features

* range on a pilot paired with its data component in vector mode ([b31f48a](https://github.com/JuliaGNSS/HardwareLoopCore.jl/commit/b31f48a6b15019e8f84a025b1c88205272d846ee)), closes [#17](https://github.com/JuliaGNSS/HardwareLoopCore.jl/issues/17)


### BREAKING CHANGES

* requires HardwareLoopProtocol 3 (BandEntry gained the
IF; 2.x receivers refuse the segment). LoopCore throws an ArgumentError
when the segment's band table is not the driver's; create the segment
with `bands = driver_capabilities(driver).bands`. A driver for a front
end with a non-zero IF must report it in its BandEntry and apply it to
its carrier NCOs.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* requires HardwareLoopProtocol 2 (the nav ring layout;
1.x receivers refuse the segment). LoopCore has two
more type parameters (the estimator and the channel table), so code that
spells LoopCore{D,Banks} must use LoopCore{D,Banks,E,CT} or drop the
parameters. Rebuild the receiver against HardwareLoopProtocol 2.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>

# [1.1.0](https://github.com/JuliaGNSS/HardwareLoopCore.jl/compare/v1.0.2...v1.1.0) (2026-10-10)


### Features

* step every channel of a satellite through its estimator ([4271117](https://github.com/JuliaGNSS/HardwareLoopCore.jl/commit/42711174ef60f63d09002b808f5c5af9333cea0a)), closes [JuliaGNSS/TrackingLoops.jl#36](https://github.com/JuliaGNSS/TrackingLoops.jl/issues/36)

## [1.0.2](https://github.com/JuliaGNSS/HardwareLoopCore.jl/compare/v1.0.1...v1.0.2) (2026-10-06)


### Bug Fixes

* **commands:** answer a command for a channel the loop lacks on channel 1 ([327137f](https://github.com/JuliaGNSS/HardwareLoopCore.jl/commit/327137fc45c6dfc7ff86690ceeff2b4ea4102a86))
* **core:** say how many antennas the driver and the core have ([038284d](https://github.com/JuliaGNSS/HardwareLoopCore.jl/commit/038284dffd43d749bfcef5e7a0298bda4a6630c9))
* **deps:** allow TrackingLoops 2 and 3 ([#13](https://github.com/JuliaGNSS/HardwareLoopCore.jl/issues/13)) ([96e3469](https://github.com/JuliaGNSS/HardwareLoopCore.jl/commit/96e3469b18aa006de2c16bd49eeb5c8efcc650f4))
* **deps:** take the loop packages from the registry ([be7b4c6](https://github.com/JuliaGNSS/HardwareLoopCore.jl/commit/be7b4c6fb466426c31cb1ee427a427cbf4c3f766))
* **fold:** read a passenger's bits in its driver's carrier frame ([47046e6](https://github.com/JuliaGNSS/HardwareLoopCore.jl/commit/47046e60f502f6a49254bfe7751619389c6f3dee))
* **noise:** give a multi-antenna loop the noise covariance it combines ([279d936](https://github.com/JuliaGNSS/HardwareLoopCore.jl/commit/279d936ea9c2f692cf261df937cb24a07a5d7bec))

## [1.0.1](https://github.com/JuliaGNSS/HardwareLoopCore.jl/compare/v1.0.0...v1.0.1) (2026-09-30)

No changes to the package. Replaces the accidental 2.0.0 release.

# 1.0.0 (2026-09-23)


### Features

* the loop process's engine — driver API, LoopCore, SimulatedDevice ([cb7a08f](https://github.com/JuliaGNSS/HardwareLoopCore.jl/commit/cb7a08ffb5f2351661c0757b6b6efaadb3c4afe7))
