# Changelog

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
