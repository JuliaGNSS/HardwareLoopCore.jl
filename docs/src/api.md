# API Reference

## Driver API

```@docs
AbstractLoopDriver
DriverCapabilities
driver_capabilities
read_records!
write_word!
arm!
release!
assignment_start
sample_count
wait_records
overflowed_channels!
```

## Arming

```@docs
ArmSpec
ArmOutcome
ARM_ACCEPTED
arm_rejected
```

## Records

```@docs
DeviceRecord
MAX_RECORD_TAPS
pack_taps
strobe_record
is_strobe
```

## Core and service pass

```@docs
LoopCore
service_pass!
run!
take_records!
fold_closed_epochs!
commit_words!
handle_commands!
confirm_arms!
rearm_noise_references!
LATENCY_EDGES_US
```

## Configuration

```@docs
LoopConfig
```

## Simulated device

```@docs
SimulatedDevice
correlate_chunk!
```
