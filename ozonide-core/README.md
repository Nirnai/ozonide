# ozonide-core

Shared task bodies, traits, messages, and topics for the ozonide flight controller.
Everything here is `no_std`-compatible and hardware-agnostic. Concrete hardware
drivers live in `firmware/` and `sitl/`.

---

## Data pipeline

Each stage is an async task. Tasks communicate exclusively through pub/sub topics.
Trait bounds on each task mark the injection points for hardware-specific implementations.

```
  ┌─────────────────────────────────────────────────────────────────────────────┐
  │                           ozonide-core pipeline                             │
  └─────────────────────────────────────────────────────────────────────────────┘

  ┌────────────┐
  │ IMU sensor │  (hardware: SPI/I²C in firmware, physics model in SITL)
  └─────┬──────┘
        │  impl ImuSource
        ▼
  ┌─────────────────┐   publishes    ┌──────────────────────────┐
  │    imu_task     │ ─────────────▶ │  IMU_TOPIC               │
  └─────────────────┘                │  ImuData  (accel + gyro) │
                                     └────────────┬─────────────┘
                                                  │  subscribes
                                                  ▼
  ┌──────────────────────────┐   publishes    ┌──────────────────────────┐
  │  state_estimation_task   │ ─────────────▶ │  VEHICLE_STATE_TOPIC     │
  │                          │                │  VehicleState            │
  │  impl StateEstimator     │                │  (roll, pitch, rates …)  │
  │  (ComplementaryFilter)   │                └────────────┬─────────────┘
  └──────────────────────────┘                             │  subscribes
                                                           ▼
                          ┌──────────────────────────────────────────────┐
  impl SetpointSource ──▶ │              control_task                    │ publishes
  (RC / autopilot)        │                                              │ ─────────▶  CONTROL_DEMAND_TOPIC
                          │  impl Controller                             │             ControlDemand
                          │  (CascadedPidController)                     │             (thrust + torques)
                          └──────────────────────────────────────────────┘

                                                  CONTROL_DEMAND_TOPIC
                                                           │  subscribes
                                                           ▼
  ┌──────────────────────────┐   publishes    ┌──────────────────────────┐
  │    actuator_task         │ ─────────────▶ │  ACTUATOR_TOPIC          │
  │                          │                │  ActuatorCommand         │
  │  allocate_control()      │                │  [FR, RL, FL, RR]        │
  │  impl ActuatorSink       │                └──────────────────────────┘
  │  (PWM / UDP)             │
  └──────────────────────────┘
```

> `rate_monitor` can be attached to any topic's `.count()` to log its publish
> rate in Hz. It is a utility task and not shown above.

---

## Cascaded PID controller

`CascadedPidController` implements the `Controller` trait. Internally it runs two
nested PID loops — an outer attitude loop and an inner angular velocity loop —
operating on the same `VehicleState` each cycle.

```
  AttitudeSetpoint                      VehicleState
  (roll, pitch, yaw_rate, thrust)            │
          │                                  │
          ▼                                  │
  ┌───────────────────────┐                  │
  │   AttitudeController  │ ◀────────────────┤  error = setpoint.angle − state.angle
  │   (proportional only) │                  │
  └───────────┬───────────┘                  │
              │  AngularVelocitySetpoint      │
              │  (roll_rate, pitch_rate,      │
              │   yaw_rate passthrough)       │
              ▼                              │
  ┌──────────────────────────────┐           │
  │  AngularVelocityController   │ ◀─────────┘  error = setpoint.rate − state.rate
  │  (PID per axis)              │
  └──────────────┬───────────────┘
                 │  TorqueSetpoint
                 │  (roll_torque, pitch_torque, yaw_torque)
                 │
                 ▼
  ┌──────────────────────────────┐
  │     CascadedPidController    │  combines TorqueSetpoint + AttitudeSetpoint.thrust
  └──────────────┬───────────────┘
                 │  ControlDemand
                 │  (thrust, roll_torque, pitch_torque, yaw_torque)
                 ▼
           allocate_control()
```

---

## Motor mixing (X-configuration)

`allocate_control` maps virtual axes to per-motor throttle via the mixing matrix.
Motor order (top view, Z-up, nose forward): `FR=0  RL=1  FL=2  RR=3`.

```
              thrust   roll   pitch   yaw     spin
  FR [0]  [   +1,      -1,    -1,    +1  ]    CW
  RL [1]  [   +1,      +1,    +1,    +1  ]    CW
  FL [2]  [   +1,      +1,    -1,    -1  ]   CCW
  RR [3]  [   +1,      -1,    +1,    -1  ]   CCW

          nose
           ▲
      FL ●   ● FR
          \ /
           X
          / \
      RL ●   ● RR
```

Output throttle is clamped to `[0, 1]` per motor. Priority saturation (authority
limiting) is planned in `control/authority.rs` and will run before this stage.

---

## Trait injection points

| Trait             | Implemented by                     | Used in                  |
|-------------------|------------------------------------|--------------------------|
| `ImuSource`       | SPI driver / physics sim           | `imu_task`               |
| `StateEstimator`  | `ComplementaryAttitudeEstimator`   | `state_estimation_task`  |
| `SetpointSource`  | RC receiver / topic subscriber     | `control_task`           |
| `Controller`      | `CascadedPidController`            | `control_task`           |
| `ActuatorSink`    | PWM driver / UDP socket            | `actuator_task`          |
