// use ozonide_core::{
//     control::{
//         allocate_normalized_throttle_commands, AnglePidController, AnglePidGains,
//         PidGains, RatePidController, RatePidGains,
//     },
//     estimation::ComplementaryAttitudeEstimator,
//     msgs::{ActuatorCommand, AttitudeSetpoint},
//     topics::{ACTUATOR_TOPIC, IMU_TOPIC},
// };

// /// Hover setpoint: wings level, zero yaw rate, ~50% thrust.
// ///
// /// Hover throttle is derived from the simulator's physical parameters:
// ///   omega_hover = sqrt(mass·g / (4·k_t)) ≈ 1493 rad/s
// ///   throttle    = omega_hover / omega_max ≈ 0.498
// /// Adjust `thrust` if the drone climbs or descends at rest.
// const HOVER_SETPOINT: AttitudeSetpoint = AttitudeSetpoint {
//     roll: 0.0,
//     pitch: 0.0,
//     yaw_rate: 0.0,
//     thrust: 0.5,
// };

// /// Outer-loop (angle) PID gains.
// ///
// /// Output is a rate setpoint in rad/s, capped at ±3 rad/s (~170°/s).
// /// Start with kp only; add ki if there is steady-state angle offset.
// fn angle_gains() -> AnglePidGains {
//     let axis = PidGains {
//         kp: 4.0,
//         ki: 0.0,
//         kd: 0.0,
//         integral_limit: 0.0,
//         output_limit: 3.0,
//     };
//     AnglePidGains { roll: axis, pitch: axis }
// }

// /// Inner-loop (rate) PID gains.
// ///
// /// Output is a normalised torque in [-1, 1] fed to the mixer.
// /// Yaw uses lower gains — higher inertia, weaker actuator authority.
// fn rate_gains() -> RatePidGains {
//     let roll_pitch = PidGains {
//         kp: 0.15,
//         ki: 0.05,
//         kd: 0.05,
//         integral_limit: 0.3,
//         output_limit: 0.5,
//     };
//     let yaw = PidGains {
//         kp: 0.10,
//         ki: 0.02,
//         kd: 0.00,
//         integral_limit: 0.3,
//         output_limit: 0.5,
//     };
//     RatePidGains { roll: roll_pitch, pitch: roll_pitch, yaw }
// }

// /// Full attitude control cascade running at IMU rate (~1 kHz).
// ///
// /// Data flow:
// /// ```text
// /// IMU_TOPIC → ComplementaryAttitudeEstimator → AnglePid → RatePid → mixer → ACTUATOR_TOPIC
// /// ```
// #[embassy_executor::task]
// pub async fn flight_controller_task() {
//     let mut filter = ComplementaryAttitudeEstimator::new(0.98);
//     let mut angle_pid = AnglePidController::default();
//     let mut rate_pid = RatePidController::default();

//     let angle_gains = angle_gains();
//     let rate_gains = rate_gains();

//     let actuator_pub = ACTUATOR_TOPIC.publisher();
//     let mut imu_sub = IMU_TOPIC.subscriber();
//     let mut prev_timestamp_us: Option<u64> = None;

//     loop {
//         let imu = imu_sub.changed().await;
//         let estimate = filter.update(&imu);

//         // Skip the first sample — filter and dt tracking both need a prior timestamp.
//         let Some(prev_us) = prev_timestamp_us else {
//             prev_timestamp_us = Some(imu.timestamp_us);
//             continue;
//         };
//         let dt = (imu.timestamp_us - prev_us) as f32 * 1e-6;
//         prev_timestamp_us = Some(imu.timestamp_us);

//         let rate_sp = angle_pid.update(&HOVER_SETPOINT, &estimate, &angle_gains, dt);
//         let rate_corr = rate_pid.update(
//             rate_sp.roll_rate,
//             rate_sp.pitch_rate,
//             rate_sp.yaw_rate,
//             &estimate,
//             &rate_gains,
//             dt,
//         );

//         let throttles = allocate_normalized_throttle_commands(
//             HOVER_SETPOINT.thrust,
//             rate_corr.roll,
//             rate_corr.pitch,
//             rate_corr.yaw,
//         );

//         actuator_pub.publish(ActuatorCommand {
//             motor_throttle: [throttles[0], throttles[1], throttles[2], throttles[3]],
//         });
//     }
// }
