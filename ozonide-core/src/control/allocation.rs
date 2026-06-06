use nalgebra::{Vector4, Matrix4};


/// Maps normalised virtual commands to per-motor throttle for an X-configuration quadrotor.
///
/// # Inputs
///
/// | Parameter | Range     | Meaning |
/// |-----------|-----------|---------|
/// | `thrust`  | `[0, 1]`  | Collective throttle. `0` = motors off, `1` = full power. |
/// | `roll`    | `[-1, 1]` | Roll correction from rate PID. Positive = roll right. |
/// | `pitch`   | `[-1, 1]` | Pitch correction from rate PID. Positive = pitch forward. |
/// | `yaw`     | `[-1, 1]` | Yaw correction from rate PID. Positive = rotate CCW from above. |
///
/// # Mixing matrix
///
/// Each motor's throttle is the sum of the collective thrust plus signed contributions
/// from each virtual axis. Signs are derived from the motor's moment arm and spin
/// direction (CW motors produce positive yaw torque, CCW motors negative):
///
/// ```text
///            thrust   roll   pitch   yaw
/// FR [0]:  [  +1,     -1,    -1,    +1  ]   (CW,  front-right)
/// RL [1]:  [  +1,     +1,    +1,    +1  ]   (CW,  rear-left)
/// FL [2]:  [  +1,     +1,    -1,    -1  ]   (CCW, front-left)
/// RR [3]:  [  +1,     -1,    +1,    -1  ]   (CCW, rear-right)
/// ```
///
/// # Output
///
/// Per-motor throttle `[FR, RL, FL, RR]` clamped to `[0, 1]`. Clamping is applied
/// independently per motor; no priority scaling is performed here — see
/// `authority` for saturation handling before this function is called.
pub fn allocate_normalized_throttle_commands(thrust: f32, roll: f32, pitch: f32, yaw: f32) -> Vector4<f32> {
    const MIXING_MATRIX: Matrix4<f32> = Matrix4::new(
        1.0, -1.0, -1.0,  1.0,  // row 0: FR [thrust, roll, pitch, yaw]
        1.0,  1.0,  1.0,  1.0,  // row 1: RL
        1.0,  1.0, -1.0, -1.0,  // row 2: FL
        1.0, -1.0,  1.0, -1.0,  // row 3: RR
    );
    let input = Vector4::new(thrust, roll, pitch, yaw);
    (MIXING_MATRIX * input).map(|x| x.clamp(0.0, 1.0))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_motors_approx(result: Vector4<f32>, expected: [f32; 4]) {
        for i in 0..4 {
            assert!(
                (result[i] - expected[i]).abs() < 1e-5,
                "motor[{i}]: got {}, expected {}", result[i], expected[i]
            );
        }
    }

    #[test]
    fn pure_thrust_sets_all_motors_equally() {
        let result = allocate_normalized_throttle_commands(0.5, 0.0, 0.0, 0.0);
        assert_motors_approx(result, [0.5, 0.5, 0.5, 0.5]);
    }

    #[test]
    fn roll_right_increases_left_motors_decreases_right() {
        // Positive roll = roll right → left motors (RL, FL) faster, right motors (FR, RR) slower.
        let result = allocate_normalized_throttle_commands(0.5, 0.1, 0.0, 0.0);
        assert_motors_approx(result, [0.4, 0.6, 0.6, 0.4]);
    }

    #[test]
    fn pitch_forward_increases_rear_motors_decreases_front() {
        // Positive pitch = pitch forward → rear motors (RL, RR) faster, front motors (FR, FL) slower.
        let result = allocate_normalized_throttle_commands(0.5, 0.0, 0.1, 0.0);
        assert_motors_approx(result, [0.4, 0.6, 0.4, 0.6]);
    }

    #[test]
    fn yaw_ccw_increases_cw_motors_decreases_ccw() {
        // Positive yaw = CCW from above → CW motors (FR, RL) faster, CCW motors (FL, RR) slower.
        let result = allocate_normalized_throttle_commands(0.5, 0.0, 0.0, 0.1);
        assert_motors_approx(result, [0.6, 0.6, 0.4, 0.4]);
    }

    #[test]
    fn zero_inputs_produce_zero_throttle() {
        let result = allocate_normalized_throttle_commands(0.0, 0.0, 0.0, 0.0);
        assert_motors_approx(result, [0.0, 0.0, 0.0, 0.0]);
    }

    #[test]
    fn saturated_output_is_clamped_to_unit_range() {
        // Large roll correction would push FR/RR below 0 and RL/FL above 1.
        let result = allocate_normalized_throttle_commands(0.5, 0.6, 0.0, 0.0);
        assert_motors_approx(result, [0.0, 1.0, 1.0, 0.0]);
    }
}
