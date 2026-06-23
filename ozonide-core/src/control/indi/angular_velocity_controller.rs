use nalgebra::{Vector3, Vector4};

use crate::msgs::{ActuatorCommand, AngularAccelerationSetpoint, AngularVelocitySetpoint, VehicleState};
use crate::traits::Controller;

use super::incremental_inversion::IncrementalInversion;
use super::input_signal_conditioning::InputSignalConditioning;
use super::inverse_actuator_model::InverseActuatorModel;

pub struct AngularVelocityController {
    conditioning: InputSignalConditioning,
    /// INDI law over the 4-D virtual control [αx, αy, αz, specific thrust].
    inversion: IncrementalInversion<4>,
    /// One model per motor — each is identified independently on the bench.
    output_map: [InverseActuatorModel; 4],
    /// Proportional rate gains [roll, pitch, yaw], units: s⁻¹.
    /// Sets the closed-loop bandwidth of the rate-tracking loop: Kp = 1/τ.
    rate_gain: Vector3<f32>,
}

impl AngularVelocityController {
    pub fn new(
        conditioning: InputSignalConditioning,
        inversion: IncrementalInversion<4>,
        output_map: [InverseActuatorModel; 4],
        rate_gain: Vector3<f32>,
    ) -> Self {
        Self { conditioning, inversion, output_map, rate_gain }
    }
}

impl Controller for AngularVelocityController {
    type Setpoint = AngularVelocitySetpoint;

    fn step(&mut self, state: &VehicleState, setpoint: &AngularVelocitySetpoint) -> ActuatorCommand {
        let cond = self.conditioning.step(state);

        // Rate-P: translate rate error into an angular acceleration setpoint.
        let angular_rate_error = Vector3::new(
            setpoint.roll_rate  - cond.angular_rate_filtered[0],
            setpoint.pitch_rate - cond.angular_rate_filtered[1],
            setpoint.yaw_rate   - cond.angular_rate_filtered[2],
        );
        let acceleration_setpoint = AngularAccelerationSetpoint {
            timestamp_us: setpoint.timestamp_us,
            roll_acceleration:  self.rate_gain[0] * angular_rate_error[0],
            pitch_acceleration: self.rate_gain[1] * angular_rate_error[1],
            yaw_acceleration:   self.rate_gain[2] * angular_rate_error[2],
            specific_thrust:    setpoint.specific_thrust,
        };

        // Pack the desired and measured virtual control, then apply the INDI law:
        // Δu = G⁻¹ · (ν_sp − ν̂), u = u₀ + Δu. Virtual control is
        // [αx, αy, αz, specific thrust].
        let virtual_desired = Vector4::new(
            acceleration_setpoint.roll_acceleration,
            acceleration_setpoint.pitch_acceleration,
            acceleration_setpoint.yaw_acceleration,
            acceleration_setpoint.specific_thrust,
        );
        let virtual_measured = Vector4::new(
            cond.angular_acceleration[0],
            cond.angular_acceleration[1],
            cond.angular_acceleration[2],
            cond.specific_thrust,
        );
        let u_cmd = self.inversion.compute(
            &virtual_desired,
            &virtual_measured,
            &cond.actuator_state,
        );

        // Keep lag model current for the next cycle.
        self.conditioning.feed_command(&u_cmd);

        let v_bat = state.battery_voltage();
        ActuatorCommand {
            motor_throttle: core::array::from_fn(|i| self.output_map[i].throttle(u_cmd[i], v_bat)),
        }
    }

    fn reset(&mut self, state: &VehicleState) {
        self.conditioning.reset(state);
    }
}
