use nalgebra::Vector3;

use crate::msgs::{ActuatorCommand, AngularAccelerationSetpoint, AngularVelocitySetpoint, VehicleState};
use crate::traits::Controller;

use super::allocation::ControlAllocator;
use super::input_signal_conditioning::InputSignalConditioning;
use super::inverse_actuator_model::InverseActuatorModel;

pub struct AngularVelocityController {
    conditioning: InputSignalConditioning,
    allocator: ControlAllocator,
    /// One model per motor — each is identified independently on the bench.
    output_map: [InverseActuatorModel; 4],
    /// Proportional rate gains [roll, pitch, yaw], units: s⁻¹.
    /// Sets the closed-loop bandwidth of the rate-tracking loop: Kp = 1/τ.
    rate_gain: Vector3<f32>,
}

impl AngularVelocityController {
    pub fn new(
        conditioning: InputSignalConditioning,
        allocator: ControlAllocator,
        output_map: [InverseActuatorModel; 4],
        rate_gain: Vector3<f32>,
    ) -> Self {
        Self { conditioning, allocator, output_map, rate_gain }
    }
}

impl Controller for AngularVelocityController {
    type Setpoint = AngularVelocitySetpoint;

    fn step(&mut self, state: &VehicleState, setpoint: &AngularVelocitySetpoint) -> ActuatorCommand {
        let cond = self.conditioning.step(state);

        // Rate-P: translate rate error into an angular acceleration setpoint.
        let omega_error = Vector3::new(
            setpoint.roll_rate  - cond.omega_filtered[0],
            setpoint.pitch_rate - cond.omega_filtered[1],
            setpoint.yaw_rate   - cond.omega_filtered[2],
        );
        let acceleration_setpoint = AngularAccelerationSetpoint {
            timestamp_us: setpoint.timestamp_us,
            roll_acceleration:  self.rate_gain[0] * omega_error[0],
            pitch_acceleration: self.rate_gain[1] * omega_error[1],
            yaw_acceleration:   self.rate_gain[2] * omega_error[2],
            specific_thrust:    setpoint.specific_thrust,
        };

        // INDI increment law: Δu = G⁻¹ · (α_sp − α̂), u = u₀ + Δu.
        let u_cmd = self.allocator.allocate(
            &acceleration_setpoint,
            &cond.angular_acceleration,
            cond.specific_thrust,
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
