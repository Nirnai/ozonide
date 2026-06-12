use crate::{
    msgs::{AttitudeSetpoint, ControlDemand, VehicleState},
    traits::Controller,
};

use crate::control::pid::{
    AngularVelocityController, AngularVelocityGains, AttitudeController, AttitudeGains,
};

pub struct CascadedPidController {
    angular_velocity_controller: AngularVelocityController,
    angular_velocity_controller_gains: AngularVelocityGains,
    attitude_controller: AttitudeController,
    attitude_controller_gains: AttitudeGains,
    last_timestamp_us: Option<u64>,
}

impl CascadedPidController {
    pub fn new(attitude_gains: AttitudeGains, angular_velocity_gains: AngularVelocityGains) -> Self {
        Self {
            attitude_controller: AttitudeController::default(),
            attitude_controller_gains: attitude_gains,
            angular_velocity_controller: AngularVelocityController::default(),
            angular_velocity_controller_gains: angular_velocity_gains,
            last_timestamp_us: None,
        }
    }
}

impl Controller for CascadedPidController {
    type Setpoint = AttitudeSetpoint;

    fn update(&mut self, state: &VehicleState, setpoint: &AttitudeSetpoint) -> ControlDemand {
        let Some(last_timestamp_us) = self.last_timestamp_us else {
            self.last_timestamp_us = Some(state.timestamp_us);
            return ControlDemand::default();
        };

        // Simulator reset: timestamp went backward — clear integrators and reseed.
        if state.timestamp_us < last_timestamp_us {
            self.reset();
            self.last_timestamp_us = Some(state.timestamp_us);
            return ControlDemand::default();
        }

        let dt = (state.timestamp_us - last_timestamp_us) as f32 * 1e-6;
        self.last_timestamp_us = Some(state.timestamp_us);

        let angular_velocity_setpoint =
            self.attitude_controller
                .update(state, setpoint, &self.attitude_controller_gains, dt);

        let torque_setpoint = self.angular_velocity_controller.update(
            state,
            &angular_velocity_setpoint,
            &self.angular_velocity_controller_gains,
            dt,
        );

        ControlDemand {
            thrust: setpoint.thrust,
            roll_torque: torque_setpoint.roll_torque,
            pitch_torque: torque_setpoint.pitch_torque,
            yaw_torque: torque_setpoint.yaw_torque,
        }
    }

    fn reset(&mut self) {
        self.attitude_controller.reset();
        self.angular_velocity_controller.reset();
        self.last_timestamp_us = None;
    }
}
