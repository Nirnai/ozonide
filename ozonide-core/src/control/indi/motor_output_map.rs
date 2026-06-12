/// Static inverse of the motor's steady-state throttle → speed map.
///
/// Forward model (identified on the bench at reference voltage V_ref):
///     Ω_ss(t, V) ≈ (V / V_ref) · (c0 + c1·t)        [rad/s]
/// i.e. an affine fit in throttle, with motor speed scaling linearly in
/// battery voltage. This is feedforward only: inversion errors show up as
/// actuator-effectiveness error, which the INDI increment law absorbs —
/// accuracy here buys transient quality, not stability.
pub struct MotorOutputMap {
    c0: f32,
    c1: f32,        // must be > 0; validate in constructor
    v_ref: f32,
    /// Used when no battery measurement is valid.
    v_nominal: f32,
    /// Floor keeping motors spinning when armed (DSHOT idle).
    throttle_idle: f32,
}

impl MotorOutputMap {
    /// Ω² command (rad²/s²) → normalized throttle [throttle_idle, 1].
    #[inline]
    pub fn throttle_for(&self, u: f32, v_bat: Option<f32>) -> f32 {
        let v = v_bat.unwrap_or(self.v_nominal);
        let omega = libm::sqrtf(u.max(0.0));
        let t = (omega * self.v_ref / v - self.c0) / self.c1;
        t.clamp(self.throttle_idle, 1.0)
    }

    /// Forward model — needed by the lag-model path and by tests.
    #[inline]
    pub fn omega_for(&self, throttle: f32, v_bat: Option<f32>) -> f32 {
        let v = v_bat.unwrap_or(self.v_nominal);
        (v / self.v_ref) * (self.c0 + self.c1 * throttle)
    }
}


// // inside IndiRateController::step, after conditioning and rate-P:
// let u_cmd = self.allocator.allocate(&sp, &omega_dot_f, thrust_f, &u0_f); // Ω²
// self.lag_model.step_toward(&u_cmd.map(libm::sqrtf), dt);                 // lag in Ω!
// let v_bat = state.battery_voltage();
// let motor_throttle =
//     core::array::from_fn(|i| self.output_map.throttle_for(u_cmd[i], v_bat));
// ActuatorCommand { motor_throttle }