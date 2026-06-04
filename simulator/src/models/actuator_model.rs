/// Aerodynamic and dynamic model for a single brushless motor + propeller.
///
/// Models the physical chain from normalized throttle command to rotor forces:
///
/// ```text
/// throttle_cmd [0, 1]
///      │
///      │  linear mapping
///      ▼
/// omega_cmd = omega_max · throttle_cmd
///      │
///      │  first-order lag  (motor inertia + ESC delay)
///      ▼
/// dω/dt = (omega_cmd − ω) / τ
///      │
///      │  blade element momentum theory
///      ▼
/// thrust  F = k_t · ω²
/// torque  Q = k_q · ω²
/// ```
///
/// The quadratic relationship between rotor speed and output forces comes from
/// aerodynamics: thrust and drag torque are both proportional to the square of
/// the air velocity at the blade tips, which is proportional to ω. The
/// throttle-to-force quadratic seen in simpler models is a collapsed version of
/// this, assuming a linear throttle→ω mapping.
///
/// The rotor speed `ω` is a state variable owned by the physics `State` struct,
/// not by this model. All methods are pure functions of their inputs so the RK4
/// integrator can evaluate derivatives at arbitrary intermediate states.
pub struct ActuatorModel {
    /// Maximum rotor speed (rad/s). Maps throttle = 1.0 → omega_cmd = omega_max.
    pub omega_max: f64,
    /// Thrust coefficient (N/(rad/s)²). F = k_t · ω².
    pub k_t: f64,
    /// Reaction torque coefficient (N·m/(rad/s)²). Q = k_q · ω².
    /// The ratio k_q/k_t is a fixed aerodynamic property of the propeller.
    pub k_q: f64,
    /// Motor time constant (s). Captures combined ESC latency and rotor inertia.
    /// Typical value: 0.015 s (15 ms) for a 5-inch racing quadrotor.
    pub tau: f64,
}

impl Default for ActuatorModel {
    fn default() -> Self {
        Self {
            omega_max: 3000.0, // rad/s (~28 000 RPM)
            k_t: 5.5e-7,       // N/(rad/s)² — hover at ~50% throttle
            k_q: 1.1e-8,       // N·m/(rad/s)² — k_q/k_t ≈ 0.02
            tau: 0.015,        // s — 15 ms motor lag
        }
    }
}

impl ActuatorModel {
    // pub fn new(omega_max: f64, k_t: f64, k_q: f64, tau: f64) -> Self {
    //     Self {
    //         omega_max,
    //         k_t,
    //         k_q,
    //         tau,
    //     }
    // }

    /// Time derivative of rotor speed for the first-order motor lag model.
    ///
    /// `dω/dt = (omega_max · throttle_cmd − ω) / τ`
    ///
    /// Integrate this alongside the rigid body state in the RK4 loop.
    /// `throttle_cmd` must be in `[0.0, 1.0]` — clamp at the system boundary
    /// (UDP receive) before calling this function.
    pub fn omega_dot(&self, omega: f64, throttle_cmd: f64) -> f64 {
        debug_assert!(
            (0.0..=1.0).contains(&throttle_cmd),
            "throttle_cmd out of range: {throttle_cmd}"
        );
        let omega_cmd = self.omega_max * throttle_cmd;
        (omega_cmd - omega) / self.tau
    }

    /// Returns `(thrust_force, drag_torque)` for the given rotor speed.
    ///
    /// Both outputs share the same `ω²` factor, so it is computed once.
    /// The sign of `drag_torque` relative to the body Z axis depends on the
    /// motor spin direction (CW vs CCW) and is applied by the caller.
    pub fn wrench(&self, omega: f64) -> (f64, f64) {
        let omega_sq = omega * omega;
        (self.k_t * omega_sq, self.k_q * omega_sq)
    }
}
