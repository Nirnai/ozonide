//! Rigid body dynamics for a quadrotor in ENU world frame.
//!
//! World frame: East-North-Up (ENU), Z points up.
//! Body frame:  X forward, Y left, Z up.
//! Quaternion q rotates vectors from body to world: v_world = q * v_body * q_conj.

/// Quadrotor physical parameters.
pub struct QuadParams {
    /// Total mass in kg.
    pub mass: f64,
    /// Moment of inertia [Ixx, Iyy, Izz] in kg·m².
    pub inertia: [f64; 3],
    /// Motor arm length from center to motor in metres.
    pub arm: f64,
    /// Thrust coefficient: thrust (N) = k_thrust * throttle^2.
    pub k_thrust: f64,
    /// Drag coefficient: reaction torque (N·m) = k_drag * throttle^2.
    pub k_drag: f64,
}

impl Default for QuadParams {
    fn default() -> Self {
        Self {
            mass: 0.5,
            inertia: [4e-3, 4e-3, 7e-3],
            arm: 0.15,
            k_thrust: 3.0,
            k_drag: 0.05,
        }
    }
}

/// Full rigid body state.
#[derive(Clone, Copy)]
pub struct State {
    /// Position in ENU world frame (metres).
    pub pos: [f64; 3],
    /// Velocity in ENU world frame (m/s).
    pub vel: [f64; 3],
    /// Unit quaternion [w, x, y, z]: body → world rotation.
    pub quat: [f64; 4],
    /// Angular velocity in body frame (rad/s).
    pub omega: [f64; 3],
}

impl Default for State {
    fn default() -> Self {
        Self {
            pos: [0.0, 0.0, 1.0],
            vel: [0.0; 3],
            quat: [1.0, 0.0, 0.0, 0.0], // identity: body aligned with world
            omega: [0.0; 3],
        }
    }
}

/// Compute net force and torque from motor throttles.
///
/// Motor layout (X config, top view, Z up):
///   2(FL,CCW) --- 0(FR,CW)
///       |     ×     |
///   1(RL,CW) --- 3(RR,CCW)
fn motor_wrench(p: &QuadParams, throttle: [f32; 4]) -> ([f64; 3], [f64; 3]) {
    let thrust: [f64; 4] = throttle.map(|t| p.k_thrust * (t as f64).powi(2));
    let drag: [f64; 4] = throttle.map(|t| p.k_drag * (t as f64).powi(2));

    // Positive thrust along body +Z
    let f_z = thrust.iter().sum::<f64>();

    // Roll (about body X): right motors 0,3 push down relative to left motors 1,2
    let mx = p.arm * (-thrust[0] + thrust[1] + thrust[2] - thrust[3]);
    // Pitch (about body Y): front motors 0,2 push down relative to rear motors 1,3
    let my = p.arm * (-thrust[0] - thrust[2] + thrust[1] + thrust[3]);
    // Yaw (about body Z): CCW reaction vs CW reaction
    // FR(CW)→+, RL(CW)→+, FL(CCW)→-, RR(CCW)→-
    let mz = drag[0] + drag[1] - drag[2] - drag[3];

    ([0.0, 0.0, f_z], [mx, my, mz])
}

/// Rotate a vector from body to world frame using quaternion q = [w,x,y,z].
pub fn body_to_world(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let [w, x, y, z] = q;
    let tx = 2.0 * (y * v[2] - z * v[1]);
    let ty = 2.0 * (z * v[0] - x * v[2]);
    let tz = 2.0 * (x * v[1] - y * v[0]);
    [
        v[0] + w * tx + y * tz - z * ty,
        v[1] + w * ty + z * tx - x * tz,
        v[2] + w * tz + x * ty - y * tx,
    ]
}

/// Rotate a vector from world to body frame (conjugate of q).
pub fn world_to_body(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let q_conj = [q[0], -q[1], -q[2], -q[3]];
    body_to_world(q_conj, v)
}

/// Advance the state by `dt` seconds given motor throttles (Euler integration).
pub fn step(state: &State, throttle: [f32; 4], p: &QuadParams, dt: f64) -> State {
    const G: f64 = 9.80665;
    let (f_body, torque_body) = motor_wrench(p, throttle);

    // Linear acceleration in world frame: thrust + gravity
    let f_world = body_to_world(state.quat, f_body);
    let acc = [
        f_world[0] / p.mass,
        f_world[1] / p.mass,
        f_world[2] / p.mass - G,
    ];

    // Angular acceleration in body frame (Euler equations, simplified diagonal inertia)
    let [ixx, iyy, izz] = p.inertia;
    let [ox, oy, oz] = state.omega;
    let alpha = [
        (torque_body[0] - (izz - iyy) * oy * oz) / ixx,
        (torque_body[1] - (ixx - izz) * oz * ox) / iyy,
        (torque_body[2] - (iyy - ixx) * ox * oy) / izz,
    ];

    // Integrate velocity and position
    let vel = state.vel.map2(acc, |v, a| v + a * dt);
    let pos = [
        state.pos[0] + state.vel[0] * dt,
        state.pos[1] + state.vel[1] * dt,
        state.pos[2] + state.vel[2] * dt,
    ];

    // Integrate omega
    let omega = [
        state.omega[0] + alpha[0] * dt,
        state.omega[1] + alpha[1] * dt,
        state.omega[2] + alpha[2] * dt,
    ];

    // Integrate quaternion: q_dot = 0.5 * q * [0, omega]
    let [w, qx, qy, qz] = state.quat;
    let dw = 0.5 * (-qx * ox - qy * oy - qz * oz);
    let dx = 0.5 * (w * ox + qy * oz - qz * oy);
    let dy = 0.5 * (w * oy + qz * ox - qx * oz);
    let dz = 0.5 * (w * oz + qx * oy - qy * ox);
    let quat_raw = [w + dw * dt, qx + dx * dt, qy + dy * dt, qz + dz * dt];
    let qnorm = quat_raw.iter().map(|v| v * v).sum::<f64>().sqrt();
    let quat = quat_raw.map(|v| v / qnorm);

    // Clamp position to ground
    let pos = if pos[2] < 0.0 {
        [pos[0], pos[1], 0.0]
    } else {
        pos
    };

    State { pos, vel, quat, omega }
}

trait Map2<T, const N: usize> {
    fn map2<U, F: Fn(T, U) -> T>(self, other: [U; N], f: F) -> [T; N];
}

impl<const N: usize> Map2<f64, N> for [f64; N] {
    fn map2<U, F: Fn(f64, U) -> f64>(self, other: [U; N], f: F) -> [f64; N] {
        let mut out = self;
        for (a, b) in out.iter_mut().zip(other) {
            *a = f(*a, b);
        }
        out
    }
}
