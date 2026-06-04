# Ozonide Simulator Development Roadmap

Each step has a concrete observable test that must pass before moving to the next.

---

## Step 1 — Realistic Physics Simulation

**Goal:** A physically accurate rigid body model. No active control, no sensor model yet — just truth-state dynamics.

**nalgebra** replaces all raw array math throughout `simulator/src/physics/`:
- `State`: `Vector3<f64>` for pos/vel/omega, `UnitQuaternion<f64>` for attitude
- `QuadParams`: `Vector3<f64>` for inertia, `Matrix3<f64>` for full inertia tensor
- `body_to_world` / `world_to_body`: replaced by `UnitQuaternion::transform_vector`

**Items to implement:**

1. **RK4 integrator** — replace Euler `step()`. Extract a `derivatives(state, throttle_eff, params) → StateDot` function; RK4 calls it 4× per step.

2. **First-order motor lag** — add `throttle_eff: Vector4<f32>` to `State`:
   ```
   dthrottle_eff/dt = (throttle_cmd − throttle_eff) / τ_motor
   ```
   `τ_motor ≈ 15 ms`. Physics integrator uses `throttle_eff`, not `throttle_cmd`. This is the **single most important sim-to-real feature** (per learning-to-fly ablation).

3. **Translational drag**:
   ```
   F_drag_world = −k_drag_lin · vel_world
   ```
   `k_drag_lin ≈ 0.1 N·s/m`.

4. **Rotational drag**:
   ```
   τ_drag_body = −k_drag_rot · omega_body
   ```

5. **Ground contact** — replace position clamp with spring-damper:
   ```
   F_contact_z = max(0, −k_spring · pos_z − k_damp · vel_z)
   ```
   `k_spring = 2000 N/m`, `k_damp = 100 N·s/m`.

6. **Random external disturbances (wind/gusts)**:
   ```
   F_dist ~ N(0, σ_force)   per axis, σ_force ≈ 0.027 · mass · g
   τ_dist ~ N(0, σ_torque)  per axis, σ_torque ≈ σ_force · arm / 10
   ```

7. **TOML config for `QuadParams`** — load from `simulator/config.toml` at startup.

**Files:** `simulator/src/physics/physics.rs`, `simulator/Cargo.toml` (add `nalgebra = "0.33"`, `toml = "0.8"`)

**Pass criterion:** `cargo run -p simulator` with no SITL → drone starts at 1 m, motor lag delays thrust build-up, translational drag damps velocity, drone descends and settles on ground via spring-damper. No divergence, no NaN.

---

## Step 2 — Verify Open-Loop Instability

**Goal:** Confirm the simulator is realistic enough that hover without active control is unstable.

**Scenario A — Airborne perturbation:**
- Spawn at 2 m altitude with small random angular velocity (ω ~ N(0, 0.1) rad/s per axis)
- Apply hover thrust to all motors (constant)
- Expected: drone tips due to disturbances + motor lag, crashes within ~3–5 s

**Scenario B — Ground takeoff:**
- Start on ground (pos_z = 0, all motors off), apply hover throttle at t = 0
- Expected: motor lag delays thrust, any asymmetry causes tipping, drone crashes

**No new code required** — validation run of Step 1 output.

**Pass criterion:** Visual confirmation in browser frontend that drone crashes in both scenarios.

---

## Step 3 — Correct IMU Sensor Model

**Goal:** IMU output passed to SITL/controller matches real MEMS sensor behaviour.

**Items to implement** in `simulator/src/models/imu_model.rs`:

1. **Accelerometer bias** — constant per-axis offset: `bias_accel ~ N(0, 0.05 m/s²)` sampled at init.

2. **Gyroscope bias** — constant per-axis offset: `bias_gyro ~ N(0, 0.005 rad/s)` sampled at init.

3. **Bias random walk** — Brownian drift each step:
   ```
   bias += N(0, σ_walk) · √dt
   ```
   `σ_walk_accel = 0.001 m/s²/√s`, `σ_walk_gyro = 0.0001 rad/s/√s`.

4. **Truth acceleration from RK4** — replace finite-difference velocity with exact specific force from the derivatives function.

5. **Quantization** — round to 16-bit ADC resolution (±16 g accel → 0.5 mg/LSB, ±2000°/s gyro → 0.06°/s/LSB).

**Files:** `simulator/src/models/imu_model.rs`, `simulator/src/physics/physics.rs` (expose acceleration from derivatives)

**Pass criterion:** Log raw IMU output to CSV for 10 s at rest on ground. Accel Z ≈ 9.81 m/s² ± noise + bias. Gyro ≈ bias + noise with visible drift.

---

## Step 4 — IMU-Based PID Hover Controller

**Goal:** A purely IMU-driven PID attitude + rate controller in the SITL that stabilizes the Step 2 open-loop scenarios.

**Control stack** (data flows top to bottom):
```
IMU_TOPIC (ImuData)
    → AttitudeEstimator   → attitude estimate (roll, pitch, rates)
    → AnglePID            → roll_cmd, pitch_cmd (outer loops, 100 Hz)
    → RatePID             → roll_rate_out, pitch_rate_out, yaw_rate_out (inner loops, 1 kHz)
    → ControlAuthority    → clamp + priority scaling
    → ControlAllocation   → 4 motor throttles (X-config mixing)
    → ACTUATOR_TOPIC (ActuatorCommand)
```

**Components:**

1. **`AttitudeEstimator`** — complementary filter (α = 0.98), gyro integration corrected by accel tilt.

2. **`RatePID`** — three independent PID loops (roll/pitch/yaw rate) with anti-windup integrator clamping.

3. **`AnglePID`** — outer roll/pitch angle loops; output feeds as setpoint into rate PID.

4. **`ControlAuthority`** (`ozonide-core/src/control/authority.rs`) — limit enforcement + priority saturation (thrust > roll/pitch > yaw) + anti-windup feedback.

5. **`ControlAllocation`** (`ozonide-core/src/control/allocation.rs`) — X-config pseudoinverse mixing matrix, maps `[thrust, roll, pitch, yaw]` → 4 motor throttles `[0, 1]`.

6. **`Setpoint`** message (`ozonide-core/src/msgs/setpoint.rs`):
   ```rust
   pub struct Setpoint {
       pub roll_deg: f32,
       pub pitch_deg: f32,
       pub yaw_rate_dps: f32,
       pub thrust_norm: f32,  // [0, 1]
   }
   ```

**Files:** `sitl/src/tasks/attitude_estimator.rs`, `sitl/src/tasks/rate_pid.rs`, `sitl/src/tasks/angle_pid.rs`, `ozonide-core/src/control/authority.rs`, `ozonide-core/src/control/allocation.rs`, `ozonide-core/src/msgs/setpoint.rs`

**Pass criterion:** Both Step 2 scenarios stabilize. Drone with perturbations → PID corrects attitude → hovers at ~1 m. Ground takeoff → self-levels → hovers.

---

## Step 5 — RL Environment Interface

**Goal:** A synchronous `SimEnv` API that an RL training loop can drive faster than realtime.

```rust
pub struct SimEnv { ... }
impl SimEnv {
    pub fn reset(&mut self, seed: u64) -> Observation { ... }
    pub fn step(&mut self, action: [f32; 4]) -> StepResult { ... }
}
```

**Observation vector** (18-dim, normalized to ~[-1, 1]):
```
[accel_xyz(3), gyro_xyz(3), quat_wxyz(4), vel_xyz(3), omega_xyz(3), alt(1), throttle_eff_avg(1)]
```

**Reward (hover):**
```
r = -(w_pos·pos_z_error² + w_vel·|vel|² + w_tilt·tilt_angle² + w_act·|Δaction|²)
```

**Termination conditions:**
- `pos_z < −0.1 m` → crashed
- `tilt > 75°` → flipped
- `pos_z > 30 m` → out of bounds
- `t > 10 s` → timeout

**Headless mode** — skips `thread::sleep` pacing; physics runs at CPU speed.

**Determinism** — `reset(seed)` initializes RNG from seed; same seed → identical trajectory.

**Files:** `simulator/src/env.rs`, `simulator/src/main.rs` (headless flag)

**Pass criterion:** Integration test: `reset(42)` + 500 `step()` calls reproducible across runs. Throughput ≥ 100 k steps/s headless.

---

## Step 6 — Advanced Controllers

**6a — RL Controller** (policy network, runs in SITL)
- Framework: **`burn`** (native Rust deep learning) — `NdArray` backend for CPU, `Wgpu` for GPU
- Simple MLP policy trained via PPO or TD3; training loop drives `SimEnv` headless
- Reference: learning-to-fly's 32-step action history, asymmetric actor-critic, curriculum reward

**6b — Linear MPC**
- Linearize drone dynamics at hover
- Solve QP over receding horizon (N=10 steps, dt=10 ms) using `osqp` Rust crate

**6c — Nonlinear MPC** (later)
- Full nonlinear dynamics inside optimizer; handles aggressive maneuvers

**Domain randomization** (prerequisite for RL sim-to-real transfer):
- Per-episode: randomize mass ×U(0.85,1.15), inertia ×U(0.8,1.2), `k_thrust` ×U(0.85,1.15), `τ_motor` ×U(0.8,1.2)
- Random constant wind force per episode: `N(0, 0.05·mass·g)` per axis
- IMU bias resampled each reset

**Pass criterion:** RL policy converges to stable hover; MPC tracks altitude setpoint.

---

## Verification Summary

| Step | Pass Criterion |
|------|----------------|
| 1 | Drone at 1 m + hover thrust, no control → descends to ground, no NaN |
| 2 | Perturbation scenario + ground takeoff scenario both crash within 5 s |
| 3 | IMU CSV at rest: accel_z ≈ 9.81 + bias + noise; gyro ≈ bias + noise with drift |
| 4 | Both Step 2 scenarios stabilize with PID running in SITL |
| 5 | `reset(42)` + 500 steps reproducible; throughput ≥ 100 k steps/s headless |
| 6 | RL policy converges to hover; MPC tracks altitude setpoint |
