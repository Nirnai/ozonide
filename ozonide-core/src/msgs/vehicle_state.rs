use serde::{Deserialize, Serialize};
use nalgebra::{Quaternion, UnitQuaternion, Vector3, Vector4};

/// Standard gravity (m/s²) - ISO 80000-3. The single source of truth for 
/// the g's ↔ m/s² conversion — referenced by the controller's thrust axis, 
/// the allocator fixtures, and the SITL plant. Do not redefine this 
/// constant elsewhere.
pub const STANDARD_GRAVITY: f32 = 9.80665;

/// Bitflags marking which [`VehicleState`] fields carry live data.
///
/// An unset bit means the field is at its placeholder value and MUST NOT be
/// consumed for control. Fields without a flag (timestamp, angular velocity,
/// specific force) are unconditionally valid: no `VehicleState` is published
/// without an IMU sample.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct StateValidity(u8);

impl StateValidity {
    pub const POSITION: Self = Self(1 << 0);
    pub const VELOCITY: Self = Self(1 << 1);
    pub const ATTITUDE: Self = Self(1 << 2);
    pub const MOTOR_SPEED: Self = Self(1 << 3);
    pub const BATTERY: Self = Self(1 << 4);

    pub const NONE: Self = Self(0);

    #[inline]
    pub const fn contains(self, flag: Self) -> bool {
        self.0 & flag.0 == flag.0
    }
    #[inline]
    pub const fn with(self, flag: Self) -> Self {
        Self(self.0 | flag.0)
    }
    #[inline]
    pub const fn without(self, flag: Self) -> Self {
        Self(self.0 & !flag.0)
    }
}

/// Estimated vehicle state published by the state estimation task, exactly
/// once per IMU sample. One message = one consistent snapshot: all fields
/// refer to the instant of the IMU sample in `timestamp_us` (slow sources
/// like motor telemetry and battery voltage are the latest values known at
/// that instant).
///
/// # Conventions
///
/// World frame: ENU (East-North-Up). 
/// Body frame: FLU (x forward, y left, z up). 
/// Motor order everywhere: `[FR, RL, FL, RR]`.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Serialize, Deserialize)]
pub struct VehicleState {
    /// Timestamp of the IMU sample that produced this estimate (µs since boot).
    pub timestamp_us: u64,

    /// Which optional fields carry live data. Check before consuming.
    pub valid: StateValidity,
   
    /// Position in world frame (ENU), metres. Valid iff `POSITION`.
    pub position: [f32; 3],


    /// Attitude quaternion, stored `[x, y, z, w]` (scalar last, nalgebra
    /// storage order). Active rotation taking vectors expressed in body (FLU)
    /// coordinates to world (ENU) coordinates: `v_world = q * v_body`.
    /// Valid iff `ATTITUDE`; identity placeholder otherwise — do NOT read
    /// the placeholder as "level".
    ///
    /// Prefer [`attitude`](Self::attitude)/[`set_attitude`](Self::set_attitude)
    /// over touching the array: component-order bugs (nalgebra constructors
    /// take `w` first while storage is scalar-last; ROS is `xyzw`) live at
    /// exactly this seam.
    pub attitude: [f32; 4],

    /// Velocity in world frame (ENU), m/s. Valid iff `VELOCITY`.
    pub linear_velocity: [f32; 3],

    /// Angular velocity in body frame (FLU), rad/s, `[x, y, z]` components —
    /// equivalently `[roll_rate, pitch_rate, yaw_rate]` in this convention.
    /// Raw (calibrated) gyroscope passthrough; always valid.
    pub angular_velocity: [f32; 3],

    /// Specific force in body frame (FLU), m/s², raw (calibrated)
    /// accelerometer passthrough; always valid.
    ///
    /// This is NOT linear acceleration: an accelerometer measures reaction
    /// force, so at rest it reads ≈ `[0, 0, +STANDARD_GRAVITY]` and in free
    /// fall it reads ≈ zero. The controller's thrust axis consumes this in
    /// g's: divide by [`STANDARD_GRAVITY`].
    pub specific_force: [f32; 3],


    /// Per-motor mechanical angular speed Ω, rad/s, `[FR, RL, FL, RR]`.
    /// Valid iff `MOTOR_SPEED`; on invalid, consumers fall back to their
    /// actuator model (INDI: lag model).
    ///
    /// Source: bidirectional-DSHOT eRPM telemetry or the simulator's motor
    /// model. Conversion from telemetry: eRPM is *electrical* RPM, so
    /// Ω = eRPM / pole_pairs · 2π/60. NOTE: validate the pole-pair factor
    /// with the bench hover-invariant check `(G·u₀)[3] ≈ 1.0` — an integer
    /// scaling error here propagates quadratically into u₀ = Ω².
    pub motor_speed: [f32; 4],

    /// Battery voltage, volts. Valid iff `BATTERY`; on invalid, consumers
    /// use their configured nominal voltage. Never divide by this field
    /// without checking validity.
    pub battery_voltage: f32,
}

impl VehicleState {
    /// Attitude as a typed rotation (body FLU → world ENU).
    /// Check `valid.contains(StateValidity::ATTITUDE)` first.
    #[inline]
    pub fn attitude(&self) -> UnitQuaternion<f32> {
        let [x, y, z, w] = self.attitude;
        UnitQuaternion::from_quaternion(Quaternion::new(w, x, y, z))
    }

    /// Store an attitude, normalizing storage order in one place.
    #[inline]
    pub fn set_attitude(&mut self, q: &UnitQuaternion<f32>) {
        let c = q.coords; // nalgebra storage: [x, y, z, w]
        self.attitude = [c[0], c[1], c[2], c[3]];
        self.valid = self.valid.with(StateValidity::ATTITUDE);
    }

    #[inline]
    pub fn linear_velocity(&self) -> Vector3<f32> {
        Vector3::from(self.linear_velocity)
    }

    #[inline]
    pub fn position(&self) -> Vector3<f32> {
        Vector3::from(self.position)
    }

    #[inline]
    pub fn angular_velocity(&self) -> Vector3<f32> {
        Vector3::from(self.angular_velocity)
    }

    #[inline]
    pub fn specific_force(&self) -> Vector3<f32> {
        Vector3::from(self.specific_force)
    }

    /// Motor speeds as a vector, or `None` if telemetry is not live —
    /// forcing the fallback decision to the call site.
    #[inline]
    pub fn motor_speed(&self) -> Option<Vector4<f32>> {
        self.valid
            .contains(StateValidity::MOTOR_SPEED)
            .then(|| Vector4::from(self.motor_speed))
    }

    /// Battery voltage, or `None` if no measurement is live.
    #[inline]
    pub fn battery_voltage(&self) -> Option<f32> {
        self.valid
            .contains(StateValidity::BATTERY)
            .then_some(self.battery_voltage)
    }
}

impl Default for VehicleState {
    fn default() -> Self {
        Self {
            timestamp_us: 0,
            valid: StateValidity::NONE,
            position: [0.0; 3],
            attitude: [0.0, 0.0, 0.0, 1.0], // identity, [x,y,z,w]
            linear_velocity: [0.0; 3],
            angular_velocity: [0.0; 3],
            specific_force: [0.0; 3],
            motor_speed: [0.0; 4],
            battery_voltage: 0.0,
        }
    }
}


#[cfg(test)]
mod tests {
    use super::*;
    use core::f32::consts::FRAC_PI_2;

    /// Pins the quaternion convention operationally: 90° yaw (CCW from
    /// above) must take body-x (forward) to world-y (north) in ENU.
    #[test]
    fn attitude_convention_round_trips() {
        let yaw_90 = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), FRAC_PI_2);
        let mut s = VehicleState::default();
        s.set_attitude(&yaw_90);

        assert!(s.valid.contains(StateValidity::ATTITUDE));
        let q = s.attitude();
        let v_world = q * Vector3::new(1.0, 0.0, 0.0); // body forward
        assert!((v_world - Vector3::new(0.0, 1.0, 0.0)).norm() < 1e-6);
    }

    #[test]
    fn attitude_storage_is_scalar_last() {
        let yaw_90 = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), FRAC_PI_2);
        let mut s = VehicleState::default();
        s.set_attitude(&yaw_90);
        let half = FRAC_PI_2 / 2.0;
        assert!((s.attitude[2] - half.sin()).abs() < 1e-6, "z component");
        assert!((s.attitude[3] - half.cos()).abs() < 1e-6, "w component (last)");
        assert!(s.attitude[0].abs() < 1e-6 && s.attitude[1].abs() < 1e-6);
    }

    #[test]
    fn invalid_fields_return_none() {
        let s = VehicleState::default();
        assert!(s.motor_speed().is_none());
        assert!(s.battery_voltage().is_none());
        assert!(!s.valid.contains(StateValidity::ATTITUDE));
    }

    #[test]
    fn validity_flags_compose() {
        let v = StateValidity::NONE
            .with(StateValidity::MOTOR_SPEED)
            .with(StateValidity::BATTERY);
        assert!(v.contains(StateValidity::MOTOR_SPEED));
        assert!(v.contains(StateValidity::BATTERY));
        assert!(!v.contains(StateValidity::POSITION));
        assert!(!v.without(StateValidity::BATTERY).contains(StateValidity::BATTERY));
    }
}
