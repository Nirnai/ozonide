use super::vector::Vector3;

pub struct Imu {
    pub timestamp_us: u64,
    pub linear_acceleration: Vector3,
    pub angular_velocity: Vector3,
}