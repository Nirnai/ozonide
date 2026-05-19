use serde::{Serialize, Deserialize};
use defmt::{Format};

#[derive(Clone, Copy, Serialize, Deserialize, Default, Format)]
pub struct ImuSample {
    pub timestamp_us: u64,
    pub linear_acceleration: [f32; 3],  // m/s², body frame
    pub angular_velocity: [f32; 3],   // rad/s, body frame
    pub temperature: f32,  // °C, for bias compensation
}

// impl ImuSample {
//     pub const ZERO: Self = Self {
//         timestamp_us: 0,
//         linear_acceleration: [0.0; 3],
//         angular_velocity: [0.0; 3],
//         temperature: 0.0,
//     };
// }