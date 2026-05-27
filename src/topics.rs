use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Watch;

use crate::msgs::{ImuSample};

pub static IMU_TOPIC: Watch<CriticalSectionRawMutex, ImuSample, 1> = Watch::new();

