use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Watch;
use core::sync::atomic::{AtomicU32};

use crate::msgs::{ImuSample};

pub static IMU_TOPIC: Watch<CriticalSectionRawMutex, ImuSample, 1> = Watch::new();
pub static IMU_PUB_COUNT: AtomicU32 = AtomicU32::new(0);
