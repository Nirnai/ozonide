//! Utility functions and helpers
//!
//! Common utilities for timing, math, data structures, etc.

// pub mod timing;
// pub mod math;
// pub mod buffers;

/// Simple delay helper (will be replaced with proper timing later)
pub fn delay_ms(_ms: u32) {
    todo!("Implement delay using timer peripheral")
}

/// Get system time in microseconds
pub fn get_time_us() -> u64 {
    todo!("Read from hardware timer")
}
