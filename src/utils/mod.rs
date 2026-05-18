use crate::board;


/// Get current time in microseconds
/// Wraps every ~8.9 seconds at 480MHz (fine for relative timing)
#[inline(always)]
pub fn get_time_us() -> u64 {
    const CPU_FREQ_MHZ: u32 = board::CPU_FREQUENCY_HZ / 1_000_000;
    
    unsafe {
        let cyccnt = (*cortex_m::peripheral::DWT::PTR).cyccnt.read();
        (cyccnt as u64) / (CPU_FREQ_MHZ as u64)
    }
}
