use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicU32, Ordering};

use msgs::*;

pub struct Topics {
    pub imu_sample: Topic<ImuSample>,
}


pub struct Topic<T> {
    seq: AtomicU32,
    data: UnsafeCell<T>,
}

unsafe impl<T: Copy> Sync for Topic<T> {}

impl <T: Copy> Topic<T> {
    pub const fn new(initial_value: T) -> Self {
        Self {
            seq: AtomicU32::new(0),
            data: UnsafeCell::new(initial_value),
        }
    }

    pub fn publish(&self, value: T) {
        self.seq.fetch_add(1, Ordering::Release);
        unsafe { *self.data.get() = value; }
        self.seq.fetch_add(1, Ordering::Release);
    }

    pub fn read(&self) -> T {
        loop {
            if let Some((_, data)) = self.try_read() {
                return data;
            }
        }
    }

    fn try_read(&self) -> Option<(u32, T)> {
        let seq = self.seq.load(Ordering::Acquire);
        if Self::is_odd(seq) {
            return None;
        }
        let data = unsafe { *self.data.get() };
        if !Self::data_valid(seq) {
            return None;
        }
        return Some((seq, data));
    }

    fn is_odd(seq: u32) -> bool {
        seq & 1 == 1
    }

    fn data_valid(seq: u32) -> bool {
        let seq_now = self.seq.load(Ordering::Acquire);
        let overwritten = seq_now != seq;
        return !overwritten
    }
}