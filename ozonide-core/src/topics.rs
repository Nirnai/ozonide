//! Pub/sub topics built on Embassy's [`Watch`] primitive.
//!
//! A [`Topic`] pairs a `Watch` (latest-value store) with a publish counter so
//! that any task can monitor the publish rate without subscribing to the data
//! stream itself.
//!
//! # Subscriber vs Observer
//!
//! `Watch` has two kinds of receivers:
//!
//! - **[`Topic::subscriber`]** — tracked. The `Watch` records whether this
//!   receiver has seen the current value. Calling `.changed().await` on it
//!   returns the next *new* value, blocking until one arrives. Use this when
//!   every sample matters (e.g. a state estimator). Up to `N` tracked
//!   subscribers can exist; exceeding that limit panics at call-site.
//!
//! - **[`Topic::observer`]** — untracked. No slot is consumed and no state is
//!   stored. Call `.try_get()` to read the latest value on demand. Missed
//!   publishes are silently dropped. Use this for displays, loggers, or any
//!   task that only needs the most recent value.
//!
//! Note: `Watch` buffers exactly **one** value. A slow subscriber does not
//! accumulate a backlog — it simply receives the latest available value when
//! it next wakes. Use `embassy_sync::channel::Channel` if you need a queue.

use core::sync::atomic::{AtomicU32, Ordering};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::{self, Watch};

use crate::msgs::{ActuatorCommand, ControlDemand, ImuData, VehicleState};

/// Global IMU topic. Published by [`crate::tasks::imu_task`] at the
/// configured ODR (default 1 kHz). Supports one tracked subscriber.
pub static IMU_TOPIC: Topic<ImuData, 1> = Topic::new();

/// Global vehicle state topic. Published by the state estimation task at IMU rate.
/// Consumed by the control computation task. Fields without an active estimator
/// default to zero — see [`VehicleState`] for which fields are populated.
pub static VEHICLE_STATE_TOPIC: Topic<VehicleState, 1> = Topic::new();

/// Global control demand topic. Published by the control task. Consumed by the
/// actuator task, which applies authority limiting and motor mixing before
/// writing to the transport layer.
pub static CONTROL_DEMAND_TOPIC: Topic<ControlDemand, 1> = Topic::new();

/// Global actuator topic. Published by the actuator task after allocation.
/// Consumed by the transport layer (UDP in SITL, PWM/DSHOT in firmware).
pub static ACTUATOR_TOPIC: Topic<ActuatorCommand, 1> = Topic::new();

/// A pub/sub topic combining a latest-value store with a publish counter.
///
/// `T` is the message type; `N` is the maximum number of concurrent tracked
/// [`subscriber`](Topic::subscriber)s. Declare as a `static`:
///
/// ```ignore
/// static MY_TOPIC: Topic<MyMsg, 2> = Topic::new();
/// ```
pub struct Topic<T: Clone, const N: usize> {
    watch: Watch<CriticalSectionRawMutex, T, N>,
    count: AtomicU32,
}

/// Handle returned by [`Topic::publisher`].
///
/// Calling [`publish`](Publisher::publish) writes the value into the topic and
/// increments the publish counter atomically.
pub struct Publisher<'a, T: Clone, const N: usize> {
    inner: watch::Sender<'a, CriticalSectionRawMutex, T, N>,
    count: &'a AtomicU32,
}

impl<'a, T: Clone, const N: usize> Publisher<'a, T, N> {
    /// Publish a value. Updates the latest-value store and increments the
    /// publish counter. Wakes all waiting subscribers.
    pub fn publish(&self, val: T) {
        self.inner.send(val);
        self.count.fetch_add(1, Ordering::Relaxed);
    }
}

impl<T: Clone, const N: usize> Topic<T, N> {
    /// Create a new topic. Intended for use in `static` initialisers.
    pub const fn new() -> Self {
        Self {
            watch: Watch::new(),
            count: AtomicU32::new(0),
        }
    }

    /// Obtain a [`Publisher`] handle for this topic.
    pub fn publisher(&self) -> Publisher<'_, T, N> {
        Publisher { inner: self.watch.sender(), count: &self.count }
    }

    /// Obtain a tracked subscriber. Blocks in `.changed().await` until the
    /// next publish. Panics if all `N` subscriber slots are already taken —
    /// increase `N` in the topic declaration.
    pub fn subscriber(&self) -> watch::Receiver<'_, CriticalSectionRawMutex, T, N> {
        self.watch.receiver().expect("subscriber limit reached — increase N in Topic<T, N>")
    }

    /// Obtain an untracked observer. Does not consume a subscriber slot.
    /// Use `.try_get()` to read the current value on demand.
    pub fn observer(&self) -> watch::AnonReceiver<'_, CriticalSectionRawMutex, T, N> {
        self.watch.anon_receiver()
    }

    /// Cumulative publish counter. Pass to [`crate::tasks::rate_monitor`] to
    /// track the publish rate of this topic in Hz.
    pub fn count(&self) -> &AtomicU32 {
        &self.count
    }
}
