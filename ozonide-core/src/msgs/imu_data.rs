use serde::{Deserialize, Serialize};

/// A single IMU measurement in SI units, body frame (FLU: X forward, Y left, Z up).
///
/// Produced by [`crate::traits::ImuSource::read`] and published on
/// [`crate::topics::IMU_TOPIC`] at the configured ODR (default 1 kHz).
///
/// # Calibration contract
///
/// Values in this message are **calibrated**: the corrections described below
/// have already been applied in the driver, upstream of this message. No
/// consumer (estimator, INDI conditioning, logger) applies further sensor
/// corrections. Residual *in-run bias drift* is explicitly NOT corrected here —
/// that is the state estimator's job, carried as bias states in the UKF.
///
/// If you are reading this because a downstream signal looks biased: first
/// check whether the calibration steps below have actually been performed for
/// this physical unit, before adding compensation anywhere else.
///
/// ## What to calibrate, how, and when
///
/// ### 1. Gyroscope bias — power-on capture (every boot)
///
/// **What:** additive offset b_g (rad/s) per axis. Dominated by turn-on bias,
/// which changes every power cycle; whatever we measure offline is stale by
/// the next boot.
///
/// **How:** at startup, with the vehicle still (verify: gyro variance over the
/// window below a threshold, else retry), average N ≈ 1–2 s of samples per
/// axis. Store as `b_g`. Compensation: `gyro = raw_gyro − b_g`.
///
/// **When:** every boot, before arming. Re-capture on disarm is optional
/// (in-run drift after warm-up is small and the UKF tracks it).
///
/// **Failure mode if skipped:** complementary filter / UKF start with a
/// constant attitude drift; yaw drifts unboundedly until VIO exists. Note the
/// INDI rate loop itself is immune (differentiation kills constant bias) —
/// this is for the estimator's benefit.
///
/// ### 2. Accelerometer bias + scale + misalignment — six-position test (once per build)
///
/// **What:** the affine error model `a_true = M · (raw − b_a)` where b_a is a
/// 3-vector bias (m/s²) and M is a 3×3 matrix combining per-axis scale error
/// (~0.5–1% on the ICM-42688-P uncompensated) and cross-axis misalignment
/// (sensor axes vs. package vs. PCB mounting). A Kalman bias state CANNOT
/// absorb M — scale/misalignment errors are multiplicative, proportional to
/// the signal, and effectively unobservable in flight.
///
/// **How:** place the vehicle (or the bare board, if the IMU→frame mounting is
/// rigid and repeatable) in 6 static orientations: ±X, ±Y, ±Z up, each held a
/// few seconds, averaged. Known truth in each pose: specific force =
/// ±STANDARD_GRAVITY on one axis, 0 on the others. Solve least-squares for the
/// 12 parameters (M, b_a). A machined corner block or the flat faces of the
/// frame are accurate enough; perfection in pose matters less than stillness.
/// Compensation: `specific_force = M · (raw_accel − b_a)`.
///
/// **When:** once per physical unit, repeated only if the IMU is remounted,
/// the board is reflowed, or after a hard crash. Drift of M over time is
/// negligible at hobby scale.
///
/// **Failure mode if skipped:** a z-axis accel bias maps 1:1 into the INDI
/// thrust axis (0.05 m/s² ≈ 0.5% thrust error) and poisons the bench
/// hover-invariant check `(G·u₀)[3] ≈ 1.0`, making k_T / mass / eRPM-scaling
/// errors indistinguishable from sensor error. Scale/misalignment leak into
/// attitude during aggressive motion.
///
/// ### 3. Temperature compensation — deliberately NOT done offline
///
/// **What:** bias varies with die temperature (hence the `temperature` field).
///
/// **Decision:** we do not build temperature look-up tables. The UKF's bias
/// states track thermal drift in-run; that is the part of the error budget
/// they are genuinely good at. The `temperature` field is published so that
/// (a) logs can correlate drift with temperature post-hoc, and (b) a
/// compensation table could be added in the driver later WITHOUT changing
/// this message or any consumer — the contract "values are calibrated" would
/// simply start covering one more term.
///
/// ### 4. Gyro scale/misalignment — deliberately skipped
///
/// Requires a rate table (known rotation rates) to identify; errors at our
/// scale (<1%) are below what the control loops notice, and INDI's incremental
/// structure absorbs them as effectiveness error. Revisit only if attitude
/// estimates show systematic error proportional to rotation speed.
///
/// ## Where the corrections live (pipeline position)
///
/// ```text
/// sensor registers → driver: apply b_g, (M, b_a) → ImuData (this message)
///                                                   → estimator (tracks residual drift as states)
///                                                   → INDI conditioning (consumes as-is)
/// ```
///
/// Calibration parameters are loaded from persistent config at boot
/// (`CalibrationData { gyro_bias, accel_bias, accel_misalignment }`); the
/// power-on gyro capture overwrites `gyro_bias` in RAM each boot.
///
/// TODO(calibration):
///   - [ ] stillness-checked power-on gyro bias capture in the IMU task
///   - [ ] six-position accel routine (bench): pose capture + LSQ solve,
///         can run as a host-side tool over logged raw samples
///   - [ ] `CalibrationData` struct + persistent storage/load at boot
///   - [ ] driver applies corrections before publishing; until then this
///         message is RAW and the doc above is aspirational — flip this
///         line when done
///   - [ ] validate end-to-end: calibrated unit, level and still, must read
///         specific_force ≈ [0, 0, +STANDARD_GRAVITY] within ~0.02 m/s²
///         and gyro ≈ 0 within noise; add as a preflight check
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Serialize, Deserialize)]
pub struct ImuData {
    /// Microseconds since boot, taken when the DRDY interrupt fired.
    /// Timestamps data-ready, not mid-sample: sensor-internal filter latency
    /// (a few hundred µs, UI-filter dependent) is constant and uncompensated.
    pub timestamp_us: u64,
    /// Specific force in m/s², body frame — calibrated accelerometer reading
    /// (see calibration contract above). Includes the reaction to gravity:
    /// at rest ≈ [0, 0, +STANDARD_GRAVITY], in free fall ≈ 0.
    pub specific_force: [f32; 3],
    /// Angular velocity in rad/s, body frame, [x, y, z] components —
    /// calibrated gyroscope reading (see calibration contract above).
    pub angular_velocity: [f32; 3],
    /// Die temperature in °C. Published for log correlation and future
    /// temperature compensation; no consumer should branch on it today.
    pub temperature: f32,
}