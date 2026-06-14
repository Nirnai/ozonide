/// Coefficients for a single biquadratic (2nd-order) IIR section.
///
/// Implements the transfer function
///
/// ```text
///         b0 + b1·z⁻¹ + b2·z⁻²
/// H(z) = ──────────────────────
///          1 + a1·z⁻¹ + a2·z⁻²
/// ```
///
/// corresponding to the difference equation
///
/// ```text
/// y[n] = b0·x[n] + b1·x[n-1] + b2·x[n-2] − a1·y[n-1] − a2·y[n-2]
/// ```
///
/// The denominator is **normalized so that `a0 = 1`**; the design
/// functions (`lowpass`, `notch`, …) divide all six raw coefficients by
/// the unnormalized `a0` before constructing this struct, so it is never
/// stored. Note the sign convention: `a1` and `a2` are *subtracted* in the
/// recursion (they sit on the output side of the equation).
///
/// The numerator (`b`) sets the **zeros** → the filter *type* (low/high/notch).
/// The denominator (`a`) sets the **poles** → the resonance and bandwidth.

#[derive(Clone, Copy, Debug)]
pub struct BiquadraticFilterCoefficients {
    a1: f32, 
    a2: f32, 
    b0: f32, 
    b1: f32, 
    b2: f32,
}

impl BiquadraticFilterCoefficients {
    /// Construct from already-normalized coefficients (`a0 = 1` assumed).
    pub(crate) fn new(b0: f32, b1: f32, b2: f32, a1: f32, a2: f32) -> Self {
        Self { a1, a2, b0, b1, b2 }
    }
}

/// A single 2nd-order IIR filter section, realized in **Transposed Direct
/// Form II**.
///
/// One `BiquadraticFilter` is the atomic building block of the library:
/// lowpass, highpass, notch, etc. are all *the same recursion* with
/// different coefficients, and higher-order filters are cascades of these
/// (see [`FilterChain`]).
///
/// # State
///
/// `z1` and `z2` are the two delay registers of the transposed structure.
/// They are **not** past inputs or outputs — they are partial sums computed
/// one and two samples ahead and stashed for future samples. This is why
/// only two state words are needed (vs. four for a literal Direct Form I).
///
/// # Hot loop
///
/// [`process`](Self::process) is five multiplies and four adds, with no
/// transcendental functions — all the `sin`/`cos`/`tan` work happens once,
/// at construction or [`update_coefficients`](Self::update_coefficients)
/// time. Cheap enough to run per-axis at full loop rate.
pub struct BiquadraticFilter {
    coeffs: BiquadraticFilterCoefficients,
    z1: f32,
    z2: f32
}

impl BiquadraticFilter {
    /// Creates a filter from precomputed coefficients with zeroed state.
    pub fn new(coeffs: BiquadraticFilterCoefficients)-> Self {
        Self { coeffs, z1: 0.0, z2: 0.0 }
    }

    pub fn coefficients(&self) -> BiquadraticFilterCoefficients {
        self.coeffs
    }

    /// Processes one input sample and returns one output sample.
    ///
    /// The read of `z1`/`z2` must happen *before* they are overwritten —
    /// the assignment order below is load-bearing, not stylistic.
    pub fn process(&mut self, x: f32) -> f32 {
        let c = &self.coeffs;
        let y = c.b0 * x + self.z1;
        self.z1 = c.b1 * x - c.a1 * y + self.z2;
        self.z2 = c.b2 * x - c.a2 * y;
        return y;
    }

    /// Replaces the coefficients **without touching the state** (`z1`, `z2`).
    ///
    /// This is the correct path for live retuning — e.g. an RPM-tracking
    /// notch whose centre frequency follows motor telemetry. Preserving the
    /// state lets the filter morph smoothly between coefficient sets;
    /// rebuilding via [`new`](Self::new) instead would zero the state and
    /// inject a transient on every update.
    pub fn update_coefficients(&mut self, coeffs: BiquadraticFilterCoefficients){
        self.coeffs = coeffs;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// An allpass biquad (b0=1, all others zero) must pass through the input unchanged.
    #[test]
    fn process_dc_unity_gain() {
        let coeffs = BiquadraticFilterCoefficients::new(1.0, 0.0, 0.0, 0.0, 0.0);
        let mut f = BiquadraticFilter::new(coeffs);
        let y = f.process(1.0);
        assert_eq!(y, 1.0, "allpass identity must output 1.0 for input 1.0, got {}", y);
    }

    /// `update_coefficients` must NOT zero the delay state.
    /// Strategy: run 100 samples through an LP filter to build non-zero state,
    /// then call update_coefficients with the same coefficients. The very next
    /// output must equal what we would get from the running filter (not from a
    /// freshly constructed one with zeroed state).
    #[test]
    fn update_coefficients_preserves_state() {
        // A simple 1-pole LP biquad: b0=b1=0.5, a1=0  (first-order MA is easy)
        // Use the unit-delay (b0=0, b1=1) to build visible state.
        let coeffs = BiquadraticFilterCoefficients::new(0.0, 1.0, 0.0, 0.0, 0.0);
        let mut f = BiquadraticFilter::new(coeffs);

        // Pump 100 samples of constant 1.0 — z1 will be 1.0 after many cycles.
        for _ in 0..100 {
            f.process(1.0);
        }

        // Apply same coefficients (no change in coeffs, but state must survive).
        f.update_coefficients(coeffs);

        // Next sample is 0.0. A unit-delay with state z1=1.0 should output 1.0.
        let y_running = f.process(0.0);

        // A fresh filter from zero state fed 0.0 outputs 0.0.
        let y_fresh = {
            let mut fresh = BiquadraticFilter::new(coeffs);
            fresh.process(0.0)
        };

        assert_ne!(
            y_running, y_fresh,
            "State was zeroed by update_coefficients — y_running={}, y_fresh={}",
            y_running, y_fresh
        );
    }

    /// Verify the transposed-DF2 recursion against a hand-computed impulse response.
    ///
    /// Coefficients: b0=1, b1=0.5, b2=0.25, a1=0.5, a2=0.25
    /// Difference equation: y[n] = x[n] + 0.5·x[n-1] + 0.25·x[n-2] − 0.5·y[n-1] − 0.25·y[n-2]
    ///
    /// Hand-computed for impulse input [1, 0, 0, 0]:
    ///   n=0: y[0] = 1·1 + 0         = 1.0
    ///   n=1: y[1] = 1·0 + 0.5·1 − 0.5·1.0 = 0.0
    ///   n=2: y[2] = 0 + 0 + 0.25·1 − 0.5·0 − 0.25·1 = 0.0
    ///   n=3: y[3] = 0 − 0.5·0 − 0.25·0 = 0.0
    #[test]
    fn transposed_df2_correctness() {
        let coeffs = BiquadraticFilterCoefficients::new(1.0, 0.5, 0.25, 0.5, 0.25);
        let mut f = BiquadraticFilter::new(coeffs);

        let y0 = f.process(1.0);
        let y1 = f.process(0.0);
        let y2 = f.process(0.0);
        let y3 = f.process(0.0);

        let eps = 1e-6_f32;
        assert!((y0 - 1.0).abs() < eps, "y[0] = {} (expected 1.0)", y0);
        assert!((y1 - 0.0).abs() < eps, "y[1] = {} (expected 0.0)", y1);
        assert!((y2 - 0.0).abs() < eps, "y[2] = {} (expected 0.0)", y2);
        assert!((y3 - 0.0).abs() < eps, "y[3] = {} (expected 0.0)", y3);
    }
}