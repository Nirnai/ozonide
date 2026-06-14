use heapless::Vec;

use super::biquadratic_filter::{BiquadraticFilter, BiquadraticFilterCoefficients};


/// A cascade of `N` biquad sections, run in series.
///
/// An Nth-order IIR filter is realized as ⌈order/2⌉ biquads. Even orders use
/// exactly order/2 full sections; **odd orders** use one degenerate section
/// with `b2 = a2 = 0`, which is a 1st-order filter wearing a biquad's clothes.
/// The chain itself stays oblivious to this — it just folds `process` across
/// its sections, so no special-casing is needed here.
pub struct FilterChain<const N: usize> {
    pub (crate) sections: Vec<BiquadraticFilter, N>,
}


impl <const N: usize> FilterChain<N> {
    pub fn new(sections: Vec<BiquadraticFilter, N>) -> Self {
        Self { sections }
    }

    pub fn process(&mut self, x: f32) -> f32 {
        let mut y = x;
        for section in &mut self.sections {
            y = section.process(y);
        }
        return y;
    }

    pub(crate) fn update_coefficients(&mut self, coeffs: [BiquadraticFilterCoefficients; N]) {
        for (section, c) in self.sections.iter_mut().zip(coeffs) {
            section.update_coefficients(c);
        }
    }

    /// Copy coefficients from `source`, **preserving each section's state**
    /// (`z1`/`z2`). The state-preserving retune primitive: build a fresh
    /// chain from a factory with new parameters, then copy its coefficients
    /// in without glitching the running filter.
    ///
    /// Both chains must have the same section count (same filter shape).
    pub fn update_coefficients_from(&mut self, source: &FilterChain<N>) {
        debug_assert_eq!(self.sections.len(), source.sections.len());
        for (dst, src) in self.sections.iter_mut().zip(source.sections.iter()) {
            dst.update_coefficients(src.coefficients());
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use heapless::Vec;

    /// Helper: build an allpass (identity) biquad — b0=1, rest zero.
    fn allpass_coeffs() -> BiquadraticFilterCoefficients {
        BiquadraticFilterCoefficients::new(1.0, 0.0, 0.0, 0.0, 0.0)
    }

    /// Helper: build a single-sample delay biquad.
    /// H(z) = z^-1 → b0=0, b1=1, b2=0, a1=0, a2=0
    fn unit_delay_coeffs() -> BiquadraticFilterCoefficients {
        BiquadraticFilterCoefficients::new(0.0, 1.0, 0.0, 0.0, 0.0)
    }

    /// A chain of two unit-delay sections must produce a 2-sample lag,
    /// NOT apply each delay to the original input (which would give 1-sample lag twice).
    #[test]
    fn cascade_is_not_all_applied_to_original_input() {
        let mut sections: Vec<BiquadraticFilter, 2> = Vec::new();
        let _ = sections.push(BiquadraticFilter::new(unit_delay_coeffs()));
        let _ = sections.push(BiquadraticFilter::new(unit_delay_coeffs()));
        let mut chain: FilterChain<2> = FilterChain { sections };

        // Feed impulse [1, 0, 0, ...]
        // With correct cascade: output is [0, 0, 1, ...]  (2-sample delay)
        // With the old bug (both applied to x): output is [0, 1, 0, ...]  (1-sample delay)
        let y0 = chain.process(1.0); // sample 0 → should be 0
        let y1 = chain.process(0.0); // sample 1 → should be 0
        let y2 = chain.process(0.0); // sample 2 → should be 1

        assert_eq!(y0, 0.0, "y0 should be 0 (2-sample delay)");
        assert_eq!(y1, 0.0, "y1 should be 0 (2-sample delay), got {}", y1);
        assert_eq!(y2, 1.0, "y2 should be 1 (2-sample delay), got {}", y2);
    }

    /// `update_coefficients_from` should copy coefficients but NOT reset state.
    /// After the update, output must remain continuous (not jump to zero-state behavior).
    ///
    /// Strategy: run a 1-pole IIR lowpass (b0=0.5, a1=-0.5) with constant 1.0 input
    /// so that z1 converges to 0.5. Then swap in unit-delay (b0=0, b1=1) coefficients.
    /// A unit-delay filter reading a non-zero z1 outputs z1 on the next `process(0.0)`,
    /// while a freshly constructed unit-delay (z1=0) outputs 0.0. These must differ.
    #[test]
    fn update_coefficients_from_copies_coeffs_not_state() {
        // b0=0.5, b1=0, b2=0, a1=-0.5, a2=0  →  single real pole at +0.5
        // Transposed DF-II: y = 0.5*x + z1;  z1 = 0*x - (-0.5)*y = 0.5*y;  z2 stays 0.
        // Settling to constant 1.0 input: y→1.0, z1→0.5.
        let lp_coeffs = BiquadraticFilterCoefficients::new(0.5, 0.0, 0.0, -0.5, 0.0);

        let mut sections_run: Vec<BiquadraticFilter, 1> = Vec::new();
        let _ = sections_run.push(BiquadraticFilter::new(lp_coeffs));
        let mut running: FilterChain<1> = FilterChain { sections: sections_run };

        // Settle with constant 1.0 — z1 converges to ~0.5 after many samples.
        for _ in 0..200 {
            running.process(1.0);
        }

        // Build a source chain with unit-delay coefficients (b0=0, b1=1, rest 0).
        let mut sections_src: Vec<BiquadraticFilter, 1> = Vec::new();
        let _ = sections_src.push(BiquadraticFilter::new(unit_delay_coeffs()));
        let source: FilterChain<1> = FilterChain { sections: sections_src };

        // Apply new coefficients — state (z1≈0.5) must be preserved.
        running.update_coefficients_from(&source);

        // Next input is 0.0:
        //   running chain: y = b0*0 + z1 = 0 + 0.5 = 0.5  (state preserved)
        //   fresh unit-delay:  y = b0*0 + z1 = 0 + 0.0 = 0.0  (zero state)
        let y_after = running.process(0.0);
        let y_fresh = {
            let mut fresh = BiquadraticFilter::new(unit_delay_coeffs());
            fresh.process(0.0)
        };

        assert_ne!(
            y_after, y_fresh,
            "State was zeroed on coefficient update — y_after={}, y_fresh={}",
            y_after, y_fresh
        );
        // Extra sanity: the running chain should output something close to 0.5.
        assert!(y_after > 0.4, "Expected y_after ≈ 0.5, got {}", y_after);
    }
}
