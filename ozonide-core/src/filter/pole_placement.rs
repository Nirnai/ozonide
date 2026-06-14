use core::f32::consts::PI;
use heapless::Vec;
use libm::{cosf, tanf};

use super::biquadratic_filter::{BiquadraticFilter, BiquadraticFilterCoefficients};
use super::filter_chain::FilterChain;

pub const MAX_ORDER: usize = 6;
pub const MAX_SECTIONS: usize = MAX_ORDER.div_ceil(2);

pub fn lowpass(sample_rate: f32, cutoff: f32, family: FilterFamily, order: u8) -> FilterChain<MAX_SECTIONS> {
    design(sample_rate, cutoff, family, order, Response::Lowpass)
}

pub fn highpass(sample_rate: f32, cutoff: f32, family: FilterFamily, order: u8) -> FilterChain<MAX_SECTIONS> {
    design(sample_rate, cutoff, family, order, Response::Highpass)
}

fn design(sample_rate: f32, cutoff: f32, family: FilterFamily, order: u8, response: Response) -> FilterChain<MAX_SECTIONS> {
    debug_assert!(cutoff > 0.0 && cutoff < sample_rate / 2.0, "cutoff must be in (0, Nyquist)");
    let warped = tanf(PI * cutoff / sample_rate);
    let mut sections = Vec::new();
    for proto in family.prototype(order) {
        let coeffs = proto.discretize(response, warped);
        let _ = sections.push(BiquadraticFilter::new(coeffs));
    }
    FilterChain { sections }
}

#[derive(Clone, Copy, Debug)]
pub(crate) enum PrototypePoles {
    SecondOrder { omega0: f32, q: f32 },
    FirstOrder { omega0: f32 },
}

#[derive(Clone, Copy)]
enum Response { Lowpass, Highpass }

impl PrototypePoles {
    fn discretize(self, response: Response, warped: f32) -> BiquadraticFilterCoefficients {
        match self {
            PrototypePoles::SecondOrder { omega0, q } => {
                let a = match response {
                    Response::Lowpass => warped * omega0,
                    Response::Highpass => warped / omega0,
                };
                let a2 = a * a;
                let g = a / q;
                let norm = 1.0 + g + a2;
                let a1 = 2.0 * (a2 - 1.0) / norm;
                let a2c = (1.0 - g + a2) / norm;
                let (b0, b1, b2) = match response {
                    Response::Lowpass => (a2 / norm, 2.0 * a2 / norm, a2 / norm),
                    Response::Highpass => (1.0 / norm, -2.0 / norm, 1.0 / norm),
                };
                BiquadraticFilterCoefficients::new(b0, b1, b2, a1, a2c)
            }
            PrototypePoles::FirstOrder { omega0 } => {
                let a = match response {
                    Response::Lowpass => warped * omega0,
                    Response::Highpass => warped / omega0,
                };
                let norm = 1.0 + a;
                let a1 = (a - 1.0) / norm;
                let (b0, b1) = match response {
                    Response::Lowpass => (a / norm, a / norm),
                    Response::Highpass => (1.0 / norm, -1.0 / norm),
                };
                BiquadraticFilterCoefficients::new(b0, b1, 0.0, a1, 0.0)
            }
        }
    }
}

pub enum FilterFamily {
    Butterworth,
    Bessel,
}

impl FilterFamily {
    pub(crate) fn prototype(&self, order: u8) -> Vec<PrototypePoles, MAX_SECTIONS> {
        debug_assert!(order >= 1 && order as usize <= MAX_ORDER);
        match self {
            FilterFamily::Butterworth => butterworth_prototype(order),
            FilterFamily::Bessel => bessel_prototype(order),
        }
    }
}

fn butterworth_prototype(order: u8) -> Vec<PrototypePoles, MAX_SECTIONS> {
    let n = order as usize;
    let mut sections = Vec::new();
    for k in 0..n / 2 {
        let theta = PI / 2.0 + ((2 * k + 1) as f32) * PI / (2.0 * n as f32);
        let q = -1.0 / (2.0 * cosf(theta));
        let _ = sections.push(PrototypePoles::SecondOrder { omega0: 1.0, q });
    }
    if n % 2 == 1 {
        let _ = sections.push(PrototypePoles::FirstOrder { omega0: 1.0 });
    }
    sections
}

struct BesselEntry {
    pairs: &'static [(f32, f32)],
    real: Option<f32>,
}

static BESSEL: [BesselEntry; MAX_ORDER - 1] = [
    BesselEntry { pairs: &[(1.2720, 0.5774)], real: None },           // N=2
    BesselEntry { pairs: &[(1.4476, 0.6910)], real: Some(1.3227) },   // N=3
    BesselEntry { pairs: &[(1.4302, 0.5219), (1.6034, 0.8055)], real: None }, // N=4
    BesselEntry { pairs: &[(1.5563, 0.5635), (1.7554, 0.9165)], real: Some(1.5023) }, // N=5
    BesselEntry { pairs: &[(1.6039, 0.5103), (1.6892, 0.6112), (1.9047, 1.0233)], real: None }, // N=6
];

fn bessel_prototype(order: u8) -> Vec<PrototypePoles, MAX_SECTIONS> {
    let entry = &BESSEL[order as usize - 2];
    let mut sections = Vec::new();
    for &(omega0, q) in entry.pairs {
        let _ = sections.push(PrototypePoles::SecondOrder { omega0, q });
    }
    if let Some(omega0) = entry.real {
        let _ = sections.push(PrototypePoles::FirstOrder { omega0 });
    }
    sections
}

#[cfg(test)]
mod tests {
    use super::*;

    /// DC gain of a Butterworth LP should be ≈ 1.0 after settling.
    #[test]
    fn butterworth_lp_dc_gain_is_unity() {
        let mut f = lowpass(1000.0, 50.0, FilterFamily::Butterworth, 2);
        let mut out = 0.0_f32;
        for _ in 0..2000 {
            out = f.process(1.0);
        }
        assert!(out > 0.99, "DC gain was {}", out);
    }

    /// Nyquist tone (alternating ±1) should be deeply attenuated by LP.
    #[test]
    fn butterworth_lp_rejects_nyquist() {
        let mut f = lowpass(1000.0, 50.0, FilterFamily::Butterworth, 2);
        let mut out = 0.0_f32;
        for n in 0..5000_u32 {
            let x = if n % 2 == 0 { 1.0_f32 } else { -1.0_f32 };
            out = f.process(x);
        }
        // Check last 100 outputs via RMS — but we only store final here, just check magnitude.
        assert!(libm::fabsf(out) < 0.01, "Nyquist not rejected, last out = {}", out);
    }

    /// HP filter at DC input should converge to near-zero.
    #[test]
    fn butterworth_hp_rejects_dc() {
        let mut f = highpass(1000.0, 50.0, FilterFamily::Butterworth, 2);
        let mut out = 0.0_f32;
        for _ in 0..2000 {
            out = f.process(1.0);
        }
        assert!(libm::fabsf(out) < 0.01, "DC not rejected by HP, out = {}", out);
    }

    /// HP filter should pass Nyquist tone with gain ≈ 1.
    #[test]
    fn butterworth_hp_passes_nyquist() {
        let mut f = highpass(1000.0, 50.0, FilterFamily::Butterworth, 2);
        let mut out = 0.0_f32;
        for n in 0..5000_u32 {
            let x = if n % 2 == 0 { 1.0_f32 } else { -1.0_f32 };
            out = f.process(x);
        }
        assert!(libm::fabsf(out) > 0.95, "Nyquist not passed by HP, last out = {}", out);
    }

    /// Bessel LP DC gain should also be ≈ 1.0 after settling.
    #[test]
    fn bessel_lp_dc_gain_is_unity() {
        let mut f = lowpass(1000.0, 50.0, FilterFamily::Bessel, 2);
        let mut out = 0.0_f32;
        for _ in 0..2000 {
            out = f.process(1.0);
        }
        assert!(out > 0.99, "Bessel LP DC gain was {}", out);
    }

    /// Order-1 Butterworth LP should work and pass DC.
    #[test]
    fn order_1_works() {
        let mut f = lowpass(1000.0, 50.0, FilterFamily::Butterworth, 1);
        let mut out = 0.0_f32;
        for _ in 0..2000 {
            out = f.process(1.0);
        }
        assert!(out > 0.99, "Order-1 LP DC gain was {}", out);
    }

    /// Order-6 Butterworth LP should compile and produce a finite result.
    #[test]
    fn order_6_works() {
        let mut f = lowpass(1000.0, 50.0, FilterFamily::Butterworth, 6);
        let mut out = 0.0_f32;
        for _ in 0..2000 {
            out = f.process(1.0);
        }
        assert!(out.is_finite(), "Order-6 LP produced non-finite output");
        assert!(out > 0.99, "Order-6 LP DC gain was {}", out);
    }

    // ---- Bode plot verification (Example 1 filter: 2nd-order Butterworth LP, fc=20 Hz, fs=1000) ----
    //
    // Drives the filter with a pure sinusoid, measures steady-state amplitude
    // and phase via single-bin DFT correlation:
    //
    //   c_sin = (2/N) Σ y[n]·sin(θn)  →  A·cos(φ)
    //   c_cos = (2/N) Σ y[n]·cos(θn)  →  A·sin(φ)
    //   gain  = sqrt(c_sin² + c_cos²)
    //   phase = atan2(c_cos, c_sin)     [rad → deg]
    //
    // Expected values from the 2nd-order Butterworth analog prototype:
    //   |H(jΩ)|  = 1/√(1 + Ω⁴)         where Ω = f/fc
    //   ∠H(jΩ)  = −atan2(√2·Ω, 1−Ω²)
    //
    // | f (Hz) | Gain  | Gain (dB) | Phase  |
    // |--------|-------|-----------|--------|
    // |  10    | 0.971 |  −0.3 dB  |  −43°  |
    // |  20 ←fc| 0.707 |  −3.0 dB  |  −90°  |
    // |  40    | 0.243 | −12.3 dB  | −137°  |
    // | 100    | 0.040 | −28.0 dB  | −164°  |

    fn measure_response(
        mut filter: FilterChain<MAX_SECTIONS>,
        fs: f32,
        freq: f32,
    ) -> (f32, f32) {
        use core::f32::consts::PI;
        use libm::{atan2f, cosf, sinf};
        let theta = core::f32::consts::TAU * freq / fs;
        let period = (fs / freq).ceil() as usize;
        let n_settle = (period * 20).max(2000);
        let n_measure = period * 10;

        for n in 0..n_settle {
            filter.process(sinf(theta * n as f32));
        }

        let mut c_sin = 0.0f32;
        let mut c_cos = 0.0f32;
        for i in 0..n_measure {
            let n = n_settle + i;
            let y = filter.process(sinf(theta * n as f32));
            c_sin += y * sinf(theta * n as f32);
            c_cos += y * cosf(theta * n as f32);
        }
        let scale = 2.0 / n_measure as f32;
        c_sin *= scale;
        c_cos *= scale;

        let gain = libm::sqrtf(c_sin * c_sin + c_cos * c_cos);
        let phase_deg = atan2f(c_cos, c_sin) * 180.0 / PI;
        (gain, phase_deg)
    }

    #[test]
    fn butterworth_lp_bode_gain_sweep() {
        let fs = 1000.0_f32;
        let fc = 20.0_f32;

        // f = 10 Hz (Ω = 0.5): expected gain ≈ 0.971 (−0.26 dB)
        let (g, _) = measure_response(lowpass(fs, fc, FilterFamily::Butterworth, 2), fs, 10.0);
        assert!((g - 0.971).abs() < 0.01, "10 Hz gain: {g:.4}");

        // f = fc = 20 Hz (Ω = 1.0): expected gain = 1/√2 ≈ 0.707 (−3 dB)
        let (g, _) = measure_response(lowpass(fs, fc, FilterFamily::Butterworth, 2), fs, 20.0);
        assert!((g - 0.707).abs() < 0.01, "20 Hz gain: {g:.4}");

        // f = 40 Hz (Ω = 2.0): expected gain ≈ 0.243 (−12.3 dB)
        let (g, _) = measure_response(lowpass(fs, fc, FilterFamily::Butterworth, 2), fs, 40.0);
        assert!((g - 0.243).abs() < 0.01, "40 Hz gain: {g:.4}");

        // f = 100 Hz (Ω = 5.0): expected gain ≈ 0.040 (−28 dB)
        let (g, _) = measure_response(lowpass(fs, fc, FilterFamily::Butterworth, 2), fs, 100.0);
        assert!((g - 0.040).abs() < 0.005, "100 Hz gain: {g:.4}");
    }

    #[test]
    fn butterworth_lp_bode_phase_sweep() {
        let fs = 1000.0_f32;
        let fc = 20.0_f32;

        // f = 10 Hz (Ω = 0.5): expected phase ≈ −43°
        let (_, p) = measure_response(lowpass(fs, fc, FilterFamily::Butterworth, 2), fs, 10.0);
        assert!((p - (-43.3)).abs() < 3.0, "10 Hz phase: {p:.1}°");

        // f = fc = 20 Hz (Ω = 1.0): expected phase = −90° exactly
        let (_, p) = measure_response(lowpass(fs, fc, FilterFamily::Butterworth, 2), fs, 20.0);
        assert!((p - (-90.0)).abs() < 2.0, "20 Hz phase: {p:.1}°");

        // f = 40 Hz (Ω = 2.0): expected phase ≈ −137°
        let (_, p) = measure_response(lowpass(fs, fc, FilterFamily::Butterworth, 2), fs, 40.0);
        assert!((p - (-136.7)).abs() < 5.0, "40 Hz phase: {p:.1}°");
    }
}
