use core::f32::consts::TAU;
use heapless::Vec;
use libm::{sinf, cosf};

use super::biquadratic_filter::{BiquadraticFilter, BiquadraticFilterCoefficients};
use super::filter_chain::FilterChain;
use super::pole_placement::MAX_SECTIONS;

enum BandKind { Notch, Bandpass }

fn band(sample_rate: f32, f0: f32, bandwidth: f32, kind: BandKind) -> FilterChain<MAX_SECTIONS> {
    debug_assert!(f0 > 0.0 && f0 < sample_rate / 2.0, "f0 must be in (0, Nyquist)");
    debug_assert!(bandwidth > 0.0, "bandwidth must be positive");
    let q = f0 / bandwidth;
    let w0 = TAU * f0 / sample_rate;
    let alpha = sinf(w0) / (2.0 * q);
    let c = cosf(w0);
    let norm = 1.0 + alpha;
    let a1 = -2.0 * c / norm;
    let a2 = (1.0 - alpha) / norm;
    let coeffs = match kind {
        BandKind::Notch => BiquadraticFilterCoefficients::new(1.0 / norm, -2.0 * c / norm, 1.0 / norm, a1, a2),
        BandKind::Bandpass => BiquadraticFilterCoefficients::new(alpha / norm, 0.0, -alpha / norm, a1, a2),
    };
    let mut sections = Vec::new();
    let _ = sections.push(BiquadraticFilter::new(coeffs));
    FilterChain { sections }
}

pub fn notch(sample_rate: f32, f0: f32, bandwidth: f32) -> FilterChain<MAX_SECTIONS> {
    band(sample_rate, f0, bandwidth, BandKind::Notch)
}

pub fn bandpass(sample_rate: f32, f0: f32, bandwidth: f32) -> FilterChain<MAX_SECTIONS> {
    band(sample_rate, f0, bandwidth, BandKind::Bandpass)
}

#[cfg(test)]
mod tests {
    use super::*;
    use libm::{sinf, fabsf, sqrtf};

    /// A notch at 100 Hz should deeply attenuate a 100 Hz sine at 1 kHz sample rate.
    #[test]
    fn notch_nulls_at_center_frequency() {
        let mut f = notch(1000.0, 100.0, 20.0);
        let freq_fraction = 100.0_f32 / 1000.0_f32;
        let mut sum_sq = 0.0_f32;
        let n_total = 5000_u32;
        let n_measure = 200_u32;
        for n in 0..n_total {
            let x = sinf(TAU * freq_fraction * n as f32);
            let y = f.process(x);
            if n >= n_total - n_measure {
                sum_sq += y * y;
            }
        }
        let rms = sqrtf(sum_sq / n_measure as f32);
        assert!(rms < 0.05, "Notch RMS at center frequency was {} (expected < 0.05)", rms);
    }

    /// A notch filter should pass DC (gain ≈ 1).
    #[test]
    fn notch_passes_dc() {
        let mut f = notch(1000.0, 100.0, 20.0);
        let mut out = 0.0_f32;
        for _ in 0..2000 {
            out = f.process(1.0);
        }
        assert!(out > 0.98, "Notch DC gain was {} (expected > 0.98)", out);
    }

    /// A bandpass filter should reject DC (output ≈ 0 for constant input).
    #[test]
    fn bandpass_nulls_at_dc() {
        let mut f = bandpass(1000.0, 100.0, 20.0);
        let mut out = 0.0_f32;
        for _ in 0..2000 {
            out = f.process(1.0);
        }
        assert!(fabsf(out) < 0.02, "Bandpass DC output was {} (expected < 0.02)", out);
    }

    /// A bandpass filter should pass the center frequency with non-trivial amplitude.
    #[test]
    fn bandpass_passes_center() {
        let mut f = bandpass(1000.0, 100.0, 20.0);
        let freq_fraction = 100.0_f32 / 1000.0_f32;
        let mut out = 0.0_f32;
        let n_total = 5000_u32;
        for n in 0..n_total {
            let x = sinf(TAU * freq_fraction * n as f32);
            out = f.process(x);
        }
        assert!(fabsf(out) > 0.3, "Bandpass gain at center was {} (expected > 0.3)", fabsf(out));
    }

    // ---- Bode plot verification (Example 2 filter: notch, f0=100 Hz, bw=20 Hz, fs=1000) ----
    //
    // The RBJ notch is symmetric around f0. The −3 dB edges sit at f0 ± bw/2,
    // i.e., 90 Hz and 110 Hz (Q = f0/bw = 5). Key points:
    //
    // | f (Hz)      | Gain  | Gain (dB) |
    // |-------------|-------|-----------|
    // |   1 (DC~)   | 1.000 |   0.0 dB  |
    // |  90 (−3 dB) | 0.707 |  −3.0 dB  |
    // | 100 (null)  | 0.000 |    −∞ dB  |
    // | 110 (−3 dB) | 0.707 |  −3.0 dB  |
    // | 499 (Ny~)   | 1.000 |   0.0 dB  |

    fn measure_gain(
        mut filter: FilterChain<MAX_SECTIONS>,
        fs: f32,
        freq: f32,
    ) -> f32 {
        let theta = TAU * freq / fs;
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
        libm::sqrtf((c_sin * scale).powi(2) + (c_cos * scale).powi(2))
    }

    #[test]
    fn notch_bode_magnitude() {
        let fs = 1000.0_f32;
        let f0 = 100.0_f32;
        let bw = 20.0_f32;

        // f = 1 Hz: deep in the passband, gain ≈ 1.000
        let g = measure_gain(notch(fs, f0, bw), fs, 1.0);
        assert!((g - 1.0).abs() < 0.01, "1 Hz gain: {g:.4}");

        // f = 90 Hz: lower −3 dB edge, gain ≈ 0.707
        let g = measure_gain(notch(fs, f0, bw), fs, 90.0);
        assert!((g - 0.707).abs() < 0.03, "90 Hz gain: {g:.4}");

        // f = 100 Hz: null, gain ≈ 0
        let g = measure_gain(notch(fs, f0, bw), fs, 100.0);
        assert!(g < 0.05, "100 Hz gain: {g:.4}");

        // f = 110 Hz: upper −3 dB edge, gain ≈ 0.707
        let g = measure_gain(notch(fs, f0, bw), fs, 110.0);
        assert!((g - 0.707).abs() < 0.03, "110 Hz gain: {g:.4}");
    }
}
