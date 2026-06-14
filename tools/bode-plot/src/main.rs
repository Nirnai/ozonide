use std::f32::consts::{PI, TAU};

use ozonide_core::filter::{Filter, FilterFamily, lowpass, notch};
use plotters::prelude::*;

const PLOTS_DIR: &str =
    concat!(env!("CARGO_MANIFEST_DIR"), "/../../ozonide-core/src/filter/plots");

fn main() {
    std::fs::create_dir_all(PLOTS_DIR).expect("create plots dir");

    let lp_path = format!("{PLOTS_DIR}/bode_lp.svg");
    bode(&lp_path,
        "2nd-order Butterworth LP — fc = 20 Hz, fs = 1 kHz",
        1000.0,
        || lowpass(1000.0, 20.0, FilterFamily::Butterworth, 2),
        Some(20.0),
        (-70.0, 5.0),
        (-200.0, 20.0),
    );

    let notch_path = format!("{PLOTS_DIR}/bode_notch.svg");
    bode(&notch_path,
        "Notch — f₀ = 100 Hz, BW = 20 Hz, fs = 1 kHz",
        1000.0,
        || notch(1000.0, 100.0, 20.0),
        Some(100.0),
        (-70.0, 5.0),
        (-200.0, 200.0),
    );

    println!("Written {lp_path}");
    println!("Written {notch_path}");
}

/// Sweep `filter_factory` over log-spaced frequencies and return
/// `(freq_hz, gain_db, phase_deg)` for each point.
fn sweep(
    filter_factory: impl Fn() -> Filter,
    fs: f32,
    n_points: usize,
) -> Vec<(f32, f32, f32)> {
    let f_min: f32 = 1.0;
    let f_max: f32 = fs / 2.0 * 0.99;
    let log_min = f_min.log10();
    let log_max = f_max.log10();

    (0..n_points)
        .map(|i| {
            let t = i as f32 / (n_points - 1) as f32;
            let freq = 10.0_f32.powf(log_min + t * (log_max - log_min));
            let (gain, phase) = measure_response(filter_factory(), fs, freq);
            let gain_db = if gain > 1e-10 {
                20.0 * gain.log10()
            } else {
                -200.0 // clip the null
            };
            (freq, gain_db, phase)
        })
        .collect()
}

/// Drive `filter` with a pure sinusoid at `freq` Hz, settle, then extract
/// steady-state gain and phase via single-bin DFT correlation:
///
///   c_sin = (2/N) Σ y[n]·sin(θn)  →  A·cos(φ)
///   c_cos = (2/N) Σ y[n]·cos(θn)  →  A·sin(φ)
///   gain  = sqrt(c_sin² + c_cos²)
///   phase = atan2(c_cos, c_sin)
fn measure_response(mut filter: Filter, fs: f32, freq: f32) -> (f32, f32) {
    let theta = TAU * freq / fs;
    let period = (fs / freq).ceil() as usize;
    let n_settle = (period * 20).max(2000);
    let n_measure = period * 10;

    for n in 0..n_settle {
        filter.process((theta * n as f32).sin());
    }

    let mut c_sin = 0.0_f32;
    let mut c_cos = 0.0_f32;
    for i in 0..n_measure {
        let n = n_settle + i;
        let angle = theta * n as f32;
        let y = filter.process(angle.sin());
        c_sin += y * angle.sin();
        c_cos += y * angle.cos();
    }
    let scale = 2.0 / n_measure as f32;
    c_sin *= scale;
    c_cos *= scale;

    let gain = (c_sin * c_sin + c_cos * c_cos).sqrt();
    let phase_deg = c_cos.atan2(c_sin) * 180.0 / PI;
    (gain, phase_deg)
}

/// Render a two-panel Bode plot (magnitude + phase) to an SVG file.
#[allow(clippy::too_many_arguments)]
fn bode(
    path: &str,
    title: &str,
    fs: f32,
    filter_factory: impl Fn() -> Filter,
    marker_hz: Option<f32>,
    mag_range: (f32, f32),   // (min_db, max_db)
    phase_range: (f32, f32), // (min_deg, max_deg)
) {
    let data = sweep(filter_factory, fs, 300);

    let f_min = 1.0_f32;
    let f_max = fs / 2.0 * 0.99;

    let root = SVGBackend::new(path, (900, 620)).into_drawing_area();
    root.fill(&WHITE).unwrap();

    let (top, bottom) = root.split_vertically(310);

    // ---- Magnitude panel ----
    let mut mag = ChartBuilder::on(&top)
        .caption(title, ("sans-serif", 15))
        .margin(10)
        .x_label_area_size(30)
        .y_label_area_size(55)
        .build_cartesian_2d(
            (f_min..f_max).log_scale(),
            mag_range.0..mag_range.1,
        )
        .unwrap();

    mag.configure_mesh()
        .y_desc("Gain (dB)")
        .x_labels(10)
        .y_labels(8)
        .light_line_style(RGBColor(220, 220, 220))
        .draw()
        .unwrap();

    // -3 dB reference line
    mag.draw_series(LineSeries::new(
        [(f_min, -3.0_f32), (f_max, -3.0_f32)],
        ShapeStyle { color: RGBColor(180, 180, 180).to_rgba(), filled: false, stroke_width: 1 },
    ))
    .unwrap()
    .label("-3 dB")
    .legend(|(x, y)| PathElement::new([(x, y), (x + 15, y)], RGBColor(180, 180, 180)));

    // Marker at fc / f0
    if let Some(f) = marker_hz {
        mag.draw_series(LineSeries::new(
            [(f, mag_range.0), (f, mag_range.1)],
            ShapeStyle { color: RGBColor(200, 100, 100).to_rgba(), filled: false, stroke_width: 1 },
        ))
        .unwrap();
    }

    // Magnitude curve
    mag.draw_series(LineSeries::new(
        data.iter().map(|&(f, g, _)| (f, g)),
        ShapeStyle { color: BLUE.to_rgba(), filled: false, stroke_width: 2 },
    ))
    .unwrap()
    .label("Magnitude")
    .legend(|(x, y)| PathElement::new([(x, y), (x + 15, y)], BLUE));

    mag.configure_series_labels()
        .border_style(BLACK)
        .background_style(WHITE.mix(0.8))
        .draw()
        .unwrap();

    // ---- Phase panel ----
    let mut ph = ChartBuilder::on(&bottom)
        .margin(10)
        .x_label_area_size(40)
        .y_label_area_size(55)
        .build_cartesian_2d(
            (f_min..f_max).log_scale(),
            phase_range.0..phase_range.1,
        )
        .unwrap();

    ph.configure_mesh()
        .x_desc("Frequency (Hz)")
        .y_desc("Phase (°)")
        .x_labels(10)
        .y_labels(8)
        .light_line_style(RGBColor(220, 220, 220))
        .draw()
        .unwrap();

    // 0° and -90° reference lines
    for &ref_deg in &[0.0_f32, -90.0, -180.0] {
        if ref_deg >= phase_range.0 && ref_deg <= phase_range.1 {
            ph.draw_series(LineSeries::new(
                [(f_min, ref_deg), (f_max, ref_deg)],
                ShapeStyle {
                    color: RGBColor(180, 180, 180).to_rgba(),
                    filled: false,
                    stroke_width: 1,
                },
            ))
            .unwrap();
        }
    }

    if let Some(f) = marker_hz {
        ph.draw_series(LineSeries::new(
            [(f, phase_range.0), (f, phase_range.1)],
            ShapeStyle { color: RGBColor(200, 100, 100).to_rgba(), filled: false, stroke_width: 1 },
        ))
        .unwrap();
    }

    ph.draw_series(LineSeries::new(
        data.iter().map(|&(f, _, p)| (f, p)),
        ShapeStyle { color: RED.to_rgba(), filled: false, stroke_width: 2 },
    ))
    .unwrap();

    root.present().unwrap();
}
