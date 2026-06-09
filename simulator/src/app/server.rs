//! Axum HTTP + WebSocket server implementation.
//!
//! Serves the bundled frontend and manages a pool of WebSocket clients, each
//! receiving a 60 Hz JSON stream of [`SimulationState`]. Control signals from
//! the frontend are translated into atomic writes that the physics loop reads
//! on its next iteration, keeping the two loops fully decoupled.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use std::time::Duration;

use axum::{
    Router,
    extract::{State, ws::{Message, WebSocket, WebSocketUpgrade}},
    response::Html,
    routing::get,
};
use serde::Serialize;
use tokio::sync::watch;

/// Vehicle state broadcast to WebSocket clients at 60 Hz.
#[derive(Clone, Serialize)]
pub struct SimulationState {
    pub sim_time_us: u64,
    pub position: [f64; 3],
    pub velocity: [f64; 3],
    pub quaternion: [f64; 4],
    /// Truth body-frame angular velocity (rad/s) — ideal gyro reading with no noise.
    pub angular_velocity: [f64; 3],
    /// Truth body-frame specific force (m/s²) — ideal accelerometer reading with no noise.
    pub specific_force: [f64; 3],
    /// Noisy gyro output sent to the SITL.
    pub imu_gyro: [f32; 3],
    /// Noisy accelerometer output sent to the SITL.
    pub imu_accel: [f32; 3],
}

#[derive(Clone)]
struct AppState {
    state_rx: watch::Receiver<SimulationState>,
    paused: Arc<AtomicBool>,
    reset_requested: Arc<AtomicBool>,
    realtime: Arc<AtomicBool>,
    disturbance_mode: Arc<AtomicU8>,
    noise_white: Arc<AtomicBool>,
    noise_bias: Arc<AtomicBool>,
    noise_walk: Arc<AtomicBool>,
}

static FRONTEND: &str = include_str!("index.html");

/// Starts the HTTP + WebSocket server on `0.0.0.0:9001`.
///
/// Intended to be spawned as a Tokio task. Runs until the process exits.
/// Communication with the physics loop uses shared atomics:
/// - `paused` / `reset_requested` / `realtime` — boolean control flags
/// - `disturbance_mode` — `0` = Gaussian, `1` = OU wind, `2` = no disturbance
/// - `noise_white` / `noise_bias` / `noise_walk` — individual IMU noise component toggles
/// - `rx` — watch channel carrying the latest [`SimulationState`] from the physics loop
pub async fn serve(
    rx: watch::Receiver<SimulationState>,
    paused: Arc<AtomicBool>,
    reset_requested: Arc<AtomicBool>,
    realtime: Arc<AtomicBool>,
    disturbance_mode: Arc<AtomicU8>,
    noise_white: Arc<AtomicBool>,
    noise_bias: Arc<AtomicBool>,
    noise_walk: Arc<AtomicBool>,
) {
    let shared = AppState {
        state_rx: rx, paused, reset_requested, realtime, disturbance_mode,
        noise_white, noise_bias, noise_walk,
    };

    let app = Router::new()
        .route("/", get(|| async { Html(FRONTEND) }))
        .route("/ws", get(ws_handler))
        .route("/play", get(play_handler))
        .route("/pause", get(pause_handler))
        .route("/reset", get(reset_handler))
        .route("/set-realtime", get(set_realtime_handler))
        .route("/set-fast", get(set_fast_handler))
        .route("/disturbance/gaussian", get(disturbance_gaussian_handler))
        .route("/disturbance/ou", get(disturbance_ou_handler))
        .route("/disturbance/none", get(disturbance_none_handler))
        .route("/noise/on",         get(noise_all_on_handler))
        .route("/noise/off",        get(noise_all_off_handler))
        .route("/noise/white/on",   get(noise_white_on_handler))
        .route("/noise/white/off",  get(noise_white_off_handler))
        .route("/noise/bias/on",    get(noise_bias_on_handler))
        .route("/noise/bias/off",   get(noise_bias_off_handler))
        .route("/noise/walk/on",    get(noise_walk_on_handler))
        .route("/noise/walk/off",   get(noise_walk_off_handler))
        .with_state(shared);

    let listener = tokio::net::TcpListener::bind("0.0.0.0:9001").await.unwrap();
    println!("Frontend:        http://localhost:9001");
    println!("WebSocket:       ws://localhost:9001/ws");
    axum::serve(listener, app).await.unwrap();
}

async fn ws_handler(State(s): State<AppState>, ws: WebSocketUpgrade) -> impl axum::response::IntoResponse {
    ws.on_upgrade(move |socket| client_loop(socket, s.state_rx))
}

async fn play_handler(State(s): State<AppState>) {
    s.paused.store(false, Ordering::Relaxed);
}

async fn pause_handler(State(s): State<AppState>) {
    s.paused.store(true, Ordering::Relaxed);
}

async fn reset_handler(State(s): State<AppState>) {
    s.reset_requested.store(true, Ordering::Relaxed);
}

async fn set_realtime_handler(State(s): State<AppState>) {
    s.realtime.store(true, Ordering::Relaxed);
}

async fn set_fast_handler(State(s): State<AppState>) {
    s.realtime.store(false, Ordering::Relaxed);
}

async fn disturbance_gaussian_handler(State(s): State<AppState>) {
    s.disturbance_mode.store(0, Ordering::Relaxed);
}

async fn disturbance_ou_handler(State(s): State<AppState>) {
    s.disturbance_mode.store(1, Ordering::Relaxed);
}

async fn disturbance_none_handler(State(s): State<AppState>) {
    s.disturbance_mode.store(2, Ordering::Relaxed);
}

async fn noise_all_on_handler(State(s): State<AppState>) {
    s.noise_white.store(true, Ordering::Relaxed);
    s.noise_bias.store(true, Ordering::Relaxed);
    s.noise_walk.store(true, Ordering::Relaxed);
}

async fn noise_all_off_handler(State(s): State<AppState>) {
    s.noise_white.store(false, Ordering::Relaxed);
    s.noise_bias.store(false, Ordering::Relaxed);
    s.noise_walk.store(false, Ordering::Relaxed);
}

async fn noise_white_on_handler(State(s): State<AppState>)  { s.noise_white.store(true,  Ordering::Relaxed); }
async fn noise_white_off_handler(State(s): State<AppState>) { s.noise_white.store(false, Ordering::Relaxed); }
async fn noise_bias_on_handler(State(s): State<AppState>)   { s.noise_bias.store(true,   Ordering::Relaxed); }
async fn noise_bias_off_handler(State(s): State<AppState>)  { s.noise_bias.store(false,  Ordering::Relaxed); }
async fn noise_walk_on_handler(State(s): State<AppState>)   { s.noise_walk.store(true,   Ordering::Relaxed); }
async fn noise_walk_off_handler(State(s): State<AppState>)  { s.noise_walk.store(false,  Ordering::Relaxed); }

async fn client_loop(mut socket: WebSocket, rx: watch::Receiver<SimulationState>) {
    let mut ticker = tokio::time::interval(Duration::from_micros(16_667)); // 60 Hz
    loop {
        ticker.tick().await;
        let state = rx.borrow().clone();
        let json = serde_json::to_string(&state).unwrap();
        if socket.send(Message::Text(json)).await.is_err() {
            break;
        }
    }
}
