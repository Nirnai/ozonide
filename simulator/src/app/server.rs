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
    /// Position in ENU world frame (metres).
    pub position: [f64; 3],
    /// Velocity in ENU world frame (m/s).
    pub velocity: [f64; 3],
    /// Unit quaternion [x, y, z, w]: body → world rotation.
    pub quaternion: [f64; 4],
}

#[derive(Clone)]
struct AppState {
    state_rx: watch::Receiver<SimulationState>,
    paused: Arc<AtomicBool>,
    reset_requested: Arc<AtomicBool>,
    realtime: Arc<AtomicBool>,
    disturbance_mode: Arc<AtomicU8>,
}

static FRONTEND: &str = include_str!("index.html");

pub async fn serve(
    rx: watch::Receiver<SimulationState>,
    paused: Arc<AtomicBool>,
    reset_requested: Arc<AtomicBool>,
    realtime: Arc<AtomicBool>,
    disturbance_mode: Arc<AtomicU8>,
) {
    let shared = AppState { state_rx: rx, paused, reset_requested, realtime, disturbance_mode };

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
