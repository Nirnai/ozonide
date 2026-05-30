use std::time::Duration;

use axum::{
    Router,
    extract::ws::{Message, WebSocket, WebSocketUpgrade},
    response::Html,
    routing::get,
};
use serde::Serialize;
use tokio::sync::watch;

/// Vehicle state broadcast to WebSocket clients at 60 Hz.
#[derive(Clone, Serialize)]
pub struct SimState {
    pub sim_time_us: u64,
    /// Position in ENU world frame (metres).
    pub position: [f64; 3],
    /// Velocity in ENU world frame (m/s).
    pub velocity: [f64; 3],
    /// Unit quaternion [w, x, y, z]: body → world rotation.
    pub quaternion: [f64; 4],
}

static FRONTEND: &str = include_str!("index.html");

pub async fn serve(rx: watch::Receiver<SimState>) {
    let app = Router::new()
        .route("/", get(|| async { Html(FRONTEND) }))
        .route(
            "/ws",
            get(move |ws: WebSocketUpgrade| {
                let rx = rx.clone();
                async move { ws.on_upgrade(move |socket| client_loop(socket, rx)) }
            }),
        );

    let listener = tokio::net::TcpListener::bind("0.0.0.0:9001").await.unwrap();
    println!("Frontend:        http://localhost:9001");
    println!("WebSocket:       ws://localhost:9001/ws");
    axum::serve(listener, app).await.unwrap();
}

async fn client_loop(mut socket: WebSocket, rx: watch::Receiver<SimState>) {
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
