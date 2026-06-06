//! HTTP and WebSocket server for the browser-based simulation frontend.
//!
//! The server binds on port 9001 and provides:
//! - `GET /`  — serves the bundled Three.js frontend
//! - `GET /ws` — WebSocket endpoint; pushes [`SimulationState`] JSON at 60 Hz
//! - Control endpoints (`/play`, `/pause`, `/reset`, `/set-realtime`, `/set-fast`,
//!   `/disturbance/*`) that write to shared atomics read by the physics loop

mod server;

pub use server::SimulationState;
pub use server::serve;
