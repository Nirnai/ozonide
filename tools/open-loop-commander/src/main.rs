/// Open-loop motor commander for validating the control allocation matrix.
///
/// Bypasses the SITL entirely: accepts keyboard inputs, computes motor throttles
/// through `allocate_control`, and sends `ActuatorCommand` directly to the
/// simulator on UDP :5006. The simulator is set to noise-off and played on
/// startup so no manual browser interaction is required.
///
/// Key bindings:
///   ↑ / ↓         — thrust up / down
///   A / D         — roll left (−) / right (+)
///   W / S         — pitch forward (+) / backward (−)
///   Q / E         — yaw CCW (+) / CW (−)
///   Space         — zero all torques (keep thrust)
///   R             — reset to hover (thrust=0.4976, torques=0)
///   Esc           — quit
use std::io::{Write, stdout};
use std::net::UdpSocket;
use std::time::{Duration, Instant};

use crossterm::{
    cursor::{Hide, MoveTo, Show},
    event::{self, Event, KeyCode},
    execute,
    terminal::{Clear, ClearType, disable_raw_mode, enable_raw_mode},
};
use ozonide_core::control::{ControlDemand, allocate_control};
use ozonide_core::msgs::ActuatorCommand;

/// Hover throttle derived from vehicle physical parameters.
const HOVER_THRUST: f32 = 0.4976;
const THRUST_STEP: f32 = 0.001;
const TORQUE_STEP: f32 = 0.001;
const LOOP_PERIOD: Duration = Duration::from_millis(20); // 50 Hz

fn main() {
    // Tell the simulator to disable noise and start playing.
    simulator_get("/noise/off");
    simulator_get("/play");
    simulator_get("/set-realtime");

    let sock = UdpSocket::bind("0.0.0.0:0").expect("bind ephemeral UDP port");
    sock.connect("127.0.0.1:5006").expect("connect to simulator :5006");

    enable_raw_mode().expect("enable raw mode");
    execute!(stdout(), Hide, Clear(ClearType::All)).ok();

    let mut demand = ControlDemand {
        thrust: HOVER_THRUST,
        roll_torque: 0.0,
        pitch_torque: 0.0,
        yaw_torque: 0.0,
    };

    'main: loop {
        let tick_start = Instant::now();

        // Drain all pending keyboard events before the next send.
        while event::poll(Duration::ZERO).unwrap_or(false) {
            if let Ok(Event::Key(key)) = event::read() {
                match key.code {
                    KeyCode::Up => {
                        demand.thrust = (demand.thrust + THRUST_STEP).min(1.0);
                    }
                    KeyCode::Down => {
                        demand.thrust = (demand.thrust - THRUST_STEP).max(0.0);
                    }
                    KeyCode::Char('a') | KeyCode::Char('A') => {
                        demand.roll_torque = (demand.roll_torque - TORQUE_STEP).max(-1.0);
                    }
                    KeyCode::Char('d') | KeyCode::Char('D') => {
                        demand.roll_torque = (demand.roll_torque + TORQUE_STEP).min(1.0);
                    }
                    KeyCode::Char('w') | KeyCode::Char('W') => {
                        demand.pitch_torque = (demand.pitch_torque + TORQUE_STEP).min(1.0);
                    }
                    KeyCode::Char('s') | KeyCode::Char('S') => {
                        demand.pitch_torque = (demand.pitch_torque - TORQUE_STEP).max(-1.0);
                    }
                    KeyCode::Char('q') | KeyCode::Char('Q') => {
                        demand.yaw_torque = (demand.yaw_torque + TORQUE_STEP).min(1.0);
                    }
                    KeyCode::Char('e') | KeyCode::Char('E') => {
                        demand.yaw_torque = (demand.yaw_torque - TORQUE_STEP).max(-1.0);
                    }
                    KeyCode::Char(' ') => {
                        demand.roll_torque = 0.0;
                        demand.pitch_torque = 0.0;
                        demand.yaw_torque = 0.0;
                    }
                    KeyCode::Char('r') | KeyCode::Char('R') => {
                        demand = ControlDemand {
                            thrust: HOVER_THRUST,
                            roll_torque: 0.0,
                            pitch_torque: 0.0,
                            yaw_torque: 0.0,
                        };
                    }
                    KeyCode::Esc => break 'main,
                    _ => {}
                }
            }
        }

        // Allocate and send.
        let cmd = allocate_control(&demand);
        send_actuator(&sock, &cmd);

        render(&demand, &cmd);

        // Pace to 50 Hz.
        let elapsed = tick_start.elapsed();
        if elapsed < LOOP_PERIOD {
            std::thread::sleep(LOOP_PERIOD - elapsed);
        }
    }

    // Send all-zero throttle before exiting so the simulator coasts gracefully.
    send_actuator(&sock, &ActuatorCommand { motor_throttle: [0.0; 4] });
    simulator_get("/pause");

    disable_raw_mode().ok();
    execute!(stdout(), Show, MoveTo(0, 12)).ok();
    println!("\nExited.");
}

fn send_actuator(sock: &UdpSocket, cmd: &ActuatorCommand) {
    let mut buf = [0u8; 64];
    if let Ok(encoded) = postcard::to_slice(cmd, &mut buf) {
        sock.send(encoded).ok();
    }
}

fn render(demand: &ControlDemand, cmd: &ActuatorCommand) {
    let mut out = stdout();
    execute!(out, MoveTo(0, 0)).ok();

    // In raw mode every line needs \r\n — plain \n only moves the cursor down.
    let lines: &[&str] = &[
        "=== Ozonide Open-Loop Commander ===  [noise=OFF]",
        "",
        "Demand:",
        &format!("  Thrust:       {:+.4}   [↑ / ↓]  ", demand.thrust),
        &format!("  Roll torque:  {:+.4}   [A / D]  ", demand.roll_torque),
        &format!("  Pitch torque: {:+.4}   [W / S]  ", demand.pitch_torque),
        &format!("  Yaw torque:   {:+.4}   [Q / E]  ", demand.yaw_torque),
        "",
        "Motor throttles:",
        &format!("  FR [0]: {:.4}   RL [1]: {:.4}  ", cmd.motor_throttle[0], cmd.motor_throttle[1]),
        &format!("  FL [2]: {:.4}   RR [3]: {:.4}  ", cmd.motor_throttle[2], cmd.motor_throttle[3]),
        "",
        "[Space] zero torques   [R] reset to hover   [Esc] quit",
    ];

    for line in lines {
        write!(out, "{:<60}\r\n", line).ok();
    }
    out.flush().ok();
}

/// Fire-and-forget HTTP GET to the simulator. Errors are silently ignored —
/// the simulator may not be running yet and that is OK.
fn simulator_get(path: &str) {
    let url = format!("http://localhost:9001{}", path);
    ureq::get(&url).call().ok();
}
