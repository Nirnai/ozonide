mod imu_model;
mod physics;

use std::net::UdpSocket;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use imu_model::ImuModel;
use ozonide_core::msgs::{ActuatorCommand, ImuData};
use physics::{QuadParams, State};

/// Hover throttle: each motor at this level produces mass*g/4 thrust.
fn hover_throttle(p: &QuadParams) -> f32 {
    let hover_thrust = p.mass * 9.80665 / 4.0;
    (hover_thrust / p.k_thrust).sqrt() as f32
}

#[tokio::main]
async fn main() {
    println!("Ozonide simulator starting...");

    let params = QuadParams::default();
    let h = hover_throttle(&params);
    println!("Hover throttle: {:.3}", h);

    // Shared actuator command: simulator reads, SITL writes.
    let actuator: Arc<Mutex<ActuatorCommand>> = Arc::new(Mutex::new(ActuatorCommand {
        motor_throttle: [h; 4],
    }));

    // Actuator receiver task: listens on UDP 5006 for commands from SITL.
    let actuator_rx = Arc::clone(&actuator);
    tokio::spawn(async move {
        let sock = UdpSocket::bind("0.0.0.0:5006").expect("bind port 5006");
        println!("Simulator listening for actuator commands on UDP :5006");
        let mut buf = [0u8; 64];
        loop {
            if let Ok((n, _)) = sock.recv_from(&mut buf) {
                if let Ok(cmd) = postcard::from_bytes::<ActuatorCommand>(&buf[..n]) {
                    *actuator_rx.lock().unwrap() = cmd;
                }
            }
        }
    });

    // Physics + IMU loop: runs synchronously on its own thread.
    // Physics at 4 kHz, IMU samples sent at 1 kHz.
    let sitl_sock = UdpSocket::bind("0.0.0.0:0").expect("bind ephemeral port");
    sitl_sock.connect("127.0.0.1:5005").expect("connect to SITL");

    let actuator_phys = Arc::clone(&actuator);
    tokio::task::spawn_blocking(move || {
        let mut state = State::default();
        let mut model = ImuModel::new(imu_model::ImuNoise::default());
        let mut rng = rand::thread_rng();
        let p = QuadParams::default();

        const PHYS_HZ: u64 = 4000;
        const IMU_HZ: u64 = 1000;
        const PHYS_DT: f64 = 1.0 / PHYS_HZ as f64;
        const PHYS_STEPS_PER_IMU: u64 = PHYS_HZ / IMU_HZ;

        let mut sim_time_us: u64 = 0;
        let mut step_count: u64 = 0;
        let start = Instant::now();

        let mut buf = [0u8; 128];

        loop {
            let throttle = actuator_phys.lock().unwrap().motor_throttle;
            state = physics::step(&state, throttle, &p, PHYS_DT);
            sim_time_us += (PHYS_DT * 1e6) as u64;
            step_count += 1;

            // Emit IMU sample every PHYS_STEPS_PER_IMU physics steps.
            if step_count % PHYS_STEPS_PER_IMU == 0 {
                let sample: ImuData = model.measure(&state, sim_time_us, &mut rng);
                if let Ok(n) = postcard::to_slice(&sample, &mut buf) {
                    sitl_sock.send(n).ok();
                }
            }

            // Real-time pacing: sleep to maintain wall-clock alignment.
            let elapsed = start.elapsed();
            let expected = Duration::from_micros(sim_time_us);
            if expected > elapsed {
                std::thread::sleep(expected - elapsed);
            }
        }
    })
    .await
    .ok();
}
