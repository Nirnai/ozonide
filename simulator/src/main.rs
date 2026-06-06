mod app;
mod models;
mod physics;

use std::net::UdpSocket;
use std::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use models::DisturbanceType;
use ozonide_core::msgs::{ActuatorCommand, ImuData};

/// Returns `(throttle_norm, omega_hover_rad_s)` for level hover.
fn hover_throttle(mass: f64, actuator_model: &models::ActuatorModel) -> (f32, f64) {
    let thrust_per_motor = mass * physics::GRAVITATIONAL_CONSTANT / 4.0;
    let omega_hover = (thrust_per_motor / actuator_model.k_t).sqrt();
    (
        (omega_hover / actuator_model.omega_max).clamp(0.0, 1.0) as f32,
        omega_hover,
    )
}

fn make_initial_state(omega_hover: f64) -> physics::VehicleState {
    physics::VehicleState {
        motor_angular_velocity: nalgebra::Vector4::from_element(omega_hover),
        ..Default::default()
    }
}

#[tokio::main]
async fn main() {
    println!("Ozonide simulator starting...");

    let vehicle_parameters = physics::VehicleParameters::default();
    let actuator_model_main = models::ActuatorModel::default();
    let (throttle_hover, omega_hover) =
        hover_throttle(vehicle_parameters.mass, &actuator_model_main);

    // Shared actuator command: simulator reads, SITL writes.
    let actuator_commands: Arc<Mutex<ActuatorCommand>> = Arc::new(Mutex::new(ActuatorCommand {
        motor_throttle: [throttle_hover; 4],
    }));

    // Simulation control flags — starts paused, full speed.
    let paused = Arc::new(AtomicBool::new(true));
    let reset_requested = Arc::new(AtomicBool::new(false));
    let realtime = Arc::new(AtomicBool::new(false));
    let disturbance_mode = Arc::new(AtomicU8::new(0));

    // Watch channel: physics loop writes SimState, WebSocket clients read it.
    let initial_state = make_initial_state(omega_hover);
    let (state_tx, state_rx) = tokio::sync::watch::channel(app::SimulationState {
        sim_time_us: 0,
        position: initial_state.position.into(),
        velocity: initial_state.linear_velocity.into(),
        quaternion: initial_state.attitude.coords.into(),
    });

    // Actuator UDP receiver.
    let actuator_commands_rx = Arc::clone(&actuator_commands);
    tokio::spawn(async move {
        let sock = UdpSocket::bind("0.0.0.0:5006").expect("bind port 5006");
        println!("Simulator listening for actuator commands on UDP :5006");
        let mut buf = [0u8; 64];
        loop {
            if let Ok((n, _)) = sock.recv_from(&mut buf) {
                if let Ok(cmd) = postcard::from_bytes::<ActuatorCommand>(&buf[..n]) {
                    *actuator_commands_rx.lock().unwrap() = cmd;
                }
            }
        }
    });

    // WebSocket + control HTTP server.
    tokio::spawn(app::serve(
        state_rx,
        Arc::clone(&paused),
        Arc::clone(&reset_requested),
        Arc::clone(&realtime),
        Arc::clone(&disturbance_mode),
    ));

    // Physics + IMU loop on a dedicated OS thread.
    let sitl_sock = UdpSocket::bind("0.0.0.0:0").expect("bind ephemeral port");
    sitl_sock
        .connect("127.0.0.1:5005")
        .expect("connect to SITL");

    let actuator_physics = Arc::clone(&actuator_commands);
    tokio::task::spawn_blocking(move || {
        let mut rng = rand::rng();
        let mut state = make_initial_state(omega_hover);
        let mut model = models::ImuModel::new(models::ImuNoise::default(), &mut rng);
        let mut disturbance_model =
            models::DisturbanceModel::new(models::DisturbanceType::NoDisturbance);
        let parameters = physics::VehicleParameters::default();
        let actuator_model = models::ActuatorModel::default();

        const PHYS_HZ: u64 = 4000;
        const IMU_HZ: u64 = 1000;
        const PHYS_DT: f64 = 1.0 / PHYS_HZ as f64;
        const PHYS_STEPS_PER_IMU: u64 = PHYS_HZ / IMU_HZ;

        let mut sim_time_us: u64 = 0;
        let mut step_count: u64 = 0;
        let mut wall_start = Instant::now();
        // Tracks whether we were paused on the previous iteration so we can
        // re-anchor wall_start on unpause and avoid a catch-up burst.
        let mut was_paused = true;
        let mut buf = [0u8; 128];

        loop {
            // Reset is checked before the paused gate so it applies immediately
            // even while paused, and the frontend gets the updated position.
            if reset_requested.swap(false, Ordering::Relaxed) {
                state = make_initial_state(omega_hover);
                disturbance_model.force = nalgebra::Vector3::zeros();
                disturbance_model.torque = nalgebra::Vector3::zeros();
                sim_time_us = 0;
                step_count = 0;
                wall_start = Instant::now();
                was_paused = paused.load(Ordering::Relaxed);
                model = models::ImuModel::new(models::ImuNoise::default(), &mut rng);
                state_tx
                    .send(app::SimulationState {
                        sim_time_us: 0,
                        position: state.position.into(),
                        velocity: state.linear_velocity.into(),
                        quaternion: state.attitude.coords.into(),
                    })
                    .ok();
            }

            if paused.load(Ordering::Relaxed) {
                was_paused = true;
                std::thread::sleep(Duration::from_millis(10));
                continue;
            }

            // Unpause transition: re-anchor wall_start so the pacing logic
            // doesn't see a large accumulated lag and rush through physics.
            if was_paused {
                wall_start = Instant::now() - Duration::from_micros(sim_time_us);
                was_paused = false;
            }

            disturbance_model.disturbance_type = match disturbance_mode.load(Ordering::Relaxed) {
                1 => DisturbanceType::OrnsteinUhlenbeck,
                2 => DisturbanceType::NoDisturbance,
                _ => DisturbanceType::Gaussian,
            };

            let throttle_raw = actuator_physics.lock().unwrap().motor_throttle;
            let throttle = nalgebra::Vector4::new(
                throttle_raw[0] as f64,
                throttle_raw[1] as f64,
                throttle_raw[2] as f64,
                throttle_raw[3] as f64,
            );
            let (new_state, state_dot) = physics::step(
                throttle,
                &state,
                &parameters,
                &actuator_model,
                &mut disturbance_model,
                &mut rng,
                PHYS_DT,
            );
            state = new_state;
            sim_time_us += (PHYS_DT * 1e6) as u64;
            step_count += 1;

            if step_count % PHYS_STEPS_PER_IMU == 0 {
                let sample: ImuData = model.measure(&state, &state_dot, sim_time_us, &mut rng);
                if let Ok(n) = postcard::to_slice(&sample, &mut buf) {
                    sitl_sock.send(n).ok();
                }

                state_tx
                    .send(app::SimulationState {
                        sim_time_us,
                        position: state.position.into(),
                        velocity: state.linear_velocity.into(),
                        quaternion: state.attitude.coords.into(),
                    })
                    .ok();
            }

            // Real-time pacing: only sleep when the flag is set.
            if realtime.load(Ordering::Relaxed) {
                let elapsed = wall_start.elapsed();
                let expected = Duration::from_micros(sim_time_us);
                if expected > elapsed {
                    std::thread::sleep(expected - elapsed);
                }
            }
        }
    })
    .await
    .ok();
}
