use embassy_time::Timer;
use ozonide_core::msgs::ActuatorTelemetry;
use ozonide_core::traits::ActuatorTelemetrySource;

/// [`ActuatorTelemetrySource`] backed by UDP frames from the simulator
/// (postcard-serialised [`ActuatorTelemetry`] on port 5007).
///
/// The simulator sends one frame per IMU tick (1 kHz), co-timed with
/// the physics step. `data_ready` polls non-blocking and yields to the
/// executor on `WouldBlock`.
pub struct ActuatorTelemetrySimulated {
    socket: Option<std::net::UdpSocket>,
    pending: Option<ActuatorTelemetry>,
}

impl ActuatorTelemetrySimulated {
    pub fn new() -> Self {
        Self { socket: None, pending: None }
    }
}

impl ActuatorTelemetrySource for ActuatorTelemetrySimulated {
    async fn init(&mut self) {
        let socket = std::net::UdpSocket::bind("0.0.0.0:5007")
            .expect("ActuatorTelemetrySimulated: bind UDP :5007");
        socket.set_nonblocking(true).expect("ActuatorTelemetrySimulated: set_nonblocking");
        log::info!("ActuatorTelemetrySimulated listening for ActuatorTelemetry on UDP :5007");
        self.socket = Some(socket);
    }

    async fn data_ready(&mut self) {
        let mut buf = [0u8; 64];
        loop {
            match self.socket.as_ref().unwrap().recv(&mut buf) {
                Ok(n) => {
                    if let Ok(data) = postcard::from_bytes::<ActuatorTelemetry>(&buf[..n]) {
                        self.pending = Some(data);
                        return;
                    }
                }
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                    Timer::after_micros(500).await;
                }
                _ => {}
            }
        }
    }

    async fn read(&mut self) -> ActuatorTelemetry {
        self.pending.take().unwrap()
    }
}
