use embassy_time::Timer;
use ozonide_core::msgs::GroundTruthState;
use ozonide_core::topics::GROUND_TRUTH_STATE_TOPIC;

/// Receives ground-truth position and velocity from the simulator (postcard-serialised
/// [`GroundTruthState`] on UDP port 5008) and publishes to [`GROUND_TRUTH_STATE_TOPIC`].
///
/// Call [`run`](Self::run) once; it loops forever, one message per IMU tick.
pub struct GroundTruthSimulated {
    socket: Option<std::net::UdpSocket>,
}

impl GroundTruthSimulated {
    pub fn new() -> Self {
        Self { socket: None }
    }

    pub async fn run(&mut self) {
        let socket = std::net::UdpSocket::bind("0.0.0.0:5008")
            .expect("GroundTruthSimulated: bind UDP :5008");
        socket.set_nonblocking(true).expect("GroundTruthSimulated: set_nonblocking");
        log::info!("GroundTruthSimulated listening for GroundTruthState on UDP :5008");
        self.socket = Some(socket);

        let publisher = GROUND_TRUTH_STATE_TOPIC.publisher();
        let mut buf = [0u8; 64];

        loop {
            match self.socket.as_ref().unwrap().recv(&mut buf) {
                Ok(n) => {
                    if let Ok(data) = postcard::from_bytes::<GroundTruthState>(&buf[..n]) {
                        publisher.publish(data);
                    }
                }
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                    Timer::after_micros(500).await;
                }
                _ => {}
            }
        }
    }
}
