use embassy_time::Timer;
use ozonide_core::msgs::ImuData;
use ozonide_core::traits::ImuSource;

/// IMU source backed by UDP frames from the simulator (postcard-serialised [`ImuData`]).
///
/// `init()` binds the socket; `data_ready()` polls it non-blocking and yields to the
/// executor on `WouldBlock`, so no separate thread or channel is needed.
pub struct ImuSimulated {
    socket: Option<std::net::UdpSocket>,
    pending: Option<ImuData>,
}

impl ImuSimulated {
    pub fn new() -> Self {
        Self { socket: None, pending: None }
    }
}

impl ImuSource for ImuSimulated {
    async fn init(&mut self) {
        let socket = std::net::UdpSocket::bind("0.0.0.0:5005")
            .expect("ImuSimulated: bind UDP :5005");
        socket.set_nonblocking(true).expect("ImuSimulated: set_nonblocking");
        log::info!("ImuSimulated listening for ImuData on UDP :5005");
        self.socket = Some(socket);
    }

    async fn data_ready(&mut self) {
        let mut buf = [0u8; 64];
        loop {
            match self.socket.as_ref().unwrap().recv(&mut buf) {
                Ok(n) => {
                    if let Ok(data) = postcard::from_bytes::<ImuData>(&buf[..n]) {
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

    async fn read(&mut self) -> ImuData {
        self.pending.take().unwrap()
    }
}
