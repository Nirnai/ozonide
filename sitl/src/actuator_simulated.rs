use std::net::UdpSocket;

use ozonide_core::msgs::ActuatorCommand;
use ozonide_core::traits::ActuatorSink;

pub struct ActuatorSimulated {
    socket: Option<UdpSocket>,
}

impl ActuatorSimulated {
    pub fn new() -> Self {
        Self { socket: None }
    }
}

impl ActuatorSink for ActuatorSimulated {
    async fn init(&mut self) {
        let socket =
            UdpSocket::bind("0.0.0.0:0").expect("ActuatorSimulated: Bind to any free UDP port");
        socket
            .connect("127.0.0.1:5006")
            .expect("ActuatorSimulated: sending to UDP 127.0.0.1:5006");
        log::info!("ActuatorSimulated sending actuator commands to UDP :5006");
        self.socket = Some(socket);
    }

    async fn write(&mut self, command: &ActuatorCommand) {
        let mut send_buffer = [0u8; 32];
        let encoded = postcard::to_slice(command, &mut send_buffer)
            .expect("ActuatorSimulated: serialize error");
        self.socket
            .as_ref()
            .unwrap()
            .send(encoded)
            .expect("ActuatorSimulated: send command!");
    }
}
