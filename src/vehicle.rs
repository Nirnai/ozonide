use crate::board::{Board, BoardInterface};
use crate::drivers::{Imu, create_imu};

type Spi = <Board as BoardInterface>::SpiBus;
type Pin = <Board as BoardInterface>::OutputPin;

pub struct Vehicle {
    pub _board: Board,
    pub imu: Option<Imu<Spi, Pin>>,
}

impl Vehicle {
    pub fn new(
        board: Board,
        imu: Option<Imu<Spi, Pin>>,
    ) -> Self {
        Self { _board: board, imu }
    }

    pub fn from_config() -> Self {
        let mut board = Board::init();
        let mut imu = None;

        for sensor in crate::config::SENSORS {
            match sensor.role {
                "imu" => {
                    let spi = board.take_spi(sensor.interface).unwrap();
                    let cs = board.take_output_pin(sensor.pin("cs")).unwrap();
                    imu = Some(create_imu(sensor, spi, cs).unwrap());
                }
                unknown => defmt::warn!("Unknown sensor role: {}", unknown),
            }
        }
        return Vehicle::new(board, imu)
    }
}


