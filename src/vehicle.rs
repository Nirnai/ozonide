use crate::board::{Board, BoardInterface};
use crate::drivers::{Imu, create_imu};

type Spi = <Board as BoardInterface>::SpiInterface;
type Pin = <Board as BoardInterface>::OutputPin;
type Int = <Board as BoardInterface>::InterruptPin;

pub struct Vehicle {
    pub _board: Board,
    pub _imu: Option<Imu<Spi, Pin, Int>>,
}

impl Vehicle {
    pub fn new(
        board: Board,
        imu: Option<Imu<Spi, Pin, Int>>,
    ) -> Self {
        Self { _board: board, _imu: imu }
    }

    pub fn from_config() -> Self {
        let mut board = Board::init();
        let mut imu = None;

        for sensor in crate::config::SENSORS {
            match sensor.role {
                "imu" => {
                    let spi = board.take_spi(sensor.interface).unwrap();
                    let cs = board.take_output_pin(sensor.pin("cs")).unwrap();
                    let int = board.take_interrupt_pin(sensor.pin("int1")).unwrap();
                    imu = Some(create_imu(sensor, spi, cs, int).unwrap());
                }
                unknown => defmt::warn!("Unknown sensor role: {}", unknown),
            }
        }
        return Vehicle::new(board, imu)
    }
}


