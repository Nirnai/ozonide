use crate::board::{Board, BoardInterface};
use crate::drivers::imu::AnyImu;

type Spi = <Board as BoardInterface>::SpiBus;
type Pin = <Board as BoardInterface>::OutputPin;

pub struct Vehicle {
    pub _board: Board,
    pub imu: Option<AnyImu<Spi, Pin>>,
    // pub baro: Option<AnyBaro<Spi, Pin>>,
}

impl Vehicle {
    pub fn new(
        board: Board,
        imu: Option<AnyImu<Spi, Pin>>,
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
                    imu = Some(crate::drivers::imu::create_imu(sensor, spi, cs).unwrap());
                }
                unknown => defmt::warn!("Unknown sensor role: {}", unknown),
            }
        }
        return Vehicle::new(board, imu)
    }
}


