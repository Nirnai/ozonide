use cortex_m::interrupt::Mutex;
use core::cell::RefCell;

use crate::board::{Board, BoardInterface};
use crate::drivers::{Imu, create_imu};
use crate::utils::SystemTime;

type Spi = <Board as BoardInterface>::SpiInterface;
type Pin = <Board as BoardInterface>::OutputPin;
type Int = <Board as BoardInterface>::InterruptPin;
type ImuType = Imu<Spi, Pin, Int>;

static IMU: Mutex<RefCell<Option<ImuType>>> = Mutex::new(RefCell::new(None));

pub struct Vehicle {
    pub board: Board,
}

impl Vehicle {
    fn new(board: Board) -> Self {
        Self { board: board }
    }

    pub fn init() -> Self {
        SystemTime::init();
        let board = Board::init();
        Vehicle::new(board)
    }

    pub fn init_sensors(&mut self) {
        for sensor in crate::config::SENSORS {
            match sensor.role {
                "imu" => {
                    let spi = self.board.take_spi(sensor.interface).unwrap();
                    let cs = self.board.take_output_pin(sensor.pin("cs")).unwrap();
                    let int = self.board.take_interrupt_pin(sensor.pin("int1")).unwrap();
                    let imu = create_imu(sensor, spi, cs, int).unwrap();
                    Vehicle::set_imu(imu);
                }
                unknown => defmt::warn!("Unknown sensor role: {}", unknown),
            }
        }
    }

    pub fn read_imu() {
        Self::with_imu(|imu| {
            let timestamp_us = SystemTime::now();
            let mut data = imu.read();
            data.timestamp_us = timestamp_us as u64;
            defmt::info!("Timestamp: {}", data.timestamp_us);
        });
    }

    fn with_imu<R>(f: impl FnOnce(&mut ImuType) -> R) -> Option<R> {
        cortex_m::interrupt::free(|cs| {
            IMU.borrow(cs).borrow_mut().as_mut().map(f)
        })
    }

    fn set_imu(imu: ImuType) {
        cortex_m::interrupt::free(|cs| {
            IMU.borrow(cs).replace(Some(imu));
        });
    }
}
