pub mod spi_core; // Connects to `src/spi_core.rs`
pub mod imu;      // Connects to `src/imu/mod.rs`

pub use crate::imu::{Accelerometer, Gyroscope, IMU};
pub use crate::spi_core::SpiCore;