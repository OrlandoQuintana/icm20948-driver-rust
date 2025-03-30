use icm20948_driver_rust::imu::{Accelerometer, Gyroscope, IMU};
use icm20948_driver_rust::spi_core::SpiCore;
use linux_embedded_hal::spidev::{Spidev, SpidevOptions, SpiModeFlags};
use linux_embedded_hal::SpidevBus;
use std::fs::File;
use std::io::Write;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut spidev = Spidev::open("/dev/spidev0.0")?;
    spidev.configure(
        &SpidevOptions::new()
            .bits_per_word(8)
            .max_speed_hz(7_000_000)
            .mode(SpiModeFlags::SPI_MODE_0)
            .build(),
    )?;

    let spidev_bus = SpidevBus::from(linux_embedded_hal::SpidevBus(spidev));
    let spi = Arc::new(Mutex::new(SpiCore::new(spidev_bus)));

    let mut imu = IMU::new(Arc::clone(&spi));
    imu.initialize()?;

    let mut accel = Accelerometer::new(Arc::clone(&spi));
    let mut gyro = Gyroscope::new(Arc::clone(&spi));

    let mut file = File::create("imu_log.csv")?;
    writeln!(file, "timestamp,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z")?;

    let start = Instant::now();
    let mut count = 0;

    while count < 100_000 {
        let timestamp = start.elapsed().as_secs_f64();

        let accel_data = accel.read()?;
        let gyro_data = gyro.read()?;

        writeln!(
            file,
            "{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6}",
            timestamp,
            gyro_data[0], gyro_data[1], gyro_data[2],
            accel_data[0], accel_data[1], accel_data[2]
        )?;
        println!("{}", count);
        count += 1;
    }

    println!("Done logging {} samples to imu_log.csv", count);
    Ok(())
}
