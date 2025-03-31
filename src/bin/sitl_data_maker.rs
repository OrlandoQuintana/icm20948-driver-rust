use icm20948_driver_rust::imu::{Accelerometer, Gyroscope, IMU};
use icm20948_driver_rust::spi_core::SpiCore;
use linux_embedded_hal::spidev::{Spidev, SpidevOptions, SpiModeFlags};
use linux_embedded_hal::SpidevBus;
use std::fs::File;
use std::io::{Write, BufWriter};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use std::fmt::Write as FmtWrite; // For writing into String buffer

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Step 1: Configure SPI
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

    // Step 2: Initialize IMU and sensors
    let mut imu = IMU::new(Arc::clone(&spi));
    imu.initialize()?;

    let mut accel = Accelerometer::new(Arc::clone(&spi));
    let mut gyro = Gyroscope::new(Arc::clone(&spi));

    // Step 3: Preallocate string buffer
    let mut buffer = String::with_capacity(25_000_000); // ~25MB
    buffer.push_str("timestamp,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z\n");

    let start = Instant::now();
    let mut count = 0;

    // Step 4: Capture data
    while count < 1_000_000 {
        let timestamp = start.elapsed().as_secs_f64();
        let accel_data = accel.read()?;
        let gyro_data = gyro.read()?;

        write!(
            &mut buffer,
            "{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6}\n",
            timestamp,
            gyro_data[0], gyro_data[1], gyro_data[2],
            accel_data[0], accel_data[1], accel_data[2]
        )?;

        count += 1;
    }

    // Step 5: Write all at once to disk
    let file = File::create("imu_log.csv")?;
    let mut writer = BufWriter::new(file);
    writer.write_all(buffer.as_bytes())?;
    writer.flush()?;

    println!("Done logging {} samples to imu_log.csv", count);
    Ok(())
}
