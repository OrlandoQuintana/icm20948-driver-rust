# **ICM-20948 Driver**

## **Overview**
This project provides a complete Rust driver for the **ICM-20948 IMU** connected to a **Raspberry Pi** via the SPI communication protocol. It supports initialization of the IMU and provides abstractions for reading data from the accelerometer and gyroscope.

The driver is modular, portable, and thread-safe, leveraging the **embedded-hal** and **linux-embedded-hal** crates for abstraction and compatibility across platforms.

---
## How to Use `icm20948-driver-rust`

The `icm20948-driver-rust` library can be used in your Rust projects to to read acclerometer and gyro data from an ICM-20948 IMU using a raspberry pi. Follow the steps below to integrate the library into your project.

### Adding `icm20948-driver-rust` as a Dependency

You can include `icm20948-driver-rust` as a dependency in your `Cargo.toml` by referencing the GitHub repository. Add the following lines to your `Cargo.toml`:

	[dependencies]
	icm20948-driver-rust = { git = "https://github.com/OrlandoQuintana/icm20948-driver-rust" }

This tells Cargo to pull the library directly from the GitHub repository and use it in your project.

----------

### Importing `icm20948-driver-rust` into Your Code

To use the library in your Rust code, import the `SpiCore`, `Accelerometer`, `Gyroscope` and `IMU` structs at the top of your file. 

Example:

	use icm20948_driver_rust::imu::{Accelerometer, Gyroscope, IMU};
	use icm20948_driver_rust::spi_core::SpiCore;

This makes the structs available for use in your code.

----------

### Using the `icm20948-driver-rust` Code Locally

If you prefer to clone the `icm20948-driver-rust` repository and use it as a local dependency, follow these steps:

1.  Clone the repository to your local machine:

		git clone https://github.com/OrlandoQuintana/icm20948-driver-rust.git
2. Place the cloned repository in a desired location on your computer.
3. Add the local path to your `Cargo.toml` dependencies:

		[dependencies]
		icm20948-driver-rust = { path = "../path/to/icm20948-driver-rust" }
	Replace `../path/to/icm20948-driver-rust` with the actual relative path to the `icm20948-driver-rust` folder.



## **Hardware Requirements**
### **Required Components**
- **ICM-20948 IMU**
  - A 9-axis inertial measurement unit featuring:
    - Accelerometer (3-axis).
    - Gyroscope (3-axis).
    - Magnetometer (3-axis, future support planned).
- **Raspberry Pi**
  - Any Linux-capable Raspberry Pi model with SPI support (e.g., Raspberry Pi 4, Raspberry Pi Zero W).

---

## **Wiring**
The ICM-20948 IMU connects to the Raspberry Pi as follows:

| **Raspberry Pi Pin** | **ICM-20948 Pin** | **Function**            |
|-----------------------|-------------------|-------------------------|
| GPIO 10 (MOSI)        | SDI              | Master Out, Slave In    |
| GPIO 9 (MISO)         | SDO              | Master In, Slave Out    |
| GPIO 11 (SCLK)        | SCL              | Serial Clock            |
| GPIO 8 (CE0)          | CS               | Chip Select             |
| GND                   | GND              | Ground                  |
| 3.3V                  | VDD              | Power Supply (3.3V)     |

To enable SPI on the Raspberry Pi:
```bash
sudo raspi-config
```
Navigate to **Interfacing Options → SPI** and enable it.

Verify SPI devices are available:
```bash
ls /dev/spidev*
```

---

## **Crates Used**
### **Key Dependencies**
1. **`embedded-hal`**:
   - A hardware abstraction layer for embedded systems.
   - Defines traits for peripherals like SPI, I²C, GPIO, etc.

2. **`linux-embedded-hal`**:
   - Implements the `embedded-hal` traits for Linux systems, enabling SPI communication through `spidev`.

3. **`std::sync::{Arc, Mutex}`**:
   - Ensures thread-safe shared access to the SPI bus between the accelerometer and gyroscope.

Install the necessary crates in your `Cargo.toml`:
```toml
[dependencies]
embedded-hal = "1.0.0"
linux-embedded-hal = "0.4"
```use rclrs::{create_node, Context, Node, Publisher, RclrsError, QOS_PROFILE_DEFAULT};
use sensor_msgs::msg::Imu as ImuMsg;
use icm20948_driver_rust::imu::{Accelerometer, Gyroscope, IMU}; // Custom Rust driver, https://github.com/OrlandoQuintana/icm20948-driver-rust
use icm20948_driver_rust::spi_core::SpiCore;
use linux_embedded_hal::spidev::{Spidev, SpidevOptions, SpiModeFlags}; // Wraps embedded-hal code used in the driver for use on Linux
use linux_embedded_hal::SpidevBus;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use biquad::{Biquad, Coefficients, DirectForm1, ToHertz}; // Crate for Butterworth low pass filter



/// Struct containing the ROS2 node, publisher, IMU components. a filter parameters
struct IMUPublisherNode {
    node: Arc<Node>,
    publisher: Arc<Publisher<ImuMsg>>,
    imu: IMU<SpidevBus>,
    accel: Accelerometer<SpidevBus>,
    gyro: Gyroscope<SpidevBus>,
    filter_x: Mutex<DirectForm1<f32>>,
    filter_y: Mutex<DirectForm1<f32>>,
    filter_z: Mutex<DirectForm1<f32>>,
}

impl IMUPublisherNode {
    /// Create a new IMU Publisher Node
    fn new(context: &Context) -> Result<Self, RclrsError> {
        
        // Standard ROS2 Rust node and publisher initialization
        let node = create_node(context, "imu_publisher").unwrap();
        let publisher = node
            .create_publisher::<ImuMsg>("/raw_imu", QOS_PROFILE_DEFAULT)
            .unwrap();

        // Configure Butterworth filter coefficients
        let coeffs = Coefficients::<f32>::from_params(
            biquad::Type::LowPass,
            200.0.hz(), // Sampling frequency (adjust as per your IMU's rate)
            15.0.hz(),   // Cutoff frequency
            0.707,      // Q factor (Butterworth characteristic)
        )
        .unwrap();

        // Create filters for each axis
        let filter_x = Mutex::new(DirectForm1::<f32>::new(coeffs));
        let filter_y = Mutex::new(DirectForm1::<f32>::new(coeffs));
        let filter_z = Mutex::new(DirectForm1::<f32>::new(coeffs));

        // Configure IMU
        let mut spidev = Spidev::open("/dev/spidev0.0").expect("Failed to open SPI device");
        spidev
            .configure(
                &SpidevOptions::new()
                    .bits_per_word(8)
                    .max_speed_hz(1_000_000)
                    .mode(SpiModeFlags::SPI_MODE_0)
                    .build(),
            )
            .expect("Failed to configure SPI device");

        let spidev_bus = SpidevBus::from(linux_embedded_hal::SpidevBus(spidev));
        let spi = Arc::new(Mutex::new(SpiCore::new(spidev_bus)));
        let imu = IMU::new(Arc::clone(&spi));
        let accel = Accelerometer::new(Arc::clone(&spi));
        let gyro = Gyroscope::new(Arc::clone(&spi));

        Ok(Self {
            node: Arc::clone(&node),
            publisher: Arc::clone(&publisher),
            imu,
            accel,
            gyro,
            filter_x,
            filter_y,
            filter_z,
        })
    }

    /// Initialize the IMU explicitly
    fn initialize_imu(&mut self) -> Result<(), String> {
        self.imu.initialize().map_err(|e| format!("IMU initialization failed: {:?}", e))?;
        Ok(())
    }

    /// Publish IMU data to the ROS2 topic
    fn publish_data(&mut self) -> Result<(), RclrsError> {
        let mut imu_msg = ImuMsg::default();

        // Read accelerometer data using read method from the driver
        if let Ok(accel_data) = self.accel.read() {
            // Filter accelerometer data
            let filtered_x = self.filter_x.lock().unwrap().run(accel_data[0] as f32);
            let filtered_y = self.filter_y.lock().unwrap().run(accel_data[1] as f32);
            let filtered_z = self.filter_z.lock().unwrap().run(accel_data[2] as f32);

            imu_msg.linear_acceleration.x = filtered_x as f64;
            imu_msg.linear_acceleration.y = filtered_y as f64;
            imu_msg.linear_acceleration.z = filtered_z as f64;

            println!(
                "Accel -> x: {:.3}, y: {:.3}, z: {:.3}",
                imu_msg.linear_acceleration.x,
                imu_msg.linear_acceleration.y,
                imu_msg.linear_acceleration.z
            );
        } else {
            println!("publish_data: Failed to read accelerometer");
        }

        // Read gyroscope data
        if let Ok(gyro_data) = self.gyro.read() {

            // Gyroscope Calibration
            // Run the gyroscope for X iterations with the gyroscope completely at rest. take the average
            // reading and subtract from all gyroscope readings to get a calibrated reading
            imu_msg.angular_velocity.x = (gyro_data[0] - (-0.0007)) as f64; // X Bias
            imu_msg.angular_velocity.y = (gyro_data[1] - (0.013)) as f64; // Y Bias
            imu_msg.angular_velocity.z = (gyro_data[2] - (0.006)) as f64; // Z Bias

            println!(
                "Gyro -> x: {:.3}, y: {:.3}, z: {:.3}",
                imu_msg.angular_velocity.x,
                imu_msg.angular_velocity.y,
                imu_msg.angular_velocity.z
            );
        } else {
            println!("publish_data: Failed to read gyroscope");
        }

        // Publish the message
        self.publisher.publish(imu_msg).unwrap();

        Ok(())
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let context = Context::new(std::env::args()).unwrap();
    let mut publisher_node = IMUPublisherNode::new(&context).unwrap();

    // Initialize the IMU explicitly
    if let Err(err) = publisher_node.initialize_imu() {
        eprintln!("Failed to initialize IMU: {}", err);
        return Err(Box::new(std::io::Error::new(std::io::ErrorKind::Other, err)));
    }

    let node_handle = publisher_node.node.clone(); // Clone the ROS2 node for spinning
    let publisher_node = Arc::new(Mutex::new(publisher_node));
    let publisher_node_thread = Arc::clone(&publisher_node);

    // Spawn a thread for publishing data
    std::thread::spawn(move || {
        let mut last_time = std::time::Instant::now();
        loop {
            // Calculate elapsed time since the last loop
            let elapsed = last_time.elapsed();
            let elapsed_ms = elapsed.as_millis() as u64;

            // Check if we've exceeded 2 ms
            if elapsed_ms >= 5 {
                // Update the timestamp for the next cycle
                last_time = std::time::Instant::now();

                // Call the publish data method
                if let Ok(mut node) = publisher_node_thread.lock() {
                    if let Err(err) = node.publish_data() {
                        eprintln!("Error publishing IMU data: {:?}", err);
                    }
                } else {
                    eprintln!("Failed to lock publisher node.");
                }
            } else {
                // Sleep for the remaining time in the 2 ms window
                std::thread::sleep(Duration::from_micros((5000 - elapsed.as_micros() as u64) as u64));
            }
        }
    });

    // Spin the node to process callbacks
    rclrs::spin(node_handle)?; // Only spin the node, without locking the publisher logic

    Ok(())
}


---

## **Driver Architecture**
The driver consists of the following layers:

1. **Hardware Layer**:
   - SPI hardware on the Raspberry Pi communicates with the ICM-20948.

2. **SPI Core Abstraction (`SpiCore`)**:
   - A reusable interface for register-based SPI communication.

3. **IMU Initialization Layer (`IMU`)**:
   - Handles hardware initialization (e.g., power management, filters, sensitivity).

4. **Sensor-Specific Abstraction**:
   - **`Accelerometer`**:
     - Reads and processes linear acceleration data (m/s²).
   - **`Gyroscope`**:
     - Reads and processes angular velocity data (radians/second).

---

## **Code Structure**
```plaintext
src/
├── main.rs              # Example usage
├── spi_core.rs          # SPI core abstraction
├── imu/                 # IMU module
│   ├── mod.rs           # IMU initialization and sensor management
│   ├── accelerometer.rs # Accelerometer logic
│   └── gyroscope.rs     # Gyroscope logic
```

---

## **Example Usage**
Here’s an example `main.rs` that demonstrates how to use the driver:

```rust
use icm20948_driver_rust::imu::{Accelerometer, Gyroscope, IMU};
use icm20948_driver_rust::spi_core::SpiCore;
use linux_embedded_hal::spidev::{Spidev, SpidevOptions, SpiModeFlags};
use linux_embedded_hal::SpidevBus;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Step 1: Create and configure the SPI device
    let mut spidev = Spidev::open("/dev/spidev0.0").expect("Failed to open SPI device");
    spidev
        .configure(
            &SpidevOptions::new()
                .bits_per_word(8)
                .max_speed_hz(1_000_000) // 1 MHz
                .mode(SpiModeFlags::SPI_MODE_0)
                .build(),
        )
        .expect("Failed to configure SPI device");

    // Step 2: Wrap Spidev in a SpidevBus
    let spidev_bus = SpidevBus::from(linux_embedded_hal::SpidevBus(spidev));

    // Step 3: Create an Arc<Mutex<SpiCore>> to share SPI access
    let spi = Arc::new(Mutex::new(SpiCore::new(spidev_bus)));

    // Step 4: Create and initialize the central IMU
    let mut imu = IMU::new(Arc::clone(&spi));
    imu.initialize().expect("Failed to initialize IMU");

    // Step 5: Create accelerometer, gyroscope
    let mut accel = Accelerometer::new(Arc::clone(&spi));
    let mut gyro = Gyroscope::new(Arc::clone(&spi));

    // Step 6: Main loop to read data
    let mut _loop_count = 0; // Loop counter

    loop {
        let loop_start = Instant::now();

        // Accelerometer Logic
        match accel.read() {
            Ok(accel_data) => println!(
                "Accelerometer - X: {:.2}, Y: {:.2}, Z: {:.2}",
                accel_data[0], accel_data[1], accel_data[2]
            ),
            Err(e) => eprintln!("Failed to read accelerometer data: {:?}", e),
        }
    
        // Gyroscope Logic
        match gyro.read() {
            Ok(gyro_data) => println!(
                "Gyroscope - X: {:.2}, Y: {:.2}, Z: {:.2}",
                gyro_data[0], gyro_data[1], gyro_data[2]
            ),
            Err(e) => eprintln!("Failed to read gyroscope data: {:?}", e),
        }
    
        // Maintain 2.5ms loop duration
        let elapsed = loop_start.elapsed();
        if elapsed < Duration::from_micros(2500) {
            thread::sleep(Duration::from_micros(2500) - elapsed);
        }
    
        _loop_count += 1;
    }
                     
}


```

---

## **Features**
- **Portability**: 
  - Built on `embedded-hal`, the driver works with any platform that supports the standard HAL traits including other Linux platforms (Nvidia Jetson series computers) or compatible microcontrollers (STM32, Arduino, etc.)
- **Thread-Safe**:
  - Shared SPI access is protected using `Arc<Mutex<SpiCore>>`. SPI communication is not inherently threadsafe so this driver utilizes Rust's borrow checker to guarantee only 1 abstraction accesses the SPI bus at a time.
- **Modular Design**:
  - Separate abstractions for SPI communication, initialization, and individual sensors.
- **Scalable**:
  - Future enhancements could include support for the ICM-20948’s magnetometer and sensor fusion algorithms.

---

## **Future Enhancements**
1. Add support for the ICM-20948 magnetometer.
2. Implement configuration options for sensitivity and filter settings.
3. Introduce advanced features like sensor fusion for calculating roll, pitch, and yaw.

---