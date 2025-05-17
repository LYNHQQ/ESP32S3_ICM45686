# ESP32-S3 ICM45686 Sensor Driver Example

This project demonstrates how to interface with and obtain data from an ICM45686 Inertial Measurement Unit (IMU) sensor using an Espressif ESP32-S3 microcontroller.

## Overview

The project initializes the ICM45686 sensor and continuously reads acceleration, gyroscope, and temperature data. The sensor communication and data processing are handled by a custom component.

## Hardware Requirements

*   ESP32-S3 based development board
*   ICM45686 IMU sensor

## Software Requirements

*   ESP-IDF framework
*   CMake
*   Ninja build tool (or other supported build tools)

## Project Structure

The project includes the following main components:

*   **main:** Contains the main application code (`main.c`) that initializes the IMU and reads data.
*   **components/ICM45686:** A custom component that provides the driver for the ICM45686 sensor.
    *   **include:** Header files for the sensor driver.
    *   **src:** Source files (`inv_imu_driver.c`, `inv_imu_transport.c`) containing the driver implementation.
*   **components/SPI_BUS:** (Likely) Contains code for managing the SPI communication with the sensor.
*   **components/MahonyAHRS** and **components/MadgwickAHRS:** (Likely) Contain implementations of Attitude and Heading Reference System (AHRS) algorithms, although they might not be directly used in the main application loop shown in `main.c`.

## Building and Running

1.  Set up the ESP-IDF environment. Refer to the official ESP-IDF documentation for detailed instructions.
2.  Navigate to the project root directory in your terminal.
3.  Run `idf.py build` to build the project.
4.  Connect your ESP32-S3 board.
5.  Run `idf.py flash monitor` to flash the build to the board and monitor the serial output.

The serial output will display the temperature readings from the ICM45686 sensor.

## Customization

*   Modify `main/main.c` to process or utilize the sensor data differently.
*   Explore the `components/ICM45686` directory to understand or modify the sensor driver implementation.
*   Integrate the Mahony or Madgwick AHRS algorithms if attitude and heading estimation is required.

## Troubleshooting

*   Ensure the ICM45686 sensor is correctly wired to the ESP32-S3 according to the project's SPI configuration (likely defined within the `components/SPI_BUS` or `components/ICM45686` source files).
*   Verify your ESP-IDF setup and toolchain are correctly configured.

## Contributing

If you wish to contribute to this project, please follow standard Git practices:

1.  Fork the repository.
2.  Create a new branch for your features or bug fixes.
3.  Make your changes.
4.  Submit a pull request.

## License

(Add license information here if applicable)

## Acknowledgments

(Add any acknowledgments here)