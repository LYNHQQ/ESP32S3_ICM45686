# ESP32-S3 ICM45686 Sensor Driver Example

This project demonstrates how to interface with and obtain data from an InvenSense ICM45686 Inertial Measurement Unit (IMU) sensor using an Espressif ESP32-S3 microcontroller. The project uses the ESP-IDF framework.

## Project Overview

The core functionality of this project is to initialize the ICM45686 sensor via SPI communication and then continuously read and print the accelerometer, gyroscope, and temperature data. The project also configures an external clock signal for the IMU. The project is structured to provide a clear example of integrating a custom sensor driver within the ESP-IDF framework.

## Hardware Requirements

*   ESP32-S3 based development board
*   ICM45686 IMU sensor

## Software Requirements

*   ESP-IDF framework (ensure you have the correct version compatible with ESP32-S3)
*   CMake
*   Ninja build tool (or other supported build tools)

## Pin Assignments

The project is configured to use the following pins on the ESP32-S3 to communicate with and provide necessary signals to the ICM45686 sensor:

*   **SPI MISO:** GPIO 13
*   **SPI MOSI:** GPIO 11
*   **SPI SCLK:** GPIO 12
*   **SPI CS:** GPIO 10
*   **IMU Interrupt (INT):** GPIO 14 (Configured as input with pull-up, rising edge trigger)
*   **IMU Clock Input (CLK_IN):** GPIO 9 (Configured as a 32 kHz PWM output using LEDC)

Please ensure your hardware connections match these assignments.

## Sensor Configuration

The project initializes the ICM45686 with the following basic configuration in `components/SPI_BUS/src/spi.c`:

*   **Power Mode:** Low Noise (LN) mode for both Accelerometer and Gyroscope.
*   **Accelerometer Full-Scale Range (FSR):** ±4g
*   **Gyroscope Full-Scale Range (FSR):** ±1000 dps
*   **Accelerometer Output Data Rate (ODR):** 200 Hz
*   **Gyroscope Output Data Rate (ODR):** 200 Hz
*   **Bandwidth:** ODR/4 for both Accelerometer and Gyroscope (in Low Noise mode).
*   **Interrupts:** Data Ready interrupt enabled on INT1 pin.

You can modify these parameters in the `setup_imu` function within `components/SPI_BUS/src/spi.c` if needed.

## Project Structure

The project includes the following main directories:

*   **main:** Contains the main application code (`main.c`) that initializes the SPI bus, sets up the IMU using the custom driver, configures the external clock signal, and enters a loop to read and print sensor data. It also includes an interrupt handler for the IMU data ready signal.
*   **components:** Contains custom components for the project.
    *   **SPI_BUS:** Component for managing the SPI communication interface.
        *   `include/spi.h`: Header file with SPI pin definitions and function declarations.
        *   `src/spi.c`: Implementation of the SPI driver and IMU setup/data reading functions.
    *   **ICM45686:** Custom component containing the driver for the ICM45686 sensor.
        *   `include/*.h`: Header files with sensor-specific definitions, register maps, and driver function declarations.
        *   `src/inv_imu_driver.c`, `src/inv_imu_transport.c`: Implementation of the core sensor driver logic.
    *   **MahonyAHRS** and **MadgwickAHRS:** Components likely containing implementations of AHRS algorithms, which could be integrated for attitude estimation.

## Building and Running

1.  **Set up ESP-IDF:** Follow the official Espressif documentation to set up the ESP-IDF environment for your operating system.
2.  **Navigate to project directory:** Open your terminal and change the current directory to the root of this project.
3.  **Build the project:** Run `idf.py build`. This will compile the application and the components.
4.  **Connect your board:** Connect your ESP32-S3 development board to your computer.
5.  **Flash and monitor:** Run `idf.py flash monitor`. This will flash the compiled firmware to the board and open a serial monitor to display the output.

The serial output will show the temperature readings and the time between IMU data ready interrupts (if `IMU_TICK` is defined), as implemented in `main.c`.

## Customization

*   **Modify Sensor Configuration:** Adjust the sensor parameters (FSR, ODR, power mode, etc.) by modifying the `setup_imu` function in `components/SPI_BUS/src/spi.c`.
*   **Process Sensor Data:** Modify the `app_main` function and `IMU_IRQ_process` task in `main/main.c` to process the accelerometer, gyroscope, and temperature data according to your application needs. You could implement filtering, sensor fusion (potentially using the provided AHRS components), or other algorithms.
*   **Modify Clock Signal:** Adjust the frequency or duty cycle of the clock signal by modifying the `clk_in_init` function in `main/main.c`.
*   **Explore Driver Functions:** Refer to the header files and source files in the `components/ICM45686` directory to explore more advanced features of the ICM45686 driver and integrate them into your project.

## Troubleshooting

*   **Hardware Connections:** Double-check the SPI, interrupt, and CLK_IN pin connections between the ESP32-S3 and the ICM45686 sensor.
*   **ESP-IDF Setup:** Ensure your ESP-IDF environment is correctly set up and the toolchain is accessible.
*   **Serial Port:** Verify that you have selected the correct serial port for flashing and monitoring.

## Contributing

If you wish to contribute to this project, please follow standard Git practices:

1.  Fork the repository.
2.  Create a new branch for your features or bug fixes.
3.  Make your changes.
4.  Submit a pull request.

## License

(Add license information here if applicable, based on the source files' licenses)

## Acknowledgments
