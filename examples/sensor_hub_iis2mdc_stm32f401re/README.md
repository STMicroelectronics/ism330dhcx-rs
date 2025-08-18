# ISM330DHCX Sensor Hub Example with IIS2MDC Magnetometer (Polling Mode)

This example demonstrates how to use the STMicroelectronics **ISM330DHCX** 6-axis IMU as a sensor hub to interface with an external **IIS2MDC** magnetometer sensor via I²C. It configures the ISM330DHCX to read magnetometer data through its internal sensor hub and continuously poll accelerometer, gyroscope, and magnetometer data, outputting the results via UART.

---

## Features Demonstrated

- **Sensor Hub Mode (Mode 2)**: ISM330DHCX acts as an I²C master to communicate with an external IIS2MDC sensor.
- **Polling-Based Data Acquisition**: Continuously polls sensor status registers to read new data.
- **Pass-Through Bus Abstraction**: Implements sensor hub read/write operations via a custom bus trait.
- **Continuous External Sensor Reading**: Configures sensor hub to continuously read magnetometer registers.
- **Direct Sensor Data Reads**: Reads accelerometer, gyroscope, and magnetometer data on data-ready flags.
- **Power and Data Rate Configuration**: Sets output data rates (ODR) and full scales for sensors.
- **Simple UART Output**: Uses blocking UART writes to output sensor data.

---

## Hardware Requirements

- STM32F4 microcontroller (or compatible)
- ISM330DHCX 6-axis IMU connected via I²C
- IIS2MDC magnetometer connected to ISM330DHCX sensor hub I²C pins (SDx/SCx)
- UART interface (e.g., USART2) for debug output

---

## Key Concepts from ISM330DHCX Application Note (AN5398)

- **Sensor Hub Registers**: Configure external sensor addresses, registers, read/write operations, and batch enable via `SLVx_ADD`, `SLVx_SUBADD`, `SLVx_CONFIG`.
- **MASTER_CONFIG Register**: Enables sensor hub I²C master, selects trigger source (internal accelerometer/gyro DRDY or external INT2 pin), and configures pull-ups.
- **Sensor Hub Data Registers**: `SENSOR_HUB_1` to `SENSOR_HUB_18` hold data read from external sensors.
- **Data Ready Flags**: Use `STATUS_REG` bits `GDA` and `XLDA` to detect new gyroscope and accelerometer data.
- **Sensor Hub Operation**: Triggered by enabling accelerometer ODR; sensor hub reads external sensor data accordingly.
- **Data Format and Conversion**: Raw sensor data are converted to physical units (mg, mdps, mG) for output.

---

## Code Overview

### Sensor Hub Bus Abstraction

- Implements `BusOperation` trait for sensor hub pass-through.
- `write_bytes` configures sensor hub to write to IIS2MDC registers.
- `write_byte_read_bytes` configures sensor hub to read from IIS2MDC registers.
- Uses accelerometer ODR as trigger for sensor hub operations.
- Waits for sensor hub operation completion by polling status flags.

### Initialization and Configuration

- Initializes clocks, GPIO, I2C, UART, and delays using STM32F4 HAL.
- Creates `Ism330dhcxMaster` instance wrapping ISM330DHCX driver.
- Creates `Iis2mdc` instance using sensor hub pass-through bus.
- Verifies device IDs for ISM330DHCX and IIS2MDC.
- Resets and configures ISM330DHCX device.
- Turns off sensors before configuration.
- Configures IIS2MDC magnetometer for continuous mode at 20 Hz.
- Configures sensor hub to read 6 bytes from IIS2MDC output registers continuously at 26 Hz.
- Enables sensor hub with write-once mode.
- Sets accelerometer and gyroscope full scale and ODR to 2g and 2000 dps at 26 Hz.
- Turns on accelerometer and gyroscope sensors at 26 Hz ODR.

### Main Polling Loop

- Continuously polls the `STATUS_REG` register to check if new accelerometer (`XLDA`) and gyroscope (`GDA`) data are available.
- When new data is ready:
  - Reads and converts accelerometer raw data to mg.
  - Reads and converts gyroscope raw data to mdps.
  - Reads raw magnetometer data from sensor hub registers and converts to mG.
  - Outputs all sensor data via UART using blocking writes.

---

## Usage Notes

- This example uses **polling** to detect new sensor data rather than interrupts or FIFO.
- The sensor hub is triggered by enabling the accelerometer ODR; sensor hub reads external sensor data accordingly.
- The sensor hub pass-through bus does not implement `read_bytes` as it is not required here.
- Proper delays and power mode transitions are handled as recommended in the application note.
- Blocking UART writes are used for simplicity; for higher performance, consider interrupt or DMA-based UART.
- Assumes IIS2MDC magnetometer is connected and powered correctly on sensor hub I²C lines.

---

## References

- [Application Note AN5398](https://www.st.com/resource/en/application_note/an5398-ism330dhcx-alwayson-6axis-imu-inertial-measurement-unit-with-embedded-machine-learning-core-and-digital-output-for-industrial-applications-stmicroelectronics.pdf)
- [ISM330DHCX Datasheet](https://www.st.com/resource/en/datasheet/ism330dhcx.pdf)
- [IIS2MDC Datasheet](https://www.st.com/resource/en/datasheet/iis2mdc.pdf)

*This README explains how to use the sensor hub feature using ISM330DHCX as master and IIS2MDC as slave.*