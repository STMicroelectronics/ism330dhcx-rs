# ISM330DHCX Sensor Hub Example with IIS2MDC Magnetometer (Embassy Async Runtime)

This example demonstrates how to use the STMicroelectronics **ISM330DHCX** 6-axis IMU as a sensor hub to interface with an external **IIS2MDC** magnetometer sensor via I²C, using the **Embassy async runtime** on an STM32 microcontroller. It configures the ISM330DHCX to read magnetometer data through its internal sensor hub and batch all sensor data (accelerometer, gyroscope, magnetometer, timestamp) into the FIFO buffer for efficient asynchronous retrieval and reporting via UART.

---

## Features Demonstrated

- **Sensor Hub Mode (Mode 2)**: ISM330DHCX acts as an I²C master to communicate with an external IIS2MDC sensor.
- **Asynchronous UART Output**: Uses Embassy’s async UART driver for non-blocking serial output.
- **Pass-Through Bus Abstraction**: Implements sensor hub read/write operations via a custom bus trait.
- **Continuous External Sensor Reading**: Configures sensor hub to continuously read magnetometer registers.
- **FIFO Buffer Usage**: Batches accelerometer, gyroscope, magnetometer, and timestamp data into FIFO.
- **FIFO Watermark and Interrupts**: Uses FIFO watermark flag to trigger asynchronous data processing.
- **Data Decoding and Conversion**: Parses FIFO tags and converts raw sensor data to physical units (mg, mdps, mG).
- **Power and Data Rate Configuration**: Sets output data rates (ODR) and full scales for sensors.
- **Block Data Update (BDU)**: Ensures data consistency during asynchronous reads.

---

## Hardware Requirements

- STM32 microcontroller supported by Embassy (e.g., STM32F401RE)
- ISM330DHCX 6-axis IMU connected via I²C
- IIS2MDC magnetometer connected to ISM330DHCX sensor hub I²C pins (SDx/SCx)
- UART interface (e.g., USART2) for debug output

---

## Key Concepts from ISM330DHCX Application Note (AN5398)

- **Sensor Hub Registers**: Configure external sensor addresses, registers, read/write operations, and batch enable via `SLVx_ADD`, `SLVx_SUBADD`, `SLVx_CONFIG`.
- **MASTER_CONFIG Register**: Enables sensor hub I²C master, selects trigger source (internal accelerometer/gyro DRDY or external INT2 pin), and configures pull-ups.
- **Sensor Hub Data Registers**: `SENSOR_HUB_1` to `SENSOR_HUB_18` hold data read from external sensors.
- **FIFO Buffer**: Stores sensor data tagged by sensor type, allowing efficient host reads. Supports batching of external sensor data.
- **FIFO Modes**: Continuous streaming mode used here to keep FIFO filled with latest sensor data.
- **FIFO Tags**: Identify sensor data type in FIFO (e.g., accelerometer, gyroscope, sensor hub slave 0).
- **Data Ready and Interrupts**: FIFO watermark interrupt used to notify host when enough data is available.

---

## Code Overview

### Sensor Hub Bus Abstraction

- Implements `BusOperation` trait for sensor hub pass-through.
- `write_bytes` configures sensor hub to write to IIS2MDC registers.
- `write_byte_read_bytes` configures sensor hub to read from IIS2MDC registers.
- Uses accelerometer ODR as trigger for sensor hub operations.
- Waits asynchronously for sensor hub operation completion via polling.

### Initialization and Configuration

- Initializes Embassy STM32 peripherals, including I2C1 and USART2 with DMA and interrupts.
- Creates `Ism330dhcxMaster` instance wrapping ISM330DHCX driver with async delay.
- Creates `Iis2mdc` instance using sensor hub pass-through bus.
- Verifies device IDs for ISM330DHCX and IIS2MDC.
- Resets and configures ISM330DHCX device.
- Configures IIS2MDC magnetometer for continuous mode at 20 Hz.
- Configures sensor hub to read 6 bytes from IIS2MDC output registers continuously at 26 Hz.
- Enables batching of accelerometer, gyroscope, magnetometer, and timestamp data into FIFO.
- Sets FIFO watermark to 15 samples and enables continuous (stream) FIFO mode.
- Enables latched interrupt notification for FIFO watermark.
- Sets accelerometer and gyroscope full scale and ODR to 2g and 2000 dps at 26 Hz.

### Main Async Loop

- Asynchronously polls FIFO watermark flag.
- When watermark reached, reads number of samples in FIFO.
- For each sample:
  - Reads FIFO tag to identify sensor data.
  - Decodes accelerometer and gyroscope raw data and converts to mg and mdps.
  - Reads sensor hub slave 0 data (magnetometer raw data) and converts to mG.
  - Reads timestamp data and converts to milliseconds.
  - Writes formatted sensor data with timestamp over UART asynchronously.

---

## Usage Notes

- Uses **internal trigger mode** for sensor hub, where accelerometer ODR triggers sensor hub reads.
- FIFO compression is not enabled; see AN5398 for compression details.
- The sensor hub pass-through bus does not implement `read_bytes` as it is not required here.
- Proper settling times and power mode transitions are handled as recommended in the application note.
- FIFO watermark interrupt is polled asynchronously; hardware interrupts can be integrated for efficiency.
- Assumes IIS2MDC magnetometer is connected and powered correctly on sensor hub I²C lines.

---

## References

- [Application Note AN5398](https://www.st.com/resource/en/application_note/an5398-ism330dhcx-alwayson-6axis-imu-inertial-measurement-unit-with-embedded-machine-learning-core-and-digital-output-for-industrial-applications-stmicroelectronics.pdf)
- [ISM330DHCX Datasheet](https://www.st.com/resource/en/datasheet/ism330dhcx.pdf)
- [IIS2MDC Datasheet](https://www.st.com/resource/en/datasheet/iis2mdc.pdf)

*This README explains how to use the sensor hub feature using ISM330DHCX as master and IIS2MDC as slave batching the data into a FIFO.*