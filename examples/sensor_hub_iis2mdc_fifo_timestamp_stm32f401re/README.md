# ISM330DHCX Sensor Hub Example with IIS2MDC Magnetometer

This example demonstrates how to use the STMicroelectronics **ISM330DHCX** 6-axis IMU as a sensor hub to interface with an external **IIS2MDC** magnetometer sensor via I²C. It configures the ISM330DHCX to read magnetometer data through its internal sensor hub and batch all sensor data (accelerometer, gyroscope, magnetometer, timestamp) into the FIFO buffer for efficient retrieval.

---

## Features Demonstrated

- **Sensor Hub Mode (Mode 2)**: Using ISM330DHCX as an I²C master to communicate with an external sensor (IIS2MDC).
- **Pass-Through Bus Implementation**: Custom bus abstraction to route sensor hub read/write operations transparently.
- **Continuous External Sensor Reading**: Configuring sensor hub to continuously read magnetometer registers.
- **FIFO Buffer Usage**: Batching accelerometer, gyroscope, magnetometer, and timestamp data into FIFO.
- **FIFO Watermark and Interrupts**: Using FIFO watermark to trigger data reads.
- **Data Decoding**: Parsing FIFO tags to identify sensor data type and converting raw data to physical units.
- **Power and Data Rate Configuration**: Setting output data rates (ODR) and full scales for sensors.
- **Block Data Update (BDU)**: Ensuring data consistency during reads.

---

## Hardware Requirements

- STM32F4 microcontroller (or compatible)
- ISM330DHCX 6-axis IMU connected via I²C
- IIS2MDC magnetometer connected to ISM330DHCX sensor hub I²C pins (SDx/SCx)
- UART interface for debug output

---

## Key Concepts from the ISM330DHCX Application Note (AN5398)

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
- Waits for sensor hub operation completion via `sh_status_get()`.

### Initialization and Configuration

- Initialize clocks, GPIO, I2C, UART, and delays.
- Create `Ism330dhcxMaster` instance wrapping ISM330DHCX driver.
- Create `Iis2mdc` instance using sensor hub pass-through bus.
- Verify device IDs for ISM330DHCX and IIS2MDC.
- Reset and configure ISM330DHCX device.
- Configure IIS2MDC magnetometer for continuous mode at 20 Hz.
- Configure sensor hub to read 6 bytes from IIS2MDC output registers continuously at 26 Hz.
- Enable batching of accelerometer, gyroscope, magnetometer, and timestamp data into FIFO.
- Set FIFO watermark to 15 samples and enable continuous (stream) FIFO mode.
- Enable latched interrupt notification for FIFO watermark.
- Set accelerometer and gyroscope full scale and ODR to 2g and 2000 dps at 26 Hz.

### Main Loop

- Poll FIFO watermark flag.
- When watermark reached, read number of samples in FIFO.
- For each sample:
  - Read FIFO tag to identify sensor data.
  - Decode accelerometer and gyroscope raw data and convert to mg and mdps.
  - Read sensor hub slave 0 data (magnetometer raw data) and convert to mG.
  - Read timestamp data and convert to milliseconds.
  - Print sensor data with timestamp over UART.

---

## Usage Notes

- The example uses **internal trigger mode** for sensor hub, where accelerometer ODR triggers sensor hub reads.
- FIFO compression is not enabled here; refer to AN5398 for compression details.
- The sensor hub pass-through bus does not implement `read_bytes` as it is not needed for this example.
- Proper settling times and power mode transitions are handled as recommended in the application note.
- FIFO watermark interrupt is polled in this example; hardware interrupt lines can be used for more efficient designs.
- The example assumes the IIS2MDC magnetometer is connected and powered correctly on the sensor hub I²C lines.

---

## References

- [Application Note AN5398](https://www.st.com/resource/en/application_note/an5398-ism330dhcx-alwayson-6axis-imu-inertial-measurement-unit-with-embedded-machine-learning-core-and-digital-output-for-industrial-applications-stmicroelectronics.pdf)
- [ISM330DHCX Datasheet](https://www.st.com/resource/en/datasheet/ism330dhcx.pdf)
- [IIS2MDC Datasheet](https://www.st.com/resource/en/datasheet/iis2mdc.pdf)

*This README explains how to use the sensor hub feature using ISM330DHCX as master and IIS2MDC as slave batching the data into a FIFO.*