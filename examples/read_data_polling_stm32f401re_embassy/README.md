# Basic ISM330DHCX Sensor Data Polling on STM32F401RE Nucleo-64 with Embassy Async Runtime

This example demonstrates how to initialize and configure the ISM330DHCX 6-axis inertial sensor (accelerometer + gyroscope) on an STM32F401RE Nucleo-64 board using the Embassy async runtime. It continuously polls the sensor for acceleration, angular rate, and temperature data and outputs the readings asynchronously over UART.

---

## Hardware Setup

- **Microcontroller Board:** STM32F401RE Nucleo-64
- **Sensor:** ISM330DHCX 6-axis IMU (Accelerometer + Gyroscope)
- **Communication Interface:** I2C1 at 100 kHz Standard Mode
- **UART:** USART2 for serial output at 115200 baud

### Default Pin Configuration

| Signal       | STM32F401RE Pin | Description                    |
|--------------|-----------------|-------------------------------|
| I2C1_SCL     | PB8             | I2C clock line (open-drain)   |
| I2C1_SDA     | PB9             | I2C data line (open-drain)    |
| USART2_TX    | PA2             | UART transmit for debug output|

The ISM330DHCX sensor is connected to the STM32F401RE via I2C1 on pins PB8 (SCL) and PB9 (SDA). UART output is routed through PA2.

---

## Code Description

### Initialization

- The Embassy STM32 HAL initializes the microcontroller peripherals and clocks.
- USART2 is configured asynchronously for 115200 baud, 8 data bits, no parity.
- I2C1 is configured at 100 kHz with DMA and interrupt support.
- A heapless string buffer is used for UART message formatting.
- A short delay is introduced to allow sensor startup.

### Sensor Configuration

- The ISM330DHCX sensor is initialized over I2C with the high I2C address.
- The sensor ID is read and verified to ensure proper communication.
- The sensor is reset to default configuration and the code waits for reset completion.
- Device configuration is restored.
- Block Data Update (BDU) is enabled to ensure data consistency during reads.
- Output Data Rates (ODR) for accelerometer and gyroscope are set to 12.5 Hz.
- Full scale ranges are set to ±2g for accelerometer and ±2000 dps for gyroscope.
- The accelerometer filtering chain is configured to use LPF1 + LPF2 path with a high-pass filter enabled.

### Data Polling Loop

- The main async loop continuously polls the sensor status flags for new data availability.
- When new accelerometer data is ready, raw data is read, converted to mg units, formatted, and sent asynchronously over UART.
- When new gyroscope data is ready, raw data is read, converted to mdps units, formatted, and sent asynchronously over UART.
- When new temperature data is ready, raw data is read, converted to degrees Celsius, formatted, and sent asynchronously over UART.

---

## Usage

1. Connect the ISM330DHCX sensor to the STM32F401RE Nucleo board via I2C1 (PB8/SCL, PB9/SDA).
2. Build and flash the firmware onto the STM32F401RE board.
3. Open a serial terminal at 115200 baud on the USART2 TX line.
4. Observe acceleration, angular rate, and temperature readings printed continuously.

---

## Notes

- This example uses polling to read sensor data asynchronously, which is simple but may be less power efficient than interrupt-driven approaches.
- Block Data Update (BDU) is enabled to prevent reading partially updated sensor data.
- Output Data Rates and full scale settings can be adjusted to suit application requirements.
- The filtering chain configuration improves accelerometer signal quality by applying low-pass and high-pass filters.

---

## References

- [STM32F401RE Nucleo-64 Board](https://www.st.com/en/evaluation-tools/nucleo-f401re.html)
- [ISM330DHCX Datasheet](https://www.st.com/resource/en/datasheet/ism330dhcx.pdf)
- [Embassy STM32 HAL](https://github.com/embassy-rs/embassy)

---

*This README explains how to initialize and poll the ISM330DHCX sensor on an STM32F401RE board using the Embassy async runtime and output sensor data via UART.*