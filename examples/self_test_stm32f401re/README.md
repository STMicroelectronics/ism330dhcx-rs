# ISM330DHCX Accelerometer and Gyroscope Self-Test on STM32F401RE Nucleo-64

This example demonstrates how to perform the built-in self-test procedure for both the accelerometer and gyroscope of the ISM330DHCX sensor using an STM32F401RE Nucleo-64 board. The self-test verifies sensor functionality by comparing sensor outputs with and without self-test excitation enabled, ensuring the sensor operates within expected ranges.

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

- The microcontroller peripherals and clocks are initialized, setting the system clock to use an 8 MHz external crystal.
- GPIO ports B and A are split to configure I2C pins (PB8, PB9) and UART TX pin (PA2).
- I2C1 is configured in standard mode at 100 kHz.
- USART2 is configured for 115200 baud, 8 data bits, no parity.

### Sensor Setup and Reset

- The ISM330DHCX sensor is initialized over I2C with the high I2C address.
- The sensor ID is read and verified to ensure proper communication.
- The sensor is reset to default configuration and the code waits for reset completion.

### Accelerometer Self-Test Procedure

- The accelerometer is configured with 52 Hz output data rate and ±4g full scale.
- The code waits for stable output by delaying 100 ms.
- It polls for new accelerometer data availability and reads 5 samples, averaging the raw acceleration values (converted to mg) with self-test disabled.
- The accelerometer self-test is enabled (positive or negative polarity selectable).
- After another 100 ms delay and polling for new data, 5 samples are read and averaged with self-test enabled.
- The absolute difference between self-test enabled and disabled averages is computed per axis.
- The differences are checked against the expected self-test range (90 to 1700 mg).
- The accelerometer self-test is disabled and the accelerometer output is turned off.

### Gyroscope Self-Test Procedure

- The gyroscope is configured with 208 Hz output data rate and ±2000 dps full scale.
- The code waits for stable output by delaying 100 ms.
- It polls for new gyroscope data availability and reads 5 samples, averaging the raw angular rate values (converted to mdps) with self-test disabled.
- The gyroscope self-test is enabled (positive or negative polarity selectable).
- After another 100 ms delay and polling for new data, 5 samples are read and averaged with self-test enabled.
- The absolute difference between self-test enabled and disabled averages is computed per axis.
- The differences are checked against the expected self-test range (150,000 to 700,000 mdps).
- The gyroscope self-test is disabled and the gyroscope output is turned off.

### Results Reporting

- The pass/fail results of accelerometer and gyroscope self-tests are printed over UART.

### Helper Functions

- `poll_new_xl_val_available` and `poll_new_gy_val_available` poll the sensor status registers until new accelerometer or gyroscope data is available, respectively.

---

## Usage

1. Connect the ISM330DHCX sensor to the STM32F401RE Nucleo board via I2C1 (PB8/SCL, PB9/SDA).
2. Build and flash the firmware onto the STM32F401RE board.
3. Open a serial terminal at 115200 baud on the USART2 TX line.
4. Observe the self-test results ("PASS" or "FAIL") for accelerometer and gyroscope printed once.
5. The program then enters an idle loop.

---

## Notes

- The self-test uses the sensor’s internal excitation to verify sensor functionality without external equipment.
- The example uses polling to wait for new sensor data, ensuring stable and valid measurements.
- Self-test polarity can be switched between positive and negative by changing the commented lines.
- The expected self-test ranges are based on the ISM330DHCX datasheet specifications.
- After self-test, the sensor outputs are disabled to save power.

---

## References

- [STM32F401RE Nucleo-64 Board](https://www.st.com/en/evaluation-tools/nucleo-f401re.html)
- [ISM330DHCX Datasheet](https://www.st.com/resource/en/datasheet/ism330dhcx.pdf)
- [stm32f4xx-hal Rust crate](https://docs.rs/stm32f4xx-hal)

---

*This README explains how to perform and verify the built-in self-test of the ISM330DHCX accelerometer and gyroscope on an STM32F401RE board and report the results via UART.*