# Six-Direction Orientation Detection with ISM330DHCX on STM32F401RE Nucleo-64

This example demonstrates how to configure and use the ISM330DHCX inertial sensor’s embedded Machine Learning Core (MLC) to detect six-direction orientation events (X, Y, Z axes pointing up or down) on an STM32F401RE Nucleo-64 board. The sensor is interfaced via I2C, and detected orientation events are output over UART in real-time.

The MLC configuration is generated at build time from a UCF (Universal Configuration File) created using STMicroelectronics’ Unico GUI tool, which abstracts the complex register programming required for the MLC.

---

## Hardware Setup

- **Microcontroller Board:** STM32F401RE Nucleo-64
- **Sensor:** ISM330DHCX 6-axis IMU (Accelerometer + Gyroscope)
- **Communication Interface:** I2C1 at 100 kHz Standard Mode
- **UART:** USART2 for serial output at 115200 baud

### Default Pin Configuration

| Signal       | STM32F401RE Pin | Description                      |
|--------------|-----------------|---------------------------------|
| I2C1_SCL     | PB8             | I2C clock line (open-drain)     |
| I2C1_SDA     | PB9             | I2C data line (open-drain)      |
| USART2_TX    | PA2             | UART transmit for debug output  |
| INT Pin      | PB10            | Sensor interrupt input (rising edge triggered) |

The ISM330DHCX sensor is connected to the STM32F401RE via I2C1 on pins PB8 (SCL) and PB9 (SDA). The sensor interrupt pin is connected to PB10, configured to trigger on rising edges. UART output is routed through PA2 for real-time event logging.

---

## Code Description

### Initialization

- The microcontroller peripherals and clocks are initialized, setting the system clock to use an 8 MHz external crystal.
- GPIO ports B and A are split to configure I2C pins (PB8, PB9), UART TX pin (PA2), and the interrupt input pin (PB10).
- I2C1 is configured in standard mode at 100 kHz.
- USART2 is configured for 115200 baud, 8 data bits, no parity.
- The interrupt pin PB10 is configured as input with interrupt on rising edge, and NVIC interrupt is enabled.

### Sensor Configuration

- The ISM330DHCX sensor is initialized over I2C with the high I2C address.
- The sensor ID is read and verified to ensure proper communication.
- The sensor is reset to default configuration and the code waits for reset completion.
- The Machine Learning Core (MLC) is configured by writing a predefined UCF sequence (`SIX_D`) to the sensor registers. This configures the sensor to detect six orientation states.
- Interrupt routing is set to output MLC events on interrupt pin 1.
- Interrupt notification mode is configured to pulsed embedded latched mode.

### Event Handling Loop

- The main loop waits for interrupts using the `wfi` (Wait For Interrupt) instruction.
- When an MLC event is detected (MLC1 flag set), the event code is read from the sensor.
- The event code is converted into a human-readable six-direction event (e.g., "X-axis pointing up") and printed over UART.

### Interrupt Service Routine

- The EXTI15_10 interrupt handler clears the interrupt pending bit on the sensor interrupt pin to allow further interrupts.

---

## Usage

1. Connect the ISM330DHCX sensor to the STM32F401RE Nucleo board via I2C1 (PB8/SCL, PB9/SDA).
2. Connect the sensor interrupt output to PB10 on the STM32 board.
3. Connect UART TX (PA2) to a serial terminal or USB-to-serial adapter.
4. Build and flash the firmware onto the STM32F401RE board.
5. Open a serial terminal at 115200 baud.
6. Move or orient the sensor to trigger six-direction events.
7. Observe the detected orientation events printed in the terminal in real-time.

---

## Notes

- The example uses the ISM330DHCX’s embedded Machine Learning Core (MLC) to offload orientation detection from the MCU, reducing power consumption.
- The MLC configuration is generated from a UCF file created with STMicroelectronics’ Unico GUI tool, which allows configuring filters, features, decision trees, and meta-classifiers.
- The ISM330DHCX MLC supports up to 8 decision trees running simultaneously, with configurable features such as mean, variance, energy, peak-to-peak, zero-crossings, and more.
- The MLC output data rate and window length affect latency and accuracy.
- Interrupts are used to efficiently detect events without continuous polling.
- UART output provides a simple way to monitor sensor events for debugging or demonstration.

---

## References

- [STM32F401RE Nucleo-64 Board](https://www.st.com/en/evaluation-tools/nucleo-f401re.html)
- [ISM330DHCX Datasheet and Application Notes](https://www.st.com/en/mems-and-sensors/ism330dhcx.html)
- [AN5392 - ISM330DHCX Machine Learning Core Application Note](https://www.st.com/resource/en/application_note/an5392-ism330dhcx-machine-learning-core-stmicroelectronics.pdf)
- [Unico GUI Tool for Sensor Configuration](https://www.st.com/en/development-tools/unico.html)
- [stm32f4xx-hal Rust crate Documentation](https://docs.rs/stm32f4xx-hal)

---

*This README explains how to use the ISM330DHCX sensor’s Machine Learning Core on an STM32F401RE board to detect six-direction orientation events and output them via UART.*