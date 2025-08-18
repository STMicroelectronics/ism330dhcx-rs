# Six-Direction Orientation Detection with ISM330DHCX on STM32F401RE Nucleo-64 using Embassy

This example demonstrates how to use the ISM330DHCX inertial sensor’s embedded Machine Learning Core (MLC) to detect six-direction orientation events (X, Y, Z axes pointing up or down) on an STM32F401RE Nucleo-64 board. It leverages the Embassy async executor for efficient asynchronous handling of I2C, UART, and GPIO interrupts.

The MLC configuration is generated at build time from a UCF (Universal Configuration File) created using STMicroelectronics’ Unico GUI tool, which abstracts the complex register programming required for the MLC.

---

## Hardware Setup

- **Microcontroller Board:** STM32F401RE Nucleo-64
- **Sensor:** ISM330DHCX 6-axis IMU (Accelerometer + Gyroscope)
- **Communication Interface:** I2C1 at 100 kHz Standard Mode
- **UART:** USART2 for serial output at 115200 baud
- **Interrupt Pin:** GPIO PB10 configured as external interrupt input

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

- The Embassy STM32 HAL initializes the microcontroller peripherals and clocks.
- USART2 is configured asynchronously for 115200 baud, 8 data bits, no parity.
- I2C1 is configured at 100 kHz with DMA and interrupt support.
- GPIO PB10 is configured as an external interrupt input pin without pull-up/down resistors.
- A heapless string buffer is used for UART message formatting.

### Sensor Configuration

- The ISM330DHCX sensor is initialized over I2C with the high I2C address.
- The sensor ID is read and verified to ensure proper communication.
- The sensor is reset to default configuration and the code waits for reset completion.
- The Machine Learning Core (MLC) is configured by writing a predefined UCF sequence (`SIX_D`) to the sensor registers. This configures the sensor to detect six orientation states.
- Interrupt routing is set to output MLC events on interrupt pin 1.
- Interrupt notification mode is configured to pulsed embedded latched mode.

### Event Handling Loop

- The main async loop waits for a rising edge on the interrupt pin asynchronously.
- When an MLC event is detected (MLC1 flag set), the event code is read from the sensor.
- The event code is converted into a human-readable six-direction event (e.g., "X-axis pointing up") and sent over UART.

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

- This example uses the Embassy async runtime for efficient, non-blocking peripheral handling.
- The MLC configuration is generated from a UCF file created with STMicroelectronics’ Unico GUI tool.
- The ISM330DHCX MLC supports up to 8 decision trees running simultaneously, with configurable features such as mean, variance, energy, peak-to-peak, zero-crossings, and more.
- The MLC output data rate and window length affect latency and accuracy.
- Interrupt-driven event detection avoids continuous polling, reducing CPU load.
- UART output provides a simple way to monitor sensor events for debugging or demonstration.

---

## References

- [STM32F401RE Nucleo-64 Board](https://www.st.com/en/evaluation-tools/nucleo-f401re.html)
- [ISM330DHCX Datasheet and Application Notes](https://www.st.com/en/mems-and-sensors/ism330dhcx.html)
- [AN5392 - ISM330DHCX Machine Learning Core Application Note](https://www.st.com/resource/en/application_note/an5392-ism330dhcx-machine-learning-core-stmicroelectronics.pdf)
- [Unico GUI Tool for Sensor Configuration](https://www.st.com/en/development-tools/unico.html)
- [Embassy STM32 HAL](https://github.com/embassy-rs/embassy)

---

*This README explains how to use the ISM330DHCX sensor’s Machine Learning Core on an STM32F401RE board with the Embassy async runtime to detect six-direction orientation events and output them via UART.*