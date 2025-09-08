# ISM330DHCX Free Fall Detection Using Embedded Finite State Machine on STM32F401RE Nucleo-64 with Embassy Framework

This example demonstrates how to detect free fall events using the **ISM330DHCX** inertial sensor's embedded Finite State Machine (FSM) on an **STM32F401RE** microcontroller board. The sensor is configured via I2C, and detected events are reported over UART. The application leverages the **Embassy** asynchronous embedded framework to efficiently manage peripherals and concurrency.

---

## Hardware Setup

- **Microcontroller Board:** STM32F401RE Nucleo-64
- **Sensor:** ISM330DHCX 6-axis IMU (accelerometer + gyroscope)
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

- The program initializes microcontroller peripherals including clocks, GPIO pins, I2C, UART, and a delay abstraction using the **Embassy** asynchronous framework.
- I2C1 is configured at 100 kHz Standard Mode on pins PB8 (SCL) and PB9 (SDA) with interrupt and DMA support.
- UART is configured on USART2 (PA2) at 115200 baud for serial output with buffered interrupt handling.
- A delay provider is instantiated for timing operations.

### Sensor Configuration

- The ISM330DHCX sensor is instantiated over I2C with the high I2C address.
- The device ID is read and verified; if mismatched, the program panics.
- The sensor is reset to default configuration and the program waits until reset completes.
- The embedded Finite State Machine is configured by applying a predefined program for free fall detection.
- This configuration is generated from a Universal Configuration File (JSON) and applied at runtime using a dedicated crate that interprets JSON entries into sensor register writes.
- Interrupt routing is configured to route the FSM interrupt to the INT1 pin.
- Interrupt notification mode is set to pulsed embedded latched mode.

### Main Loop

- The program continuously polls the sensor's interrupt source registers in a non-blocking asynchronous loop.
- When the FSM indicates a free fall event, a message "Free fall detected" is printed over UART.
- The loop includes a short delay to regulate polling frequency.

---

## Usage

1. Connect the ISM330DHCX sensor to the STM32F401RE Nucleo board via I2C1 (PB8/PB9).
2. Build and flash the firmware onto the STM32F401RE board.
3. Open a serial terminal at 115200 baud on the USART2 TX line.
4. Simulate or induce a free fall event on the sensor.
5. Observe the message "Free fall detected" printed on the terminal when the event occurs.

---

## Notes

- This example uses the **Embassy** asynchronous embedded framework to manage concurrency and peripheral access efficiently.
- UART output uses buffered interrupt-driven transmission with blocking writes for simplicity.
- The FSM configuration is dynamically generated from a JSON file and applied at runtime using a dedicated crate.
- The environment is `#![no_std]` and `#![no_main]` for embedded Rust applications.
- Panic behavior is handled via a panic probe handler.
- The FSM programs must be reloaded after each power cycle of the sensor.

---

## References

- [STM32F401RE Nucleo-64 Board](https://www.st.com/en/evaluation-tools/nucleo-f401re.html)
- [ISM330DHCX Datasheet](https://www.st.com/resource/en/datasheet/ism330dhcx.pdf)
- [ISM330DHCX Application Note on FSM](https://www.st.com/resource/en/application_note/an5388-ism330dhcx-finite-state-machine-stmicroelectronics.pdf)

---

*This documentation explains the embedded Rust program for free fall detection using the ISM330DHCX sensor's embedded Finite State Machine on the STM32F401RE Nucleo-64 board, leveraging the Embassy asynchronous embedded framework and runtime configuration from a JSON file.*