# ISM330DHCX Free Fall Detection Using Embedded Finite State Machine on STM32F401RE Nucleo-64

This example demonstrates how to configure and use the **ISM330DHCX** inertial sensor's embedded Finite State Machine (FSM) to detect free fall events on an **STM32F401RE** microcontroller board. The sensor is interfaced via I2C, programmed with a predefined FSM configuration for free fall detection, and outputs event notifications over UART.

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

- The program initializes STM32F401RE peripherals: clocks, GPIO pins, I2C1, USART2, and delay abstraction.
- I2C1 is configured at 100 kHz Standard Mode on PB8 (SCL) and PB9 (SDA).
- USART2 is configured on PA2 at 115200 baud for serial debug output.
- A delay provider is created using the Cortex-M system timer (SYST).

### Sensor Setup and FSM Configuration

- The ISM330DHCX sensor is instantiated over I2C with the high I2C address.
- The sensor's device ID is read and verified to ensure correct sensor presence.
- The sensor is reset to default configuration and the program waits until reset completes.
- The embedded FSM is configured by loading a predefined Universal Configuration File (JSON) program for free fall detection.
- This JSON configuration is parsed and applied at runtime using a dedicated crate that converts the JSON data into register writes.
- Interrupt routing is configured to route the FSM interrupt to the INT1 pin.
- Interrupt notification mode is set to pulsed embedded latched mode.

### Main Loop

- The program continuously polls the sensor's FSM interrupt status registers.
- When the FSM indicates a free fall event, a message "Free fall detected" is printed over UART.
- The loop runs with a 1 ms delay between polls.

---

## Usage

1. Connect the ISM330DHCX sensor to the STM32F401RE Nucleo board via I2C1 (PB8/PB9).
2. Build and flash the firmware onto the STM32F401RE board.
3. Open a serial terminal at 115200 baud on the USART2 TX line.
4. Simulate or induce a free fall event on the sensor.
5. Observe the message "Free fall detected" printed on the terminal when the event occurs.

---

## Notes

- This example uses polling mode to check FSM interrupt status instead of hardware interrupt lines.
- UART output uses blocking writes for simplicity.
- The environment is `#![no_std]` and `#![no_main]` for embedded Rust applications.
- Panic behavior is handled via ITM debug output.
- The FSM configuration is generated from a JSON file and applied at runtime using a crate that interprets JSON data into sensor register writes.
- FSM programs must be reloaded after each power cycle of the sensor.

---

## References

- [STM32F401RE Nucleo-64 Board](https://www.st.com/en/evaluation-tools/nucleo-f401re.html)
- [ISM330DHCX Datasheet](https://www.st.com/resource/en/datasheet/ism330dhcx.pdf)
- [ISM330DHCX Application Note on FSM](https://www.st.com/resource/en/application_note/an5388-ism330dhcx-finite-state-machine-stmicroelectronics.pdf)
- [stm32f4xx-hal Rust crate](https://docs.rs/stm32f4xx-hal)

---

*This documentation explains the embedded Rust program for free fall detection using the ISM330DHCX sensor's embedded Finite State Machine on the STM32F401RE Nucleo-64 board, integrating configuration details generated from a JSON file and applied at runtime.*