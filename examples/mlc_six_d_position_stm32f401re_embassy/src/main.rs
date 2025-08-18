#![no_std]
#![no_main]

use core::fmt::{Display, Write};

use embassy_executor::Spawner;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::Pull;
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::time::khz;
use embassy_stm32::usart::{self, BufferedInterruptHandler, DataBits, Parity, UartTx};
use embassy_stm32::{bind_interrupts, peripherals, peripherals::USART2};
use embassy_time::Delay;
use embedded_hal::delay::DelayNs;
use heapless::String;

mod mlc_config;

use ism330dhcx_rs::prelude::*;
use ism330dhcx_rs::{Ism330dhcx, PROPERTY_ENABLE};
use mlc_config::SIX_D;
use st_mems_reg_config_conv::ucf_entry::MemsUcfOp;

use {defmt_rtt as _, panic_probe as _};

#[defmt::panic_handler]
fn panic() -> ! {
    core::panic!("panic via `defmt::panic!")
}

bind_interrupts!(struct Irqs {
    USART2 => BufferedInterruptHandler<USART2>;
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
});

#[repr(u8)]
enum SixdEvent {
    None,
    XPointingUp,
    XPointingDown,
    YPointingDown,
    YPointingUp,
    ZPointingUp,
    ZPointingDown,
    Unknown(u8),
}

impl From<u8> for SixdEvent {
    fn from(value: u8) -> Self {
        match value {
            0 => Self::None,
            1 => Self::XPointingUp,
            2 => Self::XPointingDown,
            3 => Self::YPointingDown,
            4 => Self::YPointingUp,
            5 => Self::ZPointingUp,
            6 => Self::ZPointingDown,
            other => Self::Unknown(other),
        }
    }
}

impl Display for SixdEvent {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::None => write!(f, "None"),
            Self::XPointingUp => write!(f, "X-axis pointing up"),
            Self::XPointingDown => write!(f, "X-axis pointing down"),
            Self::YPointingUp => write!(f, "Y-axis pointing up"),
            Self::YPointingDown => write!(f, "Y-axis pointing down"),
            Self::ZPointingUp => write!(f, "Z-axis pointing up"),
            Self::ZPointingDown => write!(f, "Z-axis pointing down"),
            Self::Unknown(v) => write!(f, "Unkown event: {v}"),
        }
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    let mut usart_cfg = usart::Config::default();
    usart_cfg.baudrate = 115200;
    usart_cfg.data_bits = DataBits::DataBits8;
    usart_cfg.parity = Parity::ParityNone;

    let mut tx = UartTx::new(p.USART2, p.PA2, p.DMA1_CH6, usart_cfg).unwrap();

    let i2c = I2c::new(
        p.I2C1,
        p.PB8,
        p.PB9,
        Irqs,
        p.DMA1_CH7,
        p.DMA1_CH5,
        khz(100),
        Default::default(),
    );

    let mut int_pin = ExtiInput::new(p.PB10, p.EXTI10, Pull::None);

    let mut delay = Delay;
    let mut msg = String::<64>::new();

    delay.delay_ms(10);

    let mut sensor = Ism330dhcx::new_i2c(i2c, ism330dhcx_rs::I2CAddress::I2cAddH, delay);

    match sensor.device_id_get() {
        Ok(value) => {
            if value != ism330dhcx_rs::ISM330DHCX_ID {
                panic!("Invalid sensor ID")
            }
        }
        Err(e) => {
            writeln!(&mut msg, "An error occured while reading sensor ID: {e:?}").unwrap();
            tx.blocking_write(msg.as_bytes()).unwrap();
            msg.clear();
        }
    }
    sensor.tim.delay_ms(25);

    // Restore default configuration
    sensor.reset_set(PROPERTY_ENABLE).unwrap();

    loop {
        let rst = sensor.reset_get().unwrap();
        if rst == 0 {
            break;
        }
    }

    // Start Matchine Learning Core configuration
    for ufc_line in SIX_D {
        match ufc_line.op {
            MemsUcfOp::Delay => sensor.tim.delay_ms(ufc_line.data.into()),
            MemsUcfOp::Write => sensor
                .write_to_register(ufc_line.address, &[ufc_line.data])
                .unwrap(),
            _ => {}
        }
    }

    // Route signals on interrupt pin 1
    let mut pin_int1_route = sensor.pin_int1_route_get().unwrap();
    pin_int1_route.mlc_int1.set_int1_mlc1(PROPERTY_ENABLE);
    sensor.pin_int1_route_set(&mut pin_int1_route).unwrap();
    // Configure interrupt pin mode notification
    sensor
        .int_notification_set(Lir::BasePulsedEmbLatched)
        .unwrap();

    loop {
        int_pin.wait_for_rising_edge().await;
        let status = sensor.mlc_status_get().unwrap();
        if status.is_mlc1() == 1 {
            let mut buff = [0_u8];
            sensor.mlc_out_get(&mut buff).unwrap();
            let catched: SixdEvent = buff[0].into();
            msg.clear();
            writeln!(&mut msg, "{catched}").unwrap();
            tx.blocking_write(msg.as_bytes()).unwrap();
        }
    }
}
