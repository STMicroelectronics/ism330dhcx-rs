#![no_main]
#![no_std]

use core::fmt::Write;

use ism330dhcx_rs::prelude::*;
use ism330dhcx_rs::{Ism330dhcx, PROPERTY_ENABLE};
use panic_itm as _;

use cortex_m_rt::entry;
use stm32f4xx_hal::{
    hal::delay::DelayNs,
    i2c::{DutyCycle, I2c, Mode},
    pac::{self},
    prelude::*,
    serial::Config,
};

mod mlc_config;
use mlc_config::FREE_FALL;
use st_mems_reg_config_conv::ucf_entry::MemsUcfOp;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.use_hse(8.MHz()).sysclk(48.MHz()).freeze();

    let mut delay = cp.SYST.delay(&clocks);

    let gpiob = dp.GPIOB.split();
    let gpioa = dp.GPIOA.split();

    let scl = gpiob.pb8;
    let sda = gpiob.pb9;

    let i2c = I2c::new(
        dp.I2C1,
        (scl, sda),
        Mode::Fast {
            frequency: 400.kHz(),
            duty_cycle: DutyCycle::Ratio2to1,
        },
        &clocks,
    );

    let tx_pin = gpioa.pa2.into_alternate();
    let mut tx = dp
        .USART2
        .tx(
            tx_pin,
            Config::default()
                .baudrate(115200.bps())
                .wordlength_8()
                .parity_none(),
            &clocks,
        )
        .unwrap();

    delay.delay_ms(5);

    let mut sensor = Ism330dhcx::new_i2c(i2c, ism330dhcx_rs::I2CAddress::I2cAddH, delay);

    match sensor.device_id_get() {
        Ok(value) => {
            if value != ism330dhcx_rs::ISM330DHCX_ID {
                panic!("Invalid sensor ID")
            }
        }
        Err(e) => writeln!(tx, "An error occured while reading sensor ID: {e:?}").unwrap(),
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
    for ufc_line in FREE_FALL {
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
    pin_int1_route.fsm_int1_a.set_int1_fsm1(PROPERTY_ENABLE);
    sensor.pin_int1_route_set(&mut pin_int1_route).unwrap();
    // Configure interrupt pin mode notification
    sensor
        .int_notification_set(Lir::BasePulsedEmbLatched)
        .unwrap();

    loop {
        // Read interrupt source registers in polling mode (no int)
        let status = sensor.all_sources_get().unwrap();
        if status.fsm_status_a.is_fsm1() == 1 {
            writeln!(tx, "Free fall detected").unwrap();
        }

        sensor.tim.delay_ms(1);
    }
}
