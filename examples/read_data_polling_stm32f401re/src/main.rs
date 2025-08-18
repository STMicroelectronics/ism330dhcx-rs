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

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.use_hse(8.MHz()).sysclk(48.MHz()).freeze();

    let delay = cp.SYST.delay(&clocks);

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

    // Start device configuration
    sensor.device_conf_set(PROPERTY_ENABLE).unwrap();
    // Enable Block Data Update
    sensor.block_data_update_set(PROPERTY_ENABLE).unwrap();

    // Set Output Data Rate
    sensor.xl_data_rate_set(OdrXl::_12_5hz).unwrap();
    sensor.gy_data_rate_set(OdrGy::_12_5hz).unwrap();
    // Set Full Scale
    sensor.xl_full_scale_set(FsXl::_2g).unwrap();
    sensor.gy_full_scale_set(FsGy::_2000dps).unwrap();

    // Configure filtering chain(No aux interface)
    // Accelerometer - LPF1 + LPF2 path
    sensor
        .xl_hp_path_on_out_set(HpSlopeXlEn::LpOdrDiv100)
        .unwrap();
    sensor.xl_filter_lp2_set(PROPERTY_ENABLE).unwrap();

    // Read samples in polling mode (no int)
    loop {
        let rdy = sensor.xl_flag_data_ready_get().unwrap();
        if rdy == 1 {
            let data_raw_xl = sensor.acceleration_raw_get().unwrap();
            let acceleration_mg = data_raw_xl.map(ism330dhcx_rs::from_fs2g_to_mg);
            writeln!(
                tx,
                "Acceleration [mg]: {:4.2}    {:4.2}    {:4.2}",
                acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]
            )
            .unwrap();
        }

        let rdy = sensor.gy_flag_data_ready_get().unwrap();
        if rdy == 1 {
            let data_raw_gy = sensor.angular_rate_raw_get().unwrap();
            let angular_rate_mdps = data_raw_gy.map(ism330dhcx_rs::from_fs2000dps_to_mdps);
            writeln!(
                tx,
                "Angular rate [mdps]: {:4.2}    {:4.2}    {:4.2}",
                angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]
            )
            .unwrap();
        }

        let rdy = sensor.temp_flag_data_ready_get().unwrap();
        if rdy == 1 {
            let data_raw_temp = sensor.temperature_raw_get().unwrap();
            let temperature_degc = ism330dhcx_rs::from_lsb_to_celsius(data_raw_temp);
            writeln!(tx, "Temperature [degC]: {:6.2}", temperature_degc).unwrap();
        }
    }
}
