#![no_main]
#![no_std]

use core::{fmt::Write, ops::Range};

use ism330dhcx_rs::prelude::*;
use ism330dhcx_rs::{Ism330dhcx, PROPERTY_ENABLE};
use panic_itm as _;
use st_mems_bus::i2c::I2cBus;

use cortex_m_rt::entry;
use stm32f4xx_hal::{
    hal::delay::DelayNs,
    i2c::{DutyCycle, I2c, Mode},
    pac::{self, I2C1},
    prelude::*,
    serial::Config,
    timer::SysDelay,
};

const ST_RANGE_MG: Range<f32> = 90.0..1700.0;
const ST_RANGE_MDPS: Range<f32> = 150000.0..700000.0;

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

    // === Accelerometer Self Test ===
    // Set Output Data Rate
    sensor.xl_data_rate_set(OdrXl::_52hz).unwrap();
    sensor.xl_full_scale_set(FsXl::_4g).unwrap();
    // Wait stable output
    sensor.tim.delay_ms(100);

    // Check if new value available
    poll_new_xl_val_available(&mut sensor);

    // Read dummy data and discard it
    let _ = sensor.acceleration_raw_get().unwrap();
    // Read 5 sample and get the average value for each axis
    let mut val_st_off = [0_f32; 3];
    for _ in 0..5 {
        poll_new_xl_val_available(&mut sensor);

        let data_raw = sensor.acceleration_raw_get().unwrap();

        (0..3).for_each(|i| val_st_off[i] += ism330dhcx_rs::from_fs4g_to_mg(data_raw[i]));
    }
    // Calculate the mg average values
    (0..3).for_each(|i| val_st_off[i] /= 5.);

    // Enable Self Test positive (or negative)
    sensor.xl_self_test_set(StXl::Positive).unwrap();
    // sensor.xl_self_test_set(StXl::Negative).unwrap();

    // Wait stable output
    sensor.tim.delay_ms(100);

    // Check if new value available
    poll_new_xl_val_available(&mut sensor);

    // Read dummy data and discard it
    let _ = sensor.acceleration_raw_get().unwrap();

    // Read 5 sample and get the average value for each axis
    let mut val_st_on = [0_f32; 3];
    for _ in 0..5 {
        poll_new_xl_val_available(&mut sensor);

        let data_raw = sensor.acceleration_raw_get().unwrap();

        (0..3).for_each(|i| val_st_on[i] += ism330dhcx_rs::from_fs4g_to_mg(data_raw[i]));
    }
    // Calculate the mg average values
    (0..3).for_each(|i| val_st_on[i] /= 5.);

    // Calculate the mg values for self test
    let mut test_val = [0_f32; 3];
    (0..3).for_each(|i| test_val[i] = (val_st_on[i] - val_st_off[i]).abs());

    let xl_st_result = test_val.iter().all(|v| ST_RANGE_MG.contains(v));

    // Disable Self Test
    sensor.xl_self_test_set(StXl::Disable).unwrap();
    // Disable sensor
    sensor.xl_data_rate_set(OdrXl::Off).unwrap();

    // === Gyroscope Self Test ===
    // Set Output Data Rate
    sensor.gy_data_rate_set(OdrGy::_208hz).unwrap();
    // Set full scale
    sensor.gy_full_scale_set(FsGy::_2000dps).unwrap();
    // Wait stable output
    sensor.tim.delay_ms(100);

    // Check if new value available
    poll_new_gy_val_available(&mut sensor);

    // Read dummy data and discard it
    let _ = sensor.angular_rate_raw_get().unwrap();
    // Read 5 sample and get the average value for each axis
    val_st_off = [0_f32; 3];
    for _ in 0..5 {
        poll_new_gy_val_available(&mut sensor);

        let data_raw = sensor.angular_rate_raw_get().unwrap();

        (0..3)
            .for_each(|i| val_st_off[i] += ism330dhcx_rs::from_fs2000dps_to_mdps(data_raw[i]));
    }
    // Calculate the mg average values
    (0..3).for_each(|i| val_st_off[i] /= 5.);

    // Enable Self Test positive (or negative)
    sensor.gy_self_test_set(StGy::Positive).unwrap();
    // sensor.gy_self_test_set(StGy::Negative).unwrap();

    // Wait stable output
    sensor.tim.delay_ms(100);

    // Check if new value available
    poll_new_gy_val_available(&mut sensor);

    // Read dummy data and discard it
    let _ = sensor.angular_rate_raw_get().unwrap();

    // Read 5 sample and get the average value for each axis
    val_st_on = [0_f32; 3];
    for _ in 0..5 {
        poll_new_gy_val_available(&mut sensor);

        let data_raw = sensor.angular_rate_raw_get().unwrap();

        (0..3).for_each(|i| val_st_on[i] += ism330dhcx_rs::from_fs2000dps_to_mdps(data_raw[i]));
    }
    // Calculate the mg average values
    (0..3).for_each(|i| val_st_on[i] /= 5.);

    // Calculate the mg values for self test
    test_val = [0_f32; 3];
    (0..3).for_each(|i| test_val[i] = (val_st_on[i] - val_st_off[i]).abs());

    let gy_st_result = test_val.iter().all(|v| ST_RANGE_MDPS.contains(v));

    // Disable self test
    sensor.gy_self_test_set(StGy::Disable).unwrap();
    // Disable sensor
    sensor.gy_data_rate_set(OdrGy::Off).unwrap();

    writeln!(
        tx,
        "{}",
        if xl_st_result {
            "XL Self Test - PASS"
        } else {
            "XL Self Test - FAIL"
        }
    )
    .unwrap();

    writeln!(
        tx,
        "{}",
        if gy_st_result {
            "GY Self Test - PASS"
        } else {
            "GY Self Test - FAIL"
        }
    )
    .unwrap();

    loop {
        sensor.tim.delay_ms(500);
    }
}

fn poll_new_xl_val_available(sensor: &mut Ism330dhcx<I2cBus<I2c<I2C1>>, SysDelay>) {
    loop {
        let drdy = sensor.xl_flag_data_ready_get().unwrap();
        if drdy == 1 {
            break;
        }
    }
}

fn poll_new_gy_val_available(sensor: &mut Ism330dhcx<I2cBus<I2c<I2C1>>, SysDelay>) {
    loop {
        let drdy = sensor.gy_flag_data_ready_get().unwrap();
        if drdy == 1 {
            break;
        }
    }
}
