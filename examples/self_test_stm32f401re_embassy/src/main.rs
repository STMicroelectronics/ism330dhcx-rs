#![no_std]
#![no_main]

use core::fmt::Write;
use core::ops::Range;

use embassy_executor::Spawner;
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::mode::Async;
use embassy_stm32::time::khz;
use embassy_stm32::usart::{self, BufferedInterruptHandler, DataBits, Parity, UartTx};
use embassy_stm32::{bind_interrupts, peripherals, peripherals::USART2};
use embassy_time::Delay;
use embedded_hal::delay::DelayNs;
use heapless::String;

use ism330dhcx_rs::prelude::*;
use ism330dhcx_rs::{Ism330dhcx, PROPERTY_ENABLE};
use st_mems_bus::i2c::I2cBus;

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

const ST_RANGE_MG: Range<f32> = 90.0..1700.0;
const ST_RANGE_MDPS: Range<f32> = 150000.0..700000.0;

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
            tx.write(msg.as_bytes()).await.unwrap();
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
        &mut msg,
        "{}",
        if xl_st_result {
            "XL Self Test - PASS"
        } else {
            "XL Self Test - FAIL"
        }
    )
    .unwrap();
    tx.write(msg.as_bytes()).await.unwrap();
    msg.clear();

    writeln!(
        &mut msg,
        "{}",
        if gy_st_result {
            "GY Self Test - PASS"
        } else {
            "GY Self Test - FAIL"
        }
    )
    .unwrap();
    tx.write(msg.as_bytes()).await.unwrap();

    loop {
        sensor.tim.delay_ms(500);
    }
}

fn poll_new_xl_val_available(sensor: &mut Ism330dhcx<I2cBus<I2c<'_, Async>>, Delay>) {
    loop {
        let drdy = sensor.xl_flag_data_ready_get().unwrap();
        if drdy == 1 {
            break;
        }
    }
}

fn poll_new_gy_val_available(sensor: &mut Ism330dhcx<I2cBus<I2c<'_, Async>>, Delay>) {
    loop {
        let drdy = sensor.gy_flag_data_ready_get().unwrap();
        if drdy == 1 {
            break;
        }
    }
}
