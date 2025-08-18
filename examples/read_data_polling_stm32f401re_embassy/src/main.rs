#![no_std]
#![no_main]

use core::fmt::Write;

use embassy_executor::Spawner;
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::time::khz;
use embassy_stm32::usart::{self, BufferedInterruptHandler, DataBits, Parity, UartTx};
use embassy_stm32::{bind_interrupts, peripherals, peripherals::USART2};
use embassy_time::Delay;
use embedded_hal::delay::DelayNs;
use heapless::String;

use ism330dhcx_rs::prelude::*;
use ism330dhcx_rs::{Ism330dhcx, PROPERTY_ENABLE};

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
            msg.clear();
            writeln!(
                &mut msg,
                "Acceleration [mg]: {:4.2}    {:4.2}    {:4.2}",
                acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]
            )
            .unwrap();
            tx.write(msg.as_bytes()).await.unwrap();
        }

        let rdy = sensor.gy_flag_data_ready_get().unwrap();
        if rdy == 1 {
            let data_raw_gy = sensor.angular_rate_raw_get().unwrap();
            let angular_rate_mdps = data_raw_gy.map(ism330dhcx_rs::from_fs2000dps_to_mdps);
            msg.clear();
            writeln!(
                &mut msg,
                "Angular rate [mdps]: {:4.2}    {:4.2}    {:4.2}",
                angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]
            )
            .unwrap();
            tx.write(msg.as_bytes()).await.unwrap();
        }

        let rdy = sensor.temp_flag_data_ready_get().unwrap();
        if rdy == 1 {
            let data_raw_temp = sensor.temperature_raw_get().unwrap();
            let temperature_degc = ism330dhcx_rs::from_lsb_to_celsius(data_raw_temp);
            msg.clear();
            writeln!(&mut msg, "Temperature [degC]: {:6.2}", temperature_degc).unwrap();
            tx.write(msg.as_bytes()).await.unwrap();
        }
    }
}
