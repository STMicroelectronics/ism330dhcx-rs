#![no_main]
#![no_std]

use core::{
    cell::{RefCell, RefMut},
    fmt::Write,
};

use iis2mdc::Iis2mdc;
use iis2mdc_rs as iis2mdc;
use ism330dhcx_rs::prelude::*;
use ism330dhcx_rs::{Ism330dhcx, PROPERTY_DISABLE, PROPERTY_ENABLE};
use st_mems_bus::{i2c::I2cBus, BusOperation};

use panic_itm as _;

use cortex_m_rt::entry;
use stm32f4xx_hal::{
    hal::delay::DelayNs,
    i2c::{DutyCycle, I2c, Mode},
    pac::{self},
    prelude::*,
    serial::Config,
};

struct Ism330dhcxMaster<B, T> {
    pub sensor: RefCell<Ism330dhcx<B, T>>,
}

impl<'a, B: BusOperation, T: DelayNs> Ism330dhcxMaster<B, T> {
    fn from_bus(bus: B, tim: T) -> Self {
        Self {
            sensor: RefCell::new(Ism330dhcx::from_bus(bus, tim)),
        }
    }

    fn sensor(&self) -> RefMut<Ism330dhcx<B, T>> {
        self.sensor.borrow_mut()
    }

    fn as_passthrough_bus(&'a self) -> PassThroughIis2mdcBus<'a, B, T> {
        PassThroughIis2mdcBus {
            master: &self.sensor,
        }
    }
}

struct PassThroughIis2mdcBus<'a, B, T> {
    master: &'a RefCell<Ism330dhcx<B, T>>,
}

impl<'a, B, T> BusOperation for PassThroughIis2mdcBus<'a, B, T>
where
    B: BusOperation,
    T: DelayNs,
{
    type Error = ism330dhcx_rs::Error<B::Error>;

    fn read_bytes(&mut self, _rbuf: &mut [u8]) -> Result<(), Self::Error> {
        panic!("Not implemented")
    }

    // Write
    fn write_bytes(&mut self, wbuf: &[u8]) -> Result<(), Self::Error> {
        // Configure Sensor Hub to write IIS2MDC
        let sh_cfg_write = ShCfgWrite {
            slv0_add: (iis2mdc::I2CAddress::I2cAdd as u8) << 1, // I2C Addr is in 8bit
            slv0_subadd: wbuf[0],
            slv0_data: wbuf[1],
        };
        let mut s = self.master.borrow_mut();
        s.sh_cfg_write(&sh_cfg_write)?;

        // Disable accelerometer
        s.xl_data_rate_set(OdrXl::Off)?;

        // Enable I2C Master
        s.sh_master_set(PROPERTY_ENABLE)?;

        // Enable eccelerometer to trigger Sensor Hub operation
        s.xl_data_rate_set(OdrXl::_104hz)?;

        // Wait Sensor Hub operation flag set
        let _ = s.acceleration_raw_get()?;
        loop {
            let drdy = s.xl_flag_data_ready_get()?;
            if drdy == 1 {
                break;
            }
        }
        let mut master_status;
        loop {
            master_status = s.sh_status_get()?;
            if master_status.sens_hub_endop() == 1 {
                break;
            }
        }

        // Disable I2C master and XL (trigger)
        s.sh_master_set(PROPERTY_DISABLE)?;
        s.xl_data_rate_set(OdrXl::Off)?;

        Ok(())
    }

    // Read
    fn write_byte_read_bytes(
        &mut self,
        wbuf: &[u8; 1],
        rbuf: &mut [u8],
    ) -> Result<(), Self::Error> {
        // Assume len = rbuf.len() to avoid error checking
        let len = rbuf.len() as u8;

        let mut s = self.master.borrow_mut();

        // Disable accelerometer
        s.xl_data_rate_set(OdrXl::Off)?;

        // Configure Sensor Hub to read IIS2MDC
        let sh_cfg_read = ShCfgRead {
            slv_add: (iis2mdc::I2CAddress::I2cAdd as u8) << 1,
            slv_subadd: wbuf[0],
            slv_len: len,
        };
        s.sh_slv0_cfg_read(&sh_cfg_read)?;
        s.sh_slave_connected_set(AuxSensOn::Slv0)?;

        // Set write_once bit
        s.sh_write_mode_set(WriteOnce::OnlyFirstCycle)?;

        // Enable I2C Master
        s.sh_master_set(PROPERTY_ENABLE)?;

        // Enable accelerometer to trigger Sensor Hub operation
        s.xl_data_rate_set(OdrXl::_104hz)?;

        // Wait Sensor Hub operation flag set
        let _ = s.acceleration_raw_get()?;
        loop {
            let drdy = s.xl_flag_data_ready_get()?;
            if drdy == 1 {
                break;
            }
        }
        let mut master_status;
        loop {
            master_status = s.sh_status_get()?;
            if master_status.sens_hub_endop() == 1 {
                break;
            }
        }

        // Disable I2C master and XL (trigger)
        s.sh_master_set(PROPERTY_DISABLE)?;
        s.xl_data_rate_set(OdrXl::Off)?;

        s.sh_read_data_raw_get(rbuf, len)?;

        Ok(())
    }
}

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.use_hse(8.MHz()).sysclk(48.MHz()).freeze();

    let mut delay = cp.SYST.delay(&clocks);
    let tim1 = dp.TIM1.delay_us(&clocks);
    let tim2 = dp.TIM2.delay_us(&clocks);

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

    let master = Ism330dhcxMaster::from_bus(
        I2cBus::new(i2c, ism330dhcx_rs::I2CAddress::I2cAddH as u8),
        tim1,
    );

    delay.delay_ms(10);

    let mag_pt_bus = master.as_passthrough_bus();
    let mut mag = Iis2mdc::from_bus(mag_pt_bus, tim2);

    delay.delay_ms(10);

    match master.sensor().device_id_get() {
        Ok(value) => {
            if value != ism330dhcx_rs::ISM330DHCX_ID {
                panic!("Invalid master ID")
            }
        }
        Err(e) => writeln!(tx, "An error occured while reading master ID: {e:?}").unwrap(),
    }
    delay.delay_ms(25);

    // Restore default configuration
    master.sensor().reset_set(PROPERTY_ENABLE).unwrap();
    loop {
        let rst = master.sensor().reset_get().unwrap();
        if rst == 0 {
            break;
        }
    }

    // Start device configuration
    master.sensor().device_conf_set(PROPERTY_ENABLE).unwrap();

    // Some hardware require to enable pull up on master I2C interface
    // master.sensor().sh_pin_mode_set(ism330dhcx_rs::ShubPuEn::InternalPullUp).unwrap();

    // Chech if IIS2MDC connected to Sensor Hub
    match mag.device_id_get() {
        Ok(value) => {
            if value != iis2mdc::IIS2MDC_ID {
                panic!("Invalid slave ID: {value} (0x{value:X})")
            }
        }
        Err(e) => writeln!(tx, "An error occured while reading slave ID: {e:?}").unwrap(),
    }
    delay.delay_ms(25);

    // Configure IIS2MDC
    mag.block_data_update_set(PROPERTY_ENABLE).unwrap();
    mag.offset_temp_comp_set(PROPERTY_ENABLE).unwrap();
    mag.operating_mode_set(iis2mdc::prelude::Md::ContinuousMode)
        .unwrap();
    mag.data_rate_set(iis2mdc::prelude::Odr::_20hz).unwrap();

    // Prepare Sensor Hub to read data from external Slave0 continuously
    master.sensor().sh_data_rate_set(ShubOdr::_26hz).unwrap();
    let sh_cfg_read = ShCfgRead {
        slv_add: (iis2mdc::I2CAddress::I2cAdd as u8) << 1,
        slv_subadd: iis2mdc::prelude::Reg::OutxLReg as u8,
        slv_len: 6,
    };
    master.sensor().sh_slv0_cfg_read(&sh_cfg_read).unwrap();

    // Configure Sensro Hub to read one slave
    master
        .sensor()
        .sh_slave_connected_set(AuxSensOn::Slv0)
        .unwrap();

    // Set write once bit
    master
        .sensor()
        .sh_write_mode_set(WriteOnce::OnlyFirstCycle)
        .unwrap();

    // Enable I2C master
    master.sensor().sh_master_set(PROPERTY_ENABLE).unwrap();

    // Set xl and gy full scale
    master.sensor().xl_full_scale_set(FsXl::_2g).unwrap();
    master.sensor().gy_full_scale_set(FsGy::_2000dps).unwrap();

    // Enable Block Data Update
    master
        .sensor()
        .block_data_update_set(PROPERTY_ENABLE)
        .unwrap();

    // Set FIFO watermark (number of unread sensor data TAG + 6 bytes stored in FIFO) to 15 samples.
    // 5 * (Acc + Gyro + Mag)
    master.sensor().fifo_watermark_set(15).unwrap();

    // Set FIFO mode to stream mode (aka Continuous Mode)
    master.sensor().fifo_mode_set(FifoMode::StreamMode).unwrap();

    // Enable latched interrupt notification
    master
        .sensor()
        .int_notification_set(Lir::AllIntLatched)
        .unwrap();

    // Enable FIFO batching of Slave0
    master
        .sensor()
        .sh_batch_slave_0_set(PROPERTY_ENABLE)
        .unwrap();

    // Set FIFO batch XL/Gyro ODR to 26Hz
    // Enable Timestamp batching with no decimation
    master.sensor().fifo_xl_batch_set(BdrXl::_26hz).unwrap();
    master.sensor().fifo_gy_batch_set(BdrGy::_26hz).unwrap();
    master
        .sensor()
        .fifo_timestamp_decimation_set(OdrTsBatch::_1)
        .unwrap();
    master.sensor().timestamp_set(PROPERTY_ENABLE).unwrap();

    // Set Output Data Rate
    master.sensor().xl_data_rate_set(OdrXl::_26hz).unwrap();
    master.sensor().gy_data_rate_set(OdrGy::_26hz).unwrap();

    loop {
        let mut timestamp = 0_f32;
        let fifo_wtm = master.sensor().fifo_wtm_flag_get().unwrap();
        if fifo_wtm > 0 {
            // Read number of samples in FIFO
            let num = master.sensor().fifo_data_level_get().unwrap();
            for _ in 0..num {
                let reg_tag = master.sensor().fifo_sensor_tag_get().unwrap();
                match reg_tag {
                    FifoTag::XlNcTag => {
                        let acceleration_mg = master
                            .sensor()
                            .acceleration_raw_get()
                            .unwrap()
                            .map(ism330dhcx_rs::from_fs2g_to_mg);
                        writeln!(
                            tx,
                            "xl[mg]: {:4.2}    {:4.2}    {:4.2} T {}.{} ms",
                            acceleration_mg[0],
                            acceleration_mg[1],
                            acceleration_mg[2],
                            (timestamp / 1000.) as u32,
                            (timestamp % 1000.) as u32
                        )
                        .unwrap();
                    }
                    FifoTag::GyroNcTag => {
                        let angular_rate = master
                            .sensor()
                            .angular_rate_raw_get()
                            .unwrap()
                            .map(ism330dhcx_rs::from_fs2000dps_to_mdps);
                        writeln!(
                            tx,
                            "gy[mdps]: {:4.2}    {:4.2}    {:4.2} T {}.{} ms",
                            angular_rate[0],
                            angular_rate[1],
                            angular_rate[2],
                            (timestamp / 1000.) as u32,
                            (timestamp % 1000.) as u32
                        )
                        .unwrap();
                    }
                    FifoTag::SensorHubSlave0 => {
                        let mut data_raw = [0_u8; 6];
                        master
                            .sensor()
                            .sh_read_data_raw_get(&mut data_raw, 6)
                            .unwrap();
                        let mag_field_m_g = [
                            iis2mdc::from_lsb_to_mgauss(i16::from_le_bytes([
                                data_raw[0],
                                data_raw[1],
                            ])),
                            iis2mdc::from_lsb_to_mgauss(i16::from_le_bytes([
                                data_raw[2],
                                data_raw[3],
                            ])),
                            iis2mdc::from_lsb_to_mgauss(i16::from_le_bytes([
                                data_raw[4],
                                data_raw[5],
                            ])),
                        ];
                        writeln!(
                            tx,
                            "mag[mG]: {:4.2}    {:4.2}    {:4.2} T {}.{} ms",
                            mag_field_m_g[0],
                            mag_field_m_g[1],
                            mag_field_m_g[2],
                            (timestamp / 1000.) as u32,
                            (timestamp % 1000.) as u32
                        )
                        .unwrap();
                    }
                    FifoTag::TimestampTag => {
                        let raw = master.sensor().fifo_out_raw_get().unwrap();
                        let tick = u32::from_le_bytes([raw[0], raw[1], raw[2], raw[3]]);
                        timestamp = ism330dhcx_rs::from_lsb_to_nsec(tick) as f32;
                    }
                    _ => {
                        // Flush unused samples
                        let _ = master.sensor().fifo_out_raw_get().unwrap();
                    }
                };
            }
        }
    }
}
