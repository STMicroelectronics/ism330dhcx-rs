use defmt::info;
use maybe_async::maybe_async;
use crate::*;

#[cfg(feature = "async")]
use iis2mdc_rs::asynchronous as iis2mdc;

#[cfg(not(feature = "async"))]
use iis2mdc_rs::blocking as iis2mdc;

const IIS2MDC_ADDR: u8 = iis2mdc::I2CAddress::I2cAdd as u8;

#[maybe_async]
pub async fn run<B, D, L, I>(bus: B, mut tx: L, mut delay: D, mut _int_pin : I) -> !
where
    B: BusOperation,
    D: DelayNs + Clone,
    L: embedded_io::Write,
    I: InterruptPin
{
    use ism330dhcx::prelude::*;
    use ism330dhcx::*;

    info!("Configuring the sensor");
    let mut sensor = Ism330dhcx::from_bus(bus, delay.clone());

    // boot time
    delay.delay_ms(5).await;

    // Check device ID
    let id = sensor.device_id_get().await.unwrap();
    info!("Device ID: {:x}", id);
    if id != ISM330DHCX_ID {
        info!("Unexpected device ID: {:x}", id);
        writeln!(tx, "Unexpected device ID: {:x}", id).unwrap();
        loop {}
    }

    // Start device configuration
    sensor.device_conf_set(PROPERTY_ENABLE).await.unwrap();

    // Some hardware require to enable pull up on master I2C interface
    // sensor.sh_pin_mode_set(ism330dhcx_rs::ShubPuEn::InternalPullUp).await.unwrap();

    /*
     * Configure IIS2MDC target
     */

    let pass = Ism330dhcxPassthrough::new_from_sensor(&mut sensor, IIS2MDC_ADDR);
    let mut iis2mdc = iis2mdc::Iis2mdc::from_bus(pass, delay.clone());
    info!("initializing IIS2MDC");

    // Check if IIS2MDC is connected to Sensor Hub
    match iis2mdc.device_id_get().await {
        Ok(id) => {
            if id != iis2mdc::IIS2MDC_ID {
                writeln!(tx, "Device (IIS2MDC) ID mismatch: {:#02x}", id).unwrap();
                loop {}
            }
        }
        Err(e) => writeln!(tx, "Error in reading id: {:?}", e).unwrap(),
    }
    delay.delay_ms(25).await;

    // Configure IIS2MDC
    iis2mdc.block_data_update_set(PROPERTY_ENABLE).await.unwrap();
    iis2mdc.offset_temp_comp_set(PROPERTY_ENABLE).await.unwrap();
    iis2mdc.operating_mode_set(iis2mdc::prelude::Md::ContinuousMode)
        .await
        .unwrap();
    iis2mdc.data_rate_set(iis2mdc::prelude::Odr::_20hz).await.unwrap();

    /*
     *  Slave settings ended: setup Sensor Hub side
     */

    // Prepare Sensor Hub to read data from external Slave0 continuously
    sensor.sh_data_rate_set(ShubOdr::_26hz).await.unwrap();
    let sh_cfg_read = ShCfgRead {
        slv_add: (iis2mdc::I2CAddress::I2cAdd as u8) << 1,
        slv_subadd: iis2mdc::prelude::Reg::OutxLReg as u8,
        slv_len: 6,
    };
    sensor.sh_slv0_cfg_read(&sh_cfg_read).await.unwrap();

    // Configure Sensor Hub to read one slave
    sensor.sh_slave_connected_set(AuxSensOn::Slv0)
        .await
        .unwrap();

    // Set write once bit
    sensor.sh_write_mode_set(WriteOnce::OnlyFirstCycle)
        .await
        .unwrap();

    // Enable I2C master
    sensor.sh_master_set(PROPERTY_ENABLE).await.unwrap();

    // Set xl and gy full scale
    sensor.xl_full_scale_set(FsXl::_2g).await.unwrap();
    sensor.gy_full_scale_set(FsGy::_2000dps).await.unwrap();

    // Enable Block Data Update
    sensor
        .block_data_update_set(PROPERTY_ENABLE)
        .await
        .unwrap();

    // Set FIFO watermark (number of unread sensor data TAG + 6 bytes stored in FIFO) to 15 samples.
    // 5 * (Acc + Gyro + Mag)
    sensor.fifo_watermark_set(15).await.unwrap();

    // Set FIFO mode to stream mode (aka Continuous Mode)
    sensor.fifo_mode_set(FifoMode::StreamMode).await.unwrap();

    // Enable latched interrupt notification
    sensor
        .int_notification_set(Lir::AllIntLatched)
        .await
        .unwrap();

    // Enable FIFO batching of Slave0
    sensor
        .sh_batch_slave_0_set(PROPERTY_ENABLE)
        .await
        .unwrap();

    // Set FIFO batch XL/Gyro ODR to 26Hz
    // Enable Timestamp batching with no decimation
    sensor.fifo_xl_batch_set(BdrXl::_26hz).await.unwrap();
    sensor.fifo_gy_batch_set(BdrGy::_26hz).await.unwrap();
    sensor
        .fifo_timestamp_decimation_set(OdrTsBatch::_1)
        .await
        .unwrap();
    sensor.timestamp_set(PROPERTY_ENABLE).await.unwrap();

    // Set Output Data Rate
    sensor.xl_data_rate_set(OdrXl::_26hz).await.unwrap();
    sensor.gy_data_rate_set(OdrGy::_26hz).await.unwrap();

    loop {
        let mut timestamp = 0_f32;
        let fifo_wtm = sensor.fifo_wtm_flag_get().await.unwrap();
        if fifo_wtm > 0 {
            // Read number of samples in FIFO
            let num = sensor.fifo_data_level_get().await.unwrap();
            for _ in 0..num {
                let reg_tag = sensor.fifo_sensor_tag_get().await.unwrap();
                match reg_tag {
                    FifoTag::XlNcTag => {
                        let acceleration_mg = sensor
                            .acceleration_raw_get()
                            .await
                            .unwrap()
                            .map(from_fs2g_to_mg);
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
                        let angular_rate = sensor
                            .angular_rate_raw_get()
                            .await
                            .unwrap()
                            .map(from_fs2000dps_to_mdps);
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
                        sensor
                            .sh_read_data_raw_get(&mut data_raw, 6)
                            .await
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
                        let raw = sensor.fifo_out_raw_get().await.unwrap();
                        let tick = u32::from_le_bytes([raw[0], raw[1], raw[2], raw[3]]);
                        timestamp = from_lsb_to_nsec(tick) as f32;
                    }
                    _ => {
                        // Flush unused samples
                        let _ = sensor.fifo_out_raw_get().await.unwrap();
                    }
                };
            }
        }
    }
}
