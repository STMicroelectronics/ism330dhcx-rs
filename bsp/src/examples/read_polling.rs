use defmt::info;
use maybe_async::maybe_async;
use crate::*;

#[maybe_async]
pub async fn run<B, D, L>(bus: B, mut tx: L, mut delay: D, _irq: ()) -> !
where
    B: BusOperation,
    D: DelayNs + Clone,
    L: embedded_io::Write
{
    use ism330dhcx::prelude::*;
    use ism330dhcx::*;

    info!("Configuring the sensor");
    let mut sensor = Ism330dhcx::from_bus(bus, delay.clone());

    // boot time
    delay.delay_ms(25).await;

    // Check device ID
    let id = sensor.device_id_get().await.unwrap();
    info!("Device ID: {:x}", id);
    if id != ISM330DHCX_ID {
        info!("Unexpected device ID: {:x}", id);
        writeln!(tx, "Unexpected device ID: {:x}", id).unwrap();
        loop {}
    }

    // Restore default configuration
    sensor.reset_set(PROPERTY_ENABLE).await.unwrap();

    loop {
        let rst = sensor.reset_get().await.unwrap();
        if rst == 0 {
            break;
        }
    }

    // Start device configuration
    sensor.device_conf_set(PROPERTY_ENABLE).await.unwrap();
    // Enable Block Data Update
    sensor.block_data_update_set(PROPERTY_ENABLE).await.unwrap();

    // Set Output Data Rate
    sensor.xl_data_rate_set(OdrXl::_12_5hz).await.unwrap();
    sensor.gy_data_rate_set(OdrGy::_12_5hz).await.unwrap();
    // Set Full Scale
    sensor.xl_full_scale_set(FsXl::_2g).await.unwrap();
    sensor.gy_full_scale_set(FsGy::_2000dps).await.unwrap();

    // Configure filtering chain(No aux interface)
    // Accelerometer - LPF1 + LPF2 path
    sensor
        .xl_hp_path_on_out_set(HpSlopeXlEn::LpOdrDiv100)
        .await.unwrap();
    sensor.xl_filter_lp2_set(PROPERTY_ENABLE).await.unwrap();

    // Read samples in polling mode (no int)
    loop {
        let rdy = sensor.xl_flag_data_ready_get().await.unwrap();
        if rdy == 1 {
            let data_raw_xl = sensor.acceleration_raw_get().await.unwrap();
            let acceleration_mg = data_raw_xl.map(from_fs2g_to_mg);
            writeln!(
                tx,
                "Acceleration [mg]: {:4.2}    {:4.2}    {:4.2}",
                acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]
            )
            .unwrap();
        }

        let rdy = sensor.gy_flag_data_ready_get().await.unwrap();
        if rdy == 1 {
            let data_raw_gy = sensor.angular_rate_raw_get().await.unwrap();
            let angular_rate_mdps = data_raw_gy.map(from_fs2000dps_to_mdps);
            writeln!(
                tx,
                "Angular rate [mdps]: {:4.2}    {:4.2}    {:4.2}",
                angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]
            )
            .unwrap();
        }

        let rdy = sensor.temp_flag_data_ready_get().await.unwrap();
        if rdy == 1 {
            let data_raw_temp = sensor.temperature_raw_get().await.unwrap();
            let temperature_degc = from_lsb_to_celsius(data_raw_temp);
            writeln!(tx, "Temperature [degC]: {:6.2}", temperature_degc).unwrap();
        }
    }
}
