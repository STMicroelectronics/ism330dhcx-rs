use defmt::info;
use maybe_async::maybe_async;
use crate::*;
use ism330dhcx::*;
use ism330dhcx::prelude::*;
use core::ops::Range;

const ST_RANGE_MG: Range<f32> = 90.0..1700.0;
const ST_RANGE_MDPS: Range<f32> = 150000.0..700000.0;

#[maybe_async]
pub async fn run<B, D, L>(bus: B, mut tx: L, mut delay: D, _int_pin: ()) -> !
where
    B: BusOperation,
    D: DelayNs + Clone,
    L: embedded_io::Write,
{

    info!("Configuring the sensor");
    let mut sensor = Ism330dhcx::from_bus(bus, delay.clone());

    // boot time
    delay.delay_ms(25).await;

    // Check device ID
    let whoami = sensor.device_id_get().await.unwrap();
    info!("Device ID: {:x}", whoami);
    if whoami != ISM330DHCX_ID {
        writeln!(tx, "Device ID mismatch: {:#02x}", whoami).unwrap();
        loop {}
    }

    // Start device configuration
    sensor.device_conf_set(PROPERTY_ENABLE).await.unwrap();
    // Enable Block Data Update
    sensor.block_data_update_set(PROPERTY_ENABLE).await.unwrap();

    // === Accelerometer Self Test ===
    // Set Output Data Rate
    sensor.xl_data_rate_set(OdrXl::_52hz).await.unwrap();
    sensor.xl_full_scale_set(FsXl::_4g).await.unwrap();
    // Wait stable output
    delay.delay_ms(100).await;

    // Check if new value available
    poll_new_xl_val_available(&mut sensor).await;

    // Read dummy data and discard it
    let _ = sensor.acceleration_raw_get().await.unwrap();
    // Read 5 sample and get the average value for each axis
    let mut val_st_off = [0_f32; 3];
    for _ in 0..5 {
        poll_new_xl_val_available(&mut sensor).await;

        let data_raw = sensor.acceleration_raw_get().await.unwrap();

        (0..3).for_each(|i| val_st_off[i] += from_fs4g_to_mg(data_raw[i]));
    }
    // Calculate the mg average values
    (0..3).for_each(|i| val_st_off[i] /= 5.);

    // Enable Self Test positive (or negative)
    sensor.xl_self_test_set(StXl::Positive).await.unwrap();
    // sensor.xl_self_test_set(StXl::Negative).await.unwrap();

    // Wait stable output
    delay.delay_ms(100).await;

    // Check if new value available
    poll_new_xl_val_available(&mut sensor).await;

    // Read dummy data and discard it
    let _ = sensor.acceleration_raw_get().await.unwrap();

    // Read 5 sample and get the average value for each axis
    let mut val_st_on = [0_f32; 3];
    for _ in 0..5 {
        poll_new_xl_val_available(&mut sensor).await;

        let data_raw = sensor.acceleration_raw_get().await.unwrap();

        (0..3).for_each(|i| val_st_on[i] += from_fs4g_to_mg(data_raw[i]));
    }
    // Calculate the mg average values
    (0..3).for_each(|i| val_st_on[i] /= 5.);

    // Calculate the mg values for self test
    let mut test_val = [0_f32; 3];
    (0..3).for_each(|i| test_val[i] = (val_st_on[i] - val_st_off[i]).abs());

    let xl_st_result = test_val.iter().all(|v| ST_RANGE_MG.contains(v));

    // Disable Self Test
    sensor.xl_self_test_set(StXl::Disable).await.unwrap();
    // Disable sensor
    sensor.xl_data_rate_set(OdrXl::Off).await.unwrap();

    // === Gyroscope Self Test ===
    // Set Output Data Rate
    sensor.gy_data_rate_set(OdrGy::_208hz).await.unwrap();
    // Set full scale
    sensor.gy_full_scale_set(FsGy::_2000dps).await.unwrap();
    // Wait stable output
    delay.delay_ms(100).await;

    // Check if new value available
    poll_new_gy_val_available(&mut sensor).await;

    // Read dummy data and discard it
    let _ = sensor.angular_rate_raw_get().await.unwrap();
    // Read 5 sample and get the average value for each axis
    val_st_off = [0_f32; 3];
    for _ in 0..5 {
        poll_new_gy_val_available(&mut sensor).await;

        let data_raw = sensor.angular_rate_raw_get().await.unwrap();

        (0..3)
            .for_each(|i| val_st_off[i] += from_fs2000dps_to_mdps(data_raw[i]));
    }
    // Calculate the mg average values
    (0..3).for_each(|i| val_st_off[i] /= 5.);

    // Enable Self Test positive (or negative)
    sensor.gy_self_test_set(StGy::Positive).await.unwrap();
    // sensor.gy_self_test_set(StGy::Negative).await.unwrap();

    // Wait stable output
    delay.delay_ms(100).await;

    // Check if new value available
    poll_new_gy_val_available(&mut sensor).await;

    // Read dummy data and discard it
    let _ = sensor.angular_rate_raw_get().await.unwrap();

    // Read 5 sample and get the average value for each axis
    val_st_on = [0_f32; 3];
    for _ in 0..5 {
        poll_new_gy_val_available(&mut sensor).await;

        let data_raw = sensor.angular_rate_raw_get().await.unwrap();

        (0..3).for_each(|i| val_st_on[i] += from_fs2000dps_to_mdps(data_raw[i]));
    }
    // Calculate the mg average values
    (0..3).for_each(|i| val_st_on[i] /= 5.);

    // Calculate the mg values for self test
    test_val = [0_f32; 3];
    (0..3).for_each(|i| test_val[i] = (val_st_on[i] - val_st_off[i]).abs());

    let gy_st_result = test_val.iter().all(|v| ST_RANGE_MDPS.contains(v));

    // Disable self test
    sensor.gy_self_test_set(StGy::Disable).await.unwrap();
    // Disable sensor
    sensor.gy_data_rate_set(OdrGy::Off).await.unwrap();

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
        delay.delay_ms(500).await;
    }
}

#[maybe_async]
async fn poll_new_xl_val_available<B, D>(sensor: &mut Ism330dhcx<B, D, MainBank>)
where
    B: BusOperation,
    D: DelayNs + Clone
{
    loop {
        let drdy = sensor.xl_flag_data_ready_get().await.unwrap();
        if drdy == 1 {
            break;
        }
    }
}

#[maybe_async]
async fn poll_new_gy_val_available<B, D>(sensor: &mut Ism330dhcx<B, D, MainBank>)
where
    B: BusOperation,
    D: DelayNs + Clone
{
    loop {
        let drdy = sensor.gy_flag_data_ready_get().await.unwrap();
        if drdy == 1 {
            break;
        }
    }
}
