use defmt::info;
use maybe_async::maybe_async;
use crate::*;

use crate::config::fsm_config::FREE_FALL;
use st_mems_reg_config_conv::ucf_entry::MemsUcfOp;

#[maybe_async]
pub async fn run<B, D, L, I>(bus: B, mut tx: L, mut delay: D, mut int_pin : I) -> !
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

    // Restore default configuration
    sensor.reset_set(PROPERTY_ENABLE).await.unwrap();

    loop {
        let rst = sensor.reset_get().await.unwrap();
        if rst == 0 {
            break;
        }
    }

    // Start FSM configuration
    for ufc_line in FREE_FALL {
        match ufc_line.op {
            MemsUcfOp::Delay => sensor.tim.delay_ms(ufc_line.data.into()).await,
            MemsUcfOp::Write => sensor
                .bus
                .write_to_register(ufc_line.address, &[ufc_line.data])
                .await
                .unwrap(),
            _ => {}
        }
    }

    // Route signals on interrupt pin 1
    let mut pin_int1_route = sensor.pin_int1_route_get().await.unwrap();
    pin_int1_route.fsm_int1_a.set_int1_fsm1(PROPERTY_ENABLE);
    sensor.pin_int1_route_set(&mut pin_int1_route).await.unwrap();
    // Configure interrupt pin mode notification
    sensor
        .int_notification_set(Lir::BasePulsedEmbLatched)
        .await
        .unwrap();

    loop {
        int_pin.wait_for_event().await;

        let status = sensor.all_sources_get().await.unwrap();
        if status.fsm_status_a.is_fsm1() == 1 {
            writeln!(tx, "Free fall detected").unwrap();
        }
    }
}
