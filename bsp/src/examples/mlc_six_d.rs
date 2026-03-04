use defmt::info;
use maybe_async::maybe_async;
use crate::*;
use core::write;
use core::fmt::Display;

use crate::config::mlc_config::SIX_D;
use st_mems_reg_config_conv::ucf_entry::MemsUcfOp;

#[repr(u8)]
enum SixdEvent {
    None,
    XPointingUp,
    XPointingDown,
    YPointingDown,
    YPointingUp,
    ZPointingUp,
    ZPointingDown,
    Unknown(u8),
}

impl From<u8> for SixdEvent {
    fn from(value: u8) -> Self {
        match value {
            0 => Self::None,
            1 => Self::XPointingUp,
            2 => Self::XPointingDown,
            3 => Self::YPointingDown,
            4 => Self::YPointingUp,
            5 => Self::ZPointingUp,
            6 => Self::ZPointingDown,
            other => Self::Unknown(other),
        }
    }
}

impl Display for SixdEvent {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::None => write!(f, "None"),
            Self::XPointingUp => write!(f, "X-axis pointing up"),
            Self::XPointingDown => write!(f, "X-axis pointing down"),
            Self::YPointingUp => write!(f, "Y-axis pointing up"),
            Self::YPointingDown => write!(f, "Y-axis pointing down"),
            Self::ZPointingUp => write!(f, "Z-axis pointing up"),
            Self::ZPointingDown => write!(f, "Z-axis pointing down"),
            Self::Unknown(v) => write!(f, "Unkown event: {v}"),
        }
    }
}

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

    // Start Machine Learning Core configuration
    for ufc_line in SIX_D {
        match ufc_line.op {
            MemsUcfOp::Delay => delay.delay_ms(ufc_line.data.into()).await,
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
    pin_int1_route.mlc_int1.set_int1_mlc1(PROPERTY_ENABLE);
    sensor.pin_int1_route_set(&mut pin_int1_route).await.unwrap();
    // Configure interrupt pin mode notification
    sensor
        .int_notification_set(Lir::BasePulsedEmbLatched)
        .await
        .unwrap();

    loop {
        int_pin.wait_for_event().await;

        let status = sensor.mlc_status_get().await.unwrap();
        if status.is_mlc1() == 1 {
            let mut buff = [0_u8];
            sensor.mlc_out_get(&mut buff).await.unwrap();
            let catched: SixdEvent = buff[0].into();
            writeln!(tx, "{catched}").unwrap();
        }
    }
}
