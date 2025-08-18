#![no_main]
#![no_std]

use core::{
    cell::RefCell,
    fmt::{Display, Write},
};

use cortex_m::interrupt::{free, Mutex};

use ism330dhcx_rs::prelude::*;
use ism330dhcx_rs::{Ism330dhcx, PROPERTY_ENABLE};
use panic_itm as _;

use cortex_m_rt::entry;
use stm32f4xx_hal::{
    gpio::{self, Edge, Input},
    hal::delay::DelayNs,
    i2c::{DutyCycle, I2c, Mode},
    pac::{self, interrupt},
    prelude::*,
    serial::Config,
};

mod mlc_config;
use mlc_config::SIX_D;
use st_mems_reg_config_conv::ucf_entry::MemsUcfOp;
type IntPin = gpio::PB10<Input>;

static INT_PIN: Mutex<RefCell<Option<IntPin>>> = Mutex::new(RefCell::new(None));
static MEMS_EVENT: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));

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

#[entry]
fn main() -> ! {
    let mut dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.use_hse(8.MHz()).sysclk(48.MHz()).freeze();

    let mut delay = cp.SYST.delay(&clocks);

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

    let mut syscfg = dp.SYSCFG.constrain();

    let mut int_pin = gpiob.pb10.into_input();
    // Configure pin for interrupts
    int_pin.make_interrupt_source(&mut syscfg);
    int_pin.trigger_on_edge(&mut dp.EXTI, Edge::Rising);
    int_pin.enable_interrupt(&mut dp.EXTI);

    // Enable interrupts
    let int_pin_num = int_pin.interrupt();
    pac::NVIC::unpend(int_pin_num);
    unsafe {
        pac::NVIC::unmask(int_pin_num);
    };

    free(|cs| INT_PIN.borrow(cs).replace(Some(int_pin)));

    delay.delay_ms(5);

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

    // Start Matchine Learning Core configuration
    for ufc_line in SIX_D {
        match ufc_line.op {
            MemsUcfOp::Delay => sensor.tim.delay_ms(ufc_line.data.into()),
            MemsUcfOp::Write => sensor
                .write_to_register(ufc_line.address, &[ufc_line.data])
                .unwrap(),
            _ => {}
        }
    }

    // Route signals on interrupt pin 1
    let mut pin_int1_route = sensor.pin_int1_route_get().unwrap();
    pin_int1_route.mlc_int1.set_int1_mlc1(PROPERTY_ENABLE);
    sensor.pin_int1_route_set(&mut pin_int1_route).unwrap();
    // Configure interrupt pin mode notification
    sensor
        .int_notification_set(Lir::BasePulsedEmbLatched)
        .unwrap();

    loop {
        // Wait for interrupt
        let mems_event = cortex_m::interrupt::free(|cs| {
            let flag = *MEMS_EVENT.borrow(cs).borrow();
            if flag {
                MEMS_EVENT.borrow(cs).replace(false);
            }
            flag
        });
        if !mems_event {
            continue;
        }
        let status = sensor.mlc_status_get().unwrap();
        if status.is_mlc1() == 1 {
            let mut buff = [0_u8];
            sensor.mlc_out_get(&mut buff).unwrap();
            let catched: SixdEvent = buff[0].into();
            writeln!(tx, "{catched}").unwrap();
        }
    }
}

#[interrupt]
fn EXTI15_10() {
    // Start a Critical Section
    cortex_m::interrupt::free(|cs| {
        // Obtain access to Peripheral and Clear Interrupt Pending Flag
        let mut int_pin = INT_PIN.borrow(cs).borrow_mut();
        if int_pin.as_mut().unwrap().check_interrupt() {
            int_pin.as_mut().unwrap().clear_interrupt_pending_bit();
        }
        MEMS_EVENT.borrow(cs).replace(true);
    });
}
