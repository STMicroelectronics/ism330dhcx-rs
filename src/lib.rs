#![no_std]
use core::fmt::Debug;
use embedded_hal::{
    delay::DelayNs,
    i2c::{I2c, SevenBitAddress},
    spi::SpiDevice,
};
use st_mems_bus::{BusOperation, EmbAdvFunctions, MemBankFunctions};
use half::f16;

pub mod prelude;
pub mod register;

use prelude::*;

/// The Ism330dhcx generic driver struct.
pub struct Ism330dhcx<B, T> {
    /// The bus driver.
    pub bus: B,
    pub tim: T,
}

/// Driver errors.
#[derive(Debug)]
pub enum Error<B> {
    Bus(B), // Error at the bus level
    InvalidFsmNumber,
    BufferTooSmall,
    FailedToReadMemBank,
    FailedToSetMemBank(MemBank),
}

impl<P, T> Ism330dhcx<st_mems_bus::i2c::I2cBus<P>, T>
where
    P: I2c,
    T: DelayNs,
{
    /// Constructor method for using the I2C bus.
    ///
    /// # Arguments
    ///
    /// * `i2c`: The I2C peripheral.
    /// * `address`: The I2C address of the Ism330dhcx sensor.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `Self`: Returns an instance of `Ism330dhcx`.
    ///     * `Err`: Returns an error if the initialization fails.
    pub fn new_i2c(i2c: P, address: I2CAddress, tim: T) -> Self {
        // Initialize the I2C bus with the Ism330dhcx address
        let bus = st_mems_bus::i2c::I2cBus::new(i2c, address as SevenBitAddress);
        Self { bus, tim }
    }
}

impl<P, T> Ism330dhcx<st_mems_bus::spi::SpiBus<P>, T>
where
    P: SpiDevice,
    T: DelayNs,
{
    /// Constructor method for using the SPI bus.
    ///
    /// # Arguments
    ///
    /// * `spi`: The SPI peripheral.
    ///
    /// # Returns
    ///
    /// * `Self`: Returns an instance of `Ism330dhcx`.
    pub fn new_spi(spi: P, tim: T) -> Self {
        // Initialize the SPI bus
        let bus = st_mems_bus::spi::SpiBus::new(spi);
        Self { bus, tim }
    }
}

impl<B: BusOperation, T: DelayNs> MemBankFunctions<MemBank> for Ism330dhcx<B, T> {
    type Error = Error<B::Error>;

    /// Enable access to the embedded functions/sensor hub configuration
    /// registers.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of reg_access in reg FUNC_CFG_ACCESS.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    fn mem_bank_set(&mut self, val: MemBank) -> Result<(), Self::Error> {
        let mut func_cfg_access =
            FuncCfgAccess::read(self).map_err(|_| Error::FailedToReadMemBank)?;
        func_cfg_access.set_reg_access(val as u8);
        func_cfg_access
            .write(self)
            .map_err(|_| Error::FailedToSetMemBank(val))
    }

    /// Enable access to the embedded functions/sensor hub configuration
    /// registers..
    ///
    /// # Arguments
    ///
    /// * `val`: Get the values of reg_access in reg FUNC_CFG_ACCESS.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`: Interface status (MANDATORY: return Ok(()) -> no Error).
    fn mem_bank_get(&mut self) -> Result<MemBank, Self::Error> {
        let val = FuncCfgAccess::read(self)?.reg_access();
        Ok(MemBank::try_from(val).unwrap_or_default())
    }
}

const PAGE_WRITE_ENABLE: u8 = 0x02;
const PAGE_READ_ENABLE: u8 = 0x01;
const PAGE_RW_DISABLE: u8 = 0x00;

impl<B: BusOperation, T: DelayNs> EmbAdvFunctions for Ism330dhcx<B, T> {
    type Error = Error<B::Error>;

    /// Write buffer in a page.
    ///
    /// # Arguments
    ///
    /// * `add`: Page line address.
    /// * `buf`: Value to write.
    /// * `len`: Buffer length.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`: Interface status (MANDATORY: return Ok(()) -> no Error).
    fn ln_pg_write(&mut self, add: u16, buf: &[u8], len: u8) -> Result<(), Self::Error> {
        MemBank::operate_over_emb(self, |state| {
            let [mut lsb, mut msb] = add.to_le_bytes();

            // Select page write
            let mut page_rw = PageRw::read(state)?;
            page_rw.set_page_rw(PAGE_WRITE_ENABLE);
            page_rw.write(state)?;

            // Set page address
            let mut page_sel = PageSel::read(state)?;
            page_sel.set_page_sel(msb);
            page_sel.write(state)?;

            PageAddress::from_bits(lsb).write(state)?;

            for &item in buf.iter().take(len as usize) {
                PageValue::from_bits(item).write(state)?;

                lsb = lsb.wrapping_add(1);
                // check if page wrap
                if lsb == 0x00 {
                    msb += 1;
                    page_sel = PageSel::read(state)?;
                    page_sel.set_page_sel(msb);
                    page_sel.write(state)?;
                }
            }

            // Reset page selection
            page_sel = PageSel::read(state)?;
            page_sel.set_page_sel(0);
            page_sel.write(state)?;

            // Unset page write
            let mut page_rw = PageRw::read(state)?;
            page_rw.set_page_rw(PAGE_RW_DISABLE);
            page_rw.write(state)
        })
    }

    /// Read a line(byte) in a page.
    ///
    /// # Arguments
    ///
    /// * `add`: Page line address.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    fn ln_pg_read(&mut self, add: u16, buf: &mut [u8], len: u8) -> Result<(), Self::Error> {
        MemBank::operate_over_emb(self, |lock| {
            let [mut lsb, mut msb] = add.to_le_bytes();

            // Set page read
            let mut page_rw = PageRw::read(lock)?;
            page_rw.set_page_rw(PAGE_READ_ENABLE);
            page_rw.write(lock)?;

            // Select page
            let mut page_sel = PageSel::read(lock)?;
            page_sel.set_page_sel(msb);
            page_sel.write(lock)?;

            // Set page address
            let mut page_address = PageAddress::from_bits(0);
            page_address.set_page_addr(lsb);
            page_address.write(lock)?;

            for i in 0..len {
                lock.read_from_register(
                    EmbReg::PageValue as u8,
                    &mut buf[i as usize..(i as usize + 1)],
                )?;

                lsb = lsb.wrapping_add(1);
                // Check if page wrap
                if lsb == 0x00 {
                    msb += 1;
                    page_sel = PageSel::read(lock)?;
                    page_sel.set_page_sel(msb);
                    page_sel.write(lock)?;
                }
            }

            // Reset page selection
            page_sel = PageSel::read(lock)?;
            page_sel.set_page_sel(0);
            page_sel.write(lock)?;

            // Unset page read
            page_rw = PageRw::read(lock)?;
            page_rw.set_page_rw(PAGE_RW_DISABLE);
            page_rw.write(lock)
        })
    }
}

impl<B: BusOperation, T: DelayNs> Ism330dhcx<B, T> {
    #[inline]
    pub fn read_from_register(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<B::Error>> {
        self.bus.read_from_register(reg, buf).map_err(Error::Bus)
    }

    #[inline]
    pub fn write_to_register(&mut self, reg: u8, buf: &[u8]) -> Result<(), Error<B::Error>> {
        self.bus.write_to_register(reg, buf).map_err(Error::Bus)
    }

    #[inline]
    pub fn from_bus(bus: B, tim: T) -> Self {
        Self { bus, tim }
    }

    /// Accelerometer full-scale selection.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of fs_xl in reg CTRL1_XL.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn xl_full_scale_set(&mut self, val: FsXl) -> Result<(), Error<B::Error>> {
        let mut ctrl1_xl = Ctrl1Xl::read(self)?;
        ctrl1_xl.set_fs_xl(val as u8);
        ctrl1_xl.write(self)
    }

    /// Accelerometer full-scale selection.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `Ism330dhcxFsXl`: Get the values of fs_xl in reg CTRL1_XL.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn xl_full_scale_get(&mut self) -> Result<FsXl, Error<B::Error>> {
        let ctrl1_xl = Ctrl1Xl::read(self)?;
        Ok(FsXl::try_from(ctrl1_xl.fs_xl()).unwrap_or_default())
    }

    /// Accelerometer UI data rate selection.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of odr_xl in reg CTRL1_XL.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn xl_data_rate_set(&mut self, mut val: OdrXl) -> Result<(), Error<B::Error>> {
        let fsm_enable = self.fsm_enable_get()?;
        let mlc_enable = self.mlc_get()?;

        val = if fsm_enable.fsm_enable_a.into_bits() != PROPERTY_DISABLE
            || fsm_enable.fsm_enable_b.into_bits() != PROPERTY_DISABLE
        {
            let fsm_odr = self.fsm_data_rate_get()?;
            match fsm_odr {
                FsmOdr::_12_5hz => match val {
                    OdrXl::Off | OdrXl::_1_6hz  => OdrXl::_12_5hz,
                    _ => val,
                },
                FsmOdr::_26hz => match val {
                    OdrXl::Off | OdrXl::_1_6hz | OdrXl::_12_5hz => OdrXl::_26hz,
                    _ => val,
                },
                FsmOdr::_52hz => match val {
                    OdrXl::Off | OdrXl::_1_6hz | OdrXl::_12_5hz | OdrXl::_26hz => OdrXl::_52hz,
                    _ => val,
                },
                FsmOdr::_104hz => match val {
                    OdrXl::Off | OdrXl::_1_6hz | OdrXl::_12_5hz | OdrXl::_26hz | OdrXl::_52hz => OdrXl::_104hz,
                    _ => val,
                },
            }
        } else {
            val
        };
        val = if mlc_enable == PROPERTY_ENABLE {
            let mlc_odr = self.mlc_data_rate_get()?;
            match mlc_odr {
                MlcOdr::_12_5hz => match val {
                    OdrXl::Off | OdrXl::_1_6hz => OdrXl::_12_5hz,
                    _ => val,
                },
                MlcOdr::_26hz => match val {
                    OdrXl::Off | OdrXl::_1_6hz | OdrXl::_12_5hz => OdrXl::_26hz,
                    _ => val,
                },
                MlcOdr::_52hz => match val {
                    OdrXl::Off | OdrXl::_1_6hz | OdrXl::_12_5hz | OdrXl::_26hz => OdrXl::_52hz,
                    _ => val,
                },
                MlcOdr::_104hz => match val {
                    OdrXl::Off | OdrXl::_1_6hz | OdrXl::_12_5hz | OdrXl::_26hz | OdrXl::_52hz => OdrXl::_104hz,
                    _ => val,
                },
            }
        } else {
            val
        };

        let mut ctrl1xl = Ctrl1Xl::read(self)?;
        ctrl1xl.set_odr_xl(val as u8);
        ctrl1xl.write(self)
    }

    /// Accelerometer UI data rate selection.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `OdrXl`: Get the values of odr_xl in reg CTRL1_XL.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn xl_data_rate_get(&mut self) -> Result<OdrXl, Error<B::Error>> {
        let ctrl1xl = Ctrl1Xl::read(self)?;
        Ok(OdrXl::try_from(ctrl1xl.odr_xl()).unwrap_or_default())
    }

    /// Gyroscope UI chain full-scale selection.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of fs_g in reg CTRL2_G.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn gy_full_scale_set(&mut self, val: FsGy) -> Result<(), Error<B::Error>> {
        let mut ctrl2_g = Ctrl2G::read(self)?;
        ctrl2_g.set_fs_g(val as u8);
        ctrl2_g.write(self)
    }

    /// Gyroscope UI chain full-scale selection.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `FsG`: Get the values of fs_g in reg CTRL2_G.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn gy_full_scale_get(&mut self) -> Result<FsGy, Error<B::Error>> {
        let ctrl2_g = Ctrl2G::read(self)?;
        Ok(FsGy::try_from(ctrl2_g.fs_g()).unwrap_or_default())
    }

    /// Gyroscope data rate.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of odr_g in reg CTRL2_G.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn gy_data_rate_set(&mut self, mut val: OdrGy) -> Result<(), Error<B::Error>> {
        let fsm_enable = self.fsm_enable_get()?;
        let mlc_enable = self.mlc_get()?;

        val = if fsm_enable.fsm_enable_a.into_bits() != PROPERTY_DISABLE
            || fsm_enable.fsm_enable_b.into_bits() != PROPERTY_DISABLE
        {
            let fsm_odr = self.fsm_data_rate_get()?;
            match fsm_odr {
                FsmOdr::_12_5hz => match val {
                    OdrGy::Off => OdrGy::_12_5hz,
                    _ => val,
                },
                FsmOdr::_26hz => match val {
                    OdrGy::Off | OdrGy::_12_5hz => OdrGy::_26hz,
                    _ => val,
                },
                FsmOdr::_52hz => match val {
                    OdrGy::Off | OdrGy::_12_5hz | OdrGy::_26hz => OdrGy::_52hz,
                    _ => val,
                },
                FsmOdr::_104hz => match val {
                    OdrGy::Off | OdrGy::_12_5hz | OdrGy::_26hz | OdrGy::_52hz => OdrGy::_104hz,
                    _ => val,
                },
            }
        } else {
            val
        };
        val = if mlc_enable == PROPERTY_ENABLE {
            let mlc_odr = self.mlc_data_rate_get()?;
            match mlc_odr {
                MlcOdr::_12_5hz => match val {
                    OdrGy::Off => OdrGy::_12_5hz,
                    _ => val,
                },
                MlcOdr::_26hz => match val {
                    OdrGy::Off | OdrGy::_12_5hz => OdrGy::_26hz,
                    _ => val,
                },
                MlcOdr::_52hz => match val {
                    OdrGy::Off | OdrGy::_12_5hz | OdrGy::_26hz => OdrGy::_52hz,
                    _ => val,
                },
                MlcOdr::_104hz => match val {
                    OdrGy::Off | OdrGy::_12_5hz | OdrGy::_26hz | OdrGy::_52hz => OdrGy::_104hz,
                    _ => val,
                },
            }
        } else {
            val
        };

        let mut ctrl2g = Ctrl2G::read(self)?;
        ctrl2g.set_odr_g(val as u8);
        ctrl2g.write(self)
    }

    /// Gyroscope data rate.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `OdrG`: Get the values of odr_g in reg CTRL2_G.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn gy_data_rate_get(&mut self) -> Result<OdrGy, Error<B::Error>> {
        let ctrl2_g = Ctrl2G::read(self)?;
        Ok(OdrGy::try_from(ctrl2_g.odr_g()).unwrap_or_default())
    }

    /// Block data update.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of bdu in reg CTRL3_C.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`: No Error.
    pub fn block_data_update_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl3_c = Ctrl3C::read(self)?;
        ctrl3_c.set_bdu(val);
        ctrl3_c.write(self)
    }

    /// Block data update.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of bdu in reg CTRL3_C.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn block_data_update_get(&mut self) -> Result<u8, Error<B::Error>> {
        let ctrl3_c = Ctrl3C::read(self)?;
        Ok(ctrl3_c.bdu())
    }

    /// Weight of XL user offset bits of registers X_OFS_USR (73h),
    /// Y_OFS_USR (74h), Z_OFS_USR (75h).
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of usr_off_w in reg CTRL6_C.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn xl_offset_weight_set(&mut self, val: UsrOffW) -> Result<(), Error<B::Error>> {
        let mut ctrl6_c = Ctrl6C::read(self)?;
        ctrl6_c.set_usr_off_w(val as u8);
        ctrl6_c.write(self)
    }

    /// Weight of XL user offset bits of registers X_OFS_USR (73h),
    /// Y_OFS_USR (74h), Z_OFS_USR (75h).
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `UsrOffW`: Get the values of usr_off_w in reg CTRL6_C.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn xl_offset_weight_get(&mut self) -> Result<UsrOffW, Error<B::Error>> {
        let ctrl6_c = Ctrl6C::read(self)?;
        Ok(UsrOffW::try_from(ctrl6_c.usr_off_w()).unwrap_or_default())
    }

    /// Accelerometer power mode.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of xl_hm_mode in reg CTRL6_C.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn xl_power_mode_set(&mut self, val: XlHmMode) -> Result<(), Error<B::Error>> {
        let mut ctrl6_c = Ctrl6C::read(self)?;
        ctrl6_c.set_xl_hm_mode((val as u8) & 0x01);
        ctrl6_c.write(self)
    }

    /// Accelerometer power mode.
    ///
    /// # Arguments
    ///
    /// * `val`: Get the values of xl_hm_mode in reg CTRL6_C.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`.
    pub fn xl_power_mode_get(&mut self) -> Result<XlHmMode, Error<B::Error>> {
        let ctrl6_c = Ctrl6C::read(self)?;
        Ok(XlHmMode::try_from(ctrl6_c.xl_hm_mode()).unwrap_or_default())
    }

    /// Operating mode for gyroscope.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of g_hm_mode in reg CTRL7_G.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn gy_power_mode_set(&mut self, val: GyHmMode) -> Result<(), Error<B::Error>> {
        let mut reg = Ctrl7G::read(self)?;
        reg.set_g_hm_mode(val as u8);
        reg.write(self)
    }

    /// Operating mode for gyroscope.
    ///
    /// # Arguments
    ///
    /// * `val`: Get the values of g_hm_mode in reg CTRL7_G.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn gy_power_mode_get(&mut self) -> Result<GyHmMode, Error<B::Error>> {
        let ctrl7_g = Ctrl7G::read(self)?;
        Ok(GyHmMode::try_from(ctrl7_g.g_hm_mode()).unwrap_or_default())
    }

    /// Read all the interrupt flag of the device.
    ///
    /// # Arguments
    ///
    /// * `val`: Get registers ALL_INT_SRC; WAKE_UP_SRC; TAP_SRC; D6D_SRC; STATUS_REG; EMB_FUNC_STATUS; FSM_STATUS_A/B.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`: No Error.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn all_sources_get(&mut self) -> Result<AllSources, Error<B::Error>> {
        let mut sources = AllSources {
            all_int_src: AllIntSrc::read(self)?,
            wake_up_src: WakeUpSrc::read(self)?,
            tap_src: TapSrc::read(self)?,
            d6d_src: D6dSrc::read(self)?,
            status_reg: StatusReg::read(self)?,
            mlc_status: MlcStatusMainpage::read(self)?,
            ..Default::default()
        };

        MemBank::operate_over_emb(self, |state| {
            sources.emb_func_status = EmbFuncStatus::read(state)?;
            sources.fsm_status_a = FsmStatusA::read(state)?;
            sources.fsm_status_b = FsmStatusB::read(state)?;
            Ok(())
        })?;

        Ok(sources)
    }

    /// The STATUS_REG register is read by the primary interface.
    pub fn status_reg_get(&mut self) -> Result<StatusReg, Error<B::Error>> {
        StatusReg::read(self)
    }

    /// Accelerometer new data available.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of xlda in reg STATUS_REG.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn xl_flag_data_ready_get(&mut self) -> Result<u8, Error<B::Error>> {
        Ok(self.status_reg_get()?.xlda())
    }

    /// Gyroscope new data available.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of gda in reg STATUS_REG.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn gy_flag_data_ready_get(&mut self) -> Result<u8, Error<B::Error>> {
        Ok(self.status_reg_get()?.gda())
    }

    /// Temperature new data available.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of tda in reg STATUS_REG.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn temp_flag_data_ready_get(&mut self) -> Result<u8, Error<B::Error>> {
        Ok(self.status_reg_get()?.tda())
    }

    /// Accelerometer X-axis user offset correction expressed in two's
    /// complement, weight depends on USR_OFF_W in CTRL6_C (15h).
    /// The value must be in the range [-127 127].
    ///
    /// # Arguments
    ///
    /// * `val`: Buffer that contains data to write.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn xl_usr_offset_x_set(&mut self, val: i8) -> Result<(), Error<B::Error>> {
        XOfsUsr::new()
            .with_x_ofs_usr(val)
            .write(self)
    }

    /// Accelerometer X-axis user offset correction expressed in two's
    /// complement, weight depends on USR_OFF_W in CTRL6_C (15h).
    /// The value must be in the range [-127 127].
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `i8`: Buffer that stores data read.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn xl_usr_offset_x_get(&mut self) -> Result<i8, Error<B::Error>> {
        XOfsUsr::read(self)
            .map(|reg| reg.x_ofs_usr())
    }

    /// Accelerometer Y-axis user offset correction expressed in two's
    /// complement, weight depends on USR_OFF_W in CTRL6_C (15h).
    /// The value must be in the range [-127 127].
    ///
    /// # Arguments
    ///
    /// * `val`: Buffer that contains data to write.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn xl_usr_offset_y_set(&mut self, val: i8) -> Result<(), Error<B::Error>> {
        YOfsUsr::new()
            .with_y_ofs_usr(val)
            .write(self)
    }

    /// Accelerometer Y-axis user offset correction expressed in two's
    /// complement, weight depends on USR_OFF_W in CTRL6_C (15h).
    /// The value must be in the range [-127 127].
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `i8`: Buffer that stores data read.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn xl_usr_offset_y_get(&mut self) -> Result<i8, Error<B::Error>> {
        YOfsUsr::read(self)
            .map(|reg| reg.y_ofs_usr())
    }

    /// Accelerometer Z-axis user offset correction expressed in two's
    /// complement, weight depends on USR_OFF_W in CTRL6_C (15h).
    /// The value must be in the range [-127 127].
    ///
    /// # Arguments
    ///
    /// * `val`: Buffer that contains data to write.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn xl_usr_offset_z_set(&mut self, val: i8) -> Result<(), Error<B::Error>> {
        ZOfsUsr::new()
            .with_z_ofs_usr(val)
            .write(self)
    }

    /// Accelerometer X-axis user offset correction expressed in two's
    /// complement, weight depends on USR_OFF_W in CTRL6_C (15h).
    /// The value must be in the range [-127 127].
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `i8`: Buffer that stores data read.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn xl_usr_offset_z_get(&mut self) -> Result<i8, Error<B::Error>> {
        ZOfsUsr::read(self)
            .map(|reg| reg.z_ofs_usr())
    }

    /// Enables user offset on out.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of usr_off_on_out in reg CTRL7_G.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn xl_usr_offset_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl7_g = Ctrl7G::read(self)?;
        ctrl7_g.set_usr_off_on_out(val);
        ctrl7_g.write(self)
    }

    /// Get user offset on out flag.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Get values of usr_off_on_out in reg CTRL7_G.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn xl_usr_offset_get(&mut self) -> Result<u8, Error<B::Error>> {
        let ctrl7_g = Ctrl7G::read(self)?;
        Ok(ctrl7_g.usr_off_on_out())
    }

    /// Reset timestamp counter.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn timestamp_rst(&mut self) -> Result<(), Error<B::Error>> {
        self.write_to_register(Reg::Timestamp2 as u8, &[0xAA])?;
        self.tim.delay_us(150); // AN5398 Section 6.4
        Ok(())
    }

    /// Enables timestamp counter.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of timestamp_en in reg CTRL10_C.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn timestamp_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl10_c = Ctrl10C::read(self)?;
        ctrl10_c.set_timestamp_en(val);
        ctrl10_c.write(self)
    }

    /// Enables timestamp counter.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of timestamp_en in reg CTRL10_C.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn timestamp_get(&mut self) -> Result<u8, Error<B::Error>> {
        let ctrl10_c = Ctrl10C::read(self)?;
        Ok(ctrl10_c.timestamp_en())
    }

    /// Timestamp first data output register (r).
    /// The value is expressed as a 32-bit word and the bit resolution
    /// is 25 us.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u32`: Buffer that stores data read.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn timestamp_raw_get(&mut self) -> Result<u32, Error<B::Error>> {
        Ok(TimestampReg::read(self)?.timestamp())
    }

    /// Circular burst-mode (rounding) read of the output registers.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of rounding in reg CTRL5_C.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn rounding_mode_set(&mut self, val: Rounding) -> Result<(), Error<B::Error>> {
        let mut ctrl5_c = Ctrl5C::read(self)?;
        ctrl5_c.set_rounding(val as u8);
        ctrl5_c.write(self)
    }

    /// Gyroscope UI chain full-scale selection.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `Rounding`: Get the values of rounding in reg CTRL5_C.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn rounding_mode_get(&mut self) -> Result<Rounding, Error<B::Error>> {
        let ctrl5_c = Ctrl5C::read(self)?;
        Ok(Rounding::try_from(ctrl5_c.rounding()).unwrap_or_default())
    }

    /// Temperature data output register (r).
    /// L and H registers together express a 16-bit word in two's
    /// complement.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `i16`: Buffer that stores data read.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn temperature_raw_get(&mut self) -> Result<i16, Error<B::Error>> {
        Ok(OutTempReg::read(self)?.temp())
    }

    /// Angular rate sensor. The value is expressed as a 16-bit
    /// word in two's complement.
    ///
    /// # Arguments
    ///
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `[i16;3]`: Buffer that stores data read.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn angular_rate_raw_get(&mut self) -> Result<[i16; 3], Error<B::Error>> {
        let val = OutXYZG::read(self)?;

        Ok([val.x, val.y, val.z])
    }

    /// Linear acceleration output register. The value is expressed as a
    /// 16-bit word in two's complement.
    ///
    /// # Arguments
    ///
    /// * `buff`: Buffer that stores data read.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn acceleration_raw_get(&mut self) -> Result<[i16; 3], Error<B::Error>> {
        let val = OutXYZA::read(self)?;

        Ok([val.x, val.y, val.z])
    }

    /// FIFO data output.
    ///
    /// # Arguments
    ///
    /// * `buff`: Buffer that stores data read.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`.
    pub fn fifo_out_raw_get(&mut self) -> Result<[u8; 6], Error<B::Error>> {
        let mut buff = [0_u8; 6];
        self.read_from_register(Reg::FifoDataOutXL as u8, &mut buff)?;
        Ok(buff)
    }

    /// Step counter output register.
    ///
    /// # Arguments
    ///
    /// * `buff`: Buffer that stores data read.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u16`: Number of steps.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn number_of_steps_get(&mut self) -> Result<u16, Error<B::Error>> {
        MemBank::operate_over_emb(self, |state| Ok(StepCounter::read(state)?.step()))
    }

    /// Reset step counter register.
    pub fn steps_reset(&mut self) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_emb(self, |state| {
            let mut emb_func_src = EmbFuncSrc::read(state)?;
            emb_func_src.set_pedo_rst_step(PROPERTY_ENABLE);
            emb_func_src.write(state)
        })
    }

    /// DEVICE_CONF bit configuration.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of device_conf in reg CTRL9_XL.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn device_conf_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl9_xl = Ctrl9Xl::read(self)?;
        ctrl9_xl.set_device_conf(val);
        ctrl9_xl.write(self)
    }

    /// DEVICE_CONF bit configuration
    ///
    /// # Arguments
    ///
    /// * `val`: Get the values of device_conf in reg CTRL9_XL.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Interface status (MANDATORY: return Ok(()) -> no Error).
    pub fn device_conf_get(&mut self) -> Result<u8, Error<B::Error>> {
        let ctrl9_xl = Ctrl9Xl::read(self)?;
        Ok(ctrl9_xl.device_conf())
    }

    /// Difference in percentage of the effective ODR (and timestamp rate)
    /// with respect to the typical.
    /// Step:  0.15%. 8-bit format, 2's complement.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `i8`: Change the values of freq_fine in reg INTERNAL_FREQ_FINE.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn odr_cal_reg_get(&mut self) -> Result<i8, Error<B::Error>> {
        Ok(InternalFreqFine::read(self)?.freq_fine())
    }

    /// Data-ready pulsed / latched mode.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of dataready_pulsed in reg COUNTER_BDR_REG1.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn data_ready_mode_set(&mut self, val: DatareadyPulsed) -> Result<(), Error<B::Error>> {
        let mut counter_bdr_reg1 = CounterBdrReg::read(self)?;
        counter_bdr_reg1.set_dataready_pulsed(val as u8);
        counter_bdr_reg1.write(self)
    }

    /// Data-ready pulsed / latched mode.
    ///
    /// # Arguments
    ///
    /// * `val`: Get the values of dataready_pulsed in reg COUNTER_BDR_REG1.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn data_ready_mode_get(&mut self) -> Result<DatareadyPulsed, Error<B::Error>> {
        let counter_bdr_reg1 = CounterBdrReg::read(self)?;
        Ok(DatareadyPulsed::try_from(counter_bdr_reg1.dataready_pulsed()).unwrap_or_default())
    }

    /// Device Who am I.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Buffer that stores data read.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn device_id_get(&mut self) -> Result<u8, Error<B::Error>> {
        WhoAmI::read(self).map(|reg| reg.id())
    }

    /// Software reset. Restore the default values in user registers.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of sw_reset in reg CTRL3_C.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    pub fn reset_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl3_c = Ctrl3C::read(self)?;
        ctrl3_c.set_sw_reset(val);
        ctrl3_c.write(self)
    }

    /// Software reset. Restore the default values in user registers.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of sw_reset in reg CTRL3_C.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn reset_get(&mut self) -> Result<u8, Error<B::Error>> {
        let ctrl3_c = Ctrl3C::read(self)?;
        Ok(ctrl3_c.sw_reset())
    }

    /// Register address automatically incremented during a multiple byte
    /// access with a serial interface.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of if_inc in reg CTRL3_C.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn auto_increment_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl3_c = Ctrl3C::read(self)?;
        ctrl3_c.set_if_inc(val);
        ctrl3_c.write(self)
    }

    /// Register address automatically incremented during a multiple byte
    /// access with a serial interface.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of if_inc in reg CTRL3_C.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn auto_increment_get(&mut self) -> Result<u8, Error<B::Error>> {
        let ctrl3_c = Ctrl3C::read(self)?;
        Ok(ctrl3_c.if_inc())
    }

    /// Reboot memory content. Reload the calibration parameters.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of boot in reg CTRL3_C.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    pub fn boot_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl3_c = Ctrl3C::read(self)?;
        ctrl3_c.set_boot(val);
        ctrl3_c.write(self)
    }

    /// Reboot memory content. Reload the calibration parameters.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of boot in reg CTRL3_C.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn boot_get(&mut self) -> Result<u8, Error<B::Error>> {
        let ctrl3_c = Ctrl3C::read(self)?;
        Ok(ctrl3_c.boot())
    }

    /// Linear acceleration sensor self-test enable.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of st_xl in reg CTRL5_C.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn xl_self_test_set(&mut self, val: StXl) -> Result<(), Error<B::Error>> {
        let mut ctrl5_c = Ctrl5C::read(self)?;
        ctrl5_c.set_st_xl(val as u8);
        ctrl5_c.write(self)
    }

    /// Linear acceleration sensor self-test enable.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `StXl`: Get the values of st_xl in reg CTRL5_C.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn xl_self_test_get(&mut self) -> Result<StXl, Error<B::Error>> {
        let ctrl5_c = Ctrl5C::read(self)?;
        Ok(StXl::try_from(ctrl5_c.st_xl()).unwrap_or_default())
    }

    /// Angular rate sensor self-test enable.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of st_g in reg CTRL5_C.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn gy_self_test_set(&mut self, val: StGy) -> Result<(), Error<B::Error>> {
        let mut ctrl5_c = Ctrl5C::read(self)?;
        ctrl5_c.set_st_g(val as u8);
        ctrl5_c.write(self)
    }

    /// Angular rate sensor self-test enable.
    ///
    /// # Arguments
    ///
    /// * `val`: Get the values of st_g in reg CTRL5_C.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`: Interface status (MANDATORY: return Ok(()) -> no Error).
    pub fn gy_self_test_get(&mut self) -> Result<StGy, Error<B::Error>> {
        let ctrl5_c = Ctrl5C::read(self)?;
        Ok(StGy::try_from(ctrl5_c.st_g()).unwrap_or_default())
    }

    /// Accelerometer output from LPF2 filtering stage selection.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of lpf2_xl_en in reg CTRL1_XL.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn xl_filter_lp2_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl1_xl = Ctrl1Xl::read(self)?;
        ctrl1_xl.set_lpf2_xl_en(val);
        ctrl1_xl.write(self)
    }

    /// Accelerometer output from LPF2 filtering stage selection.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of lpf2_xl_en in reg CTRL1_XL.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn xl_filter_lp2_get(&mut self) -> Result<u8, Error<B::Error>> {
        let ctrl1_xl = Ctrl1Xl::read(self)?;
        Ok(ctrl1_xl.lpf2_xl_en())
    }

    /// Enables gyroscope digital LPF1 if auxiliary SPI is disabled;
    /// the bandwidth can be selected through FTYPE \[2:0\] in CTRL6_C.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of lpf1_sel_g in reg CTRL4_C.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn gy_filter_lp1_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl4_c = Ctrl4C::read(self)?;
        ctrl4_c.set_lpf1_sel_g(val);
        ctrl4_c.write(self)
    }

    /// Enables gyroscope digital LPF1 if auxiliary SPI is disabled;
    /// the bandwidth can be selected through FTYPE \[2:0\] in CTRL6_C.
    pub fn gy_filter_lp1_get(&mut self) -> Result<u8, Error<B::Error>> {
        let ctrl4_c = Ctrl4C::read(self)?;
        Ok(ctrl4_c.lpf1_sel_g())
    }

    /// Mask DRDY on pin (both XL & Gyro) until filter settling ends
    /// (XL and Gyro independently masked).
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of drdy_mask in reg CTRL4_C.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn drdy_mask_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl4_c = Ctrl4C::read(self)?;
        ctrl4_c.set_drdy_mask(val);
        ctrl4_c.write(self)
    }

    /// Mask DRDY on pin (both XL & Gyro) until filter settling ends
    /// (XL and Gyro independently masked).
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of drdy_mask in reg CTRL4_C.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn drdy_mask_get(&mut self) -> Result<u8, Error<B::Error>> {
        let ctrl4_c = Ctrl4C::read(self)?;
        Ok(ctrl4_c.drdy_mask())
    }

    /// Gyroscope low pass filter 1 bandwidth.
    /// See Table 58 on Datasheet for more information
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of ftype in reg CTRL6_C.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn gy_lp1_bandwidth_set(&mut self, val: Ftype) -> Result<(), Error<B::Error>> {
        let mut ctrl6_c = Ctrl6C::read(self)?;
        ctrl6_c.set_ftype(val as u8);
        ctrl6_c.write(self)
    }

    /// Gyroscope low pass filter 1 bandwidth.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `Ftype`: Get the values of ftype in reg CTRL6_C.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn gy_lp1_bandwidth_get(&mut self) -> Result<Ftype, Error<B::Error>> {
        let ctrl6_c = Ctrl6C::read(self)?;
        Ok(Ftype::try_from(ctrl6_c.ftype()).unwrap_or_default())
    }

    /// Low pass filter 2 on 6D function selection.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of low_pass_on_6d in reg CTRL8_XL.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn xl_lp2_on_6d_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl8_xl = Ctrl8Xl::read(self)?;
        ctrl8_xl.set_low_pass_on_6d(val);
        ctrl8_xl.write(self)
    }

    /// Low pass filter 2 on 6D function selection.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of low_pass_on_6d in reg CTRL8_XL.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn xl_lp2_on_6d_get(&mut self) -> Result<u8, Error<B::Error>> {
        let ctrl8_xl = Ctrl8Xl::read(self)?;
        Ok(ctrl8_xl.low_pass_on_6d())
    }

    /// Accelerometer slope filter / high-pass filter selection on output.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of hp_slope_xl_en in reg CTRL8_XL.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn xl_hp_path_on_out_set(&mut self, val: HpSlopeXlEn) -> Result<(), Error<B::Error>> {
        let mut ctrl8_xl = Ctrl8Xl::read(self)?;

        ctrl8_xl.set_hp_slope_xl_en(((val as u8) & 0x10) >> 4);
        ctrl8_xl.set_hp_ref_mode_xl(((val as u8) & 0x20) >> 5);
        ctrl8_xl.set_hpcf_xl((val as u8) & 0x07);

        ctrl8_xl.write(self)
    }

    /// Accelerometer slope filter / high-pass filter selection on output.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `HpSlopeXlEn`: Get the values of hp_slope_xl_en in reg CTRL8_XL.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn xl_hp_path_on_out_get(&mut self) -> Result<HpSlopeXlEn, Error<B::Error>> {
        let ctrl8_xl = Ctrl8Xl::read(self)?;

        let value = (ctrl8_xl.hp_ref_mode_xl() << 5)
            + (ctrl8_xl.hp_slope_xl_en() << 4)
            + ctrl8_xl.hpcf_xl();
        Ok(HpSlopeXlEn::try_from(value).unwrap_or_default())
    }

    /// Enables accelerometer LPF2 and HPF fast-settling mode.
    /// The filter sets the second samples after writing this bit.
    /// Active only during device exit from powerdown mode.
    pub fn xl_fast_settling_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl8_xl = Ctrl8Xl::read(self)?;
        ctrl8_xl.set_fastsettl_mode_xl(val);
        ctrl8_xl.write(self)
    }

    /// Enables accelerometer LPF2 and HPF fast-settling mode.
    /// The filter sets the second samples after writing
    /// this bit. Active only during device exit from powerdown mode.
    pub fn xl_fast_settling_get(&mut self) -> Result<u8, Error<B::Error>> {
        let ctrl8_xl = Ctrl8Xl::read(self)?;
        Ok(ctrl8_xl.fastsettl_mode_xl())
    }

    /// HPF or SLOPE filter selection on wake-up and Activity/Inactivity functions.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of slope_fds in reg TAP_CFG0.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn xl_hp_path_internal_set(&mut self, val: SlopeFds) -> Result<(), Error<B::Error>> {
        let mut tap_cfg0 = TapCfg0::read(self)?;
        tap_cfg0.set_slope_fds(val as u8);
        tap_cfg0.write(self)
    }

    /// HPF or SLOPE filter selection on wake-up and Activity/Inactivity functions.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `SlopeFds`: Get the values of slope_fds in reg TAP_CFG0.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn xl_hp_path_internal_get(&mut self) -> Result<SlopeFds, Error<B::Error>> {
        let tap_cfg0 = TapCfg0::read(self)?;
        Ok(SlopeFds::try_from(tap_cfg0.slope_fds()).unwrap_or_default())
    }

    /// Enables gyroscope digital high-pass filter. The filter is enabled
    /// only if the gyro is in HP mode.
    ///
    /// # Arguments
    ///
    /// * `val`: Get the values of hp_en_g and hpm_g in reg CTRL7_G.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn gy_hp_filter_set(&mut self, val: HpmGy) -> Result<(), Error<B::Error>> {
        let mut ctrl7_g = Ctrl7G::read(self)?;
        ctrl7_g.set_hp_en_g(((val as u8) & 0x80) >> 7);
        ctrl7_g.set_hpm_g((val as u8) & 0x03);
        ctrl7_g.write(self)
    }

    /// Enables gyroscope digital high-pass filter. The filter is
    /// enabled only if the gyro is in HP mode.
    pub fn gy_hp_filter_get(&mut self) -> Result<HpmGy, Error<B::Error>> {
        let ctrl7_g = Ctrl7G::read(self)?;
        let value = (ctrl7_g.hp_en_g() << 7) + ctrl7_g.hpm_g();
        Ok(HpmGy::try_from(value).unwrap_or_default())
    }

    /// On auxiliary interface connect/disconnect SDO and OCS internal pull-up.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of ois_pu_dis in reg PIN_CTRL.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn aux_sdo_ocs_mode_set(&mut self, val: OisPuDis) -> Result<(), Error<B::Error>> {
        let mut pin_ctrl = PinCtrl::read(self)?;
        pin_ctrl.set_ois_pu_dis(val as u8);
        pin_ctrl.write(self)
    }

    /// On auxiliary interface connect/disconnect SDO and OCS internal pull-up.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `OisPuDis`: Get the values of `ois_pu_dis` in reg `PIN_CTRL`.
    pub fn aux_sdo_ocs_mode_get(&mut self) -> Result<OisPuDis, Error<B::Error>> {
        let pin_ctrl = PinCtrl::read(self)?;
        Ok(OisPuDis::try_from(pin_ctrl.ois_pu_dis()).unwrap_or_default())
    }

    /// OIS chain on aux interface power on mode.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of ois_on in reg CTRL7_G.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn aux_pw_on_ctrl_set(&mut self, val: OisOn) -> Result<(), Error<B::Error>> {
        let mut ctrl7_g = Ctrl7G::read(self)?;
        ctrl7_g.set_ois_on_en(((val as u8) & 0x02) >> 1);
        ctrl7_g.set_ois_on((val as u8) & 0x01);
        ctrl7_g.write(self)
    }

    /// OIS chain on aux interface power on mode.
    ///
    /// # Arguments
    ///
    /// * `val`: Get the values of ois_on in reg CTRL7_G.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`: Interface status (MANDATORY: return Ok(()) -> no Error).
    pub fn aux_pw_on_ctrl_get(&mut self) -> Result<OisOn, Error<B::Error>> {
        let ctrl7_g = Ctrl7G::read(self)?;
        let val = ctrl7_g.ois_on() | (ctrl7_g.ois_on_en() << 1);
        Ok(OisOn::try_from(val).unwrap_or_default())
    }

    /// The STATUS_SPIAux register is read by the auxiliary SPI.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `StatusSpiaux`: registers STATUS_SPIAUX.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn aux_status_reg_get(&mut self) -> Result<StatusSpiaux, Error<B::Error>> {
        StatusSpiaux::read(self)
    }

    /// AUX accelerometer data available.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of xlda in reg STATUS_SPIAUX.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn aux_xl_flag_data_ready_get(&mut self) -> Result<u8, Error<B::Error>> {
        Ok(self.aux_status_reg_get()?.xlda())
    }

    /// AUX gyroscope data available.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of gda in reg STATUS_SPIAUX.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn aux_gy_flag_data_ready_get(&mut self) -> Result<u8, Error<B::Error>> {
        Ok(self.aux_status_reg_get()?.gda())
    }

    /// High when the gyroscope output is in the settling phase.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of gyro_settling in reg STATUS_SPIAUX.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn aux_gy_flag_settling_get(&mut self) -> Result<u8, Error<B::Error>> {
        Ok(self.aux_status_reg_get()?.gyro_settling())
    }

    /// Selects accelerometer self-test. Effective only if XL OIS chain is enabled.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of st_xl_ois in reg INT_OIS.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn aux_xl_self_test_set(&mut self, val: StXlOis) -> Result<(), Error<B::Error>> {
        let mut int_ois = IntOis::read(self)?;
        int_ois.set_st_xl_ois(val as u8);
        int_ois.write(self)
    }

    /// Selects accelerometer self-test. Effective only if XL OIS chain is enabled.
    ///
    /// # Arguments
    ///
    /// * `val`: Get the values of st_xl_ois in reg INT_OIS.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn aux_xl_self_test_get(&mut self) -> Result<StXlOis, Error<B::Error>> {
        let int_ois = IntOis::read(self)?;
        Ok(StXlOis::try_from(int_ois.st_xl_ois()).unwrap_or_default())
    }

    /// Indicates polarity of DEN signal on OIS chain.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of den_lh_ois in reg INT_OIS.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn aux_den_polarity_set(&mut self, val: DenLhOis) -> Result<(), Error<B::Error>> {
        let mut int_ois = IntOis::read(self)?;
        int_ois.set_den_lh_ois(val as u8);
        int_ois.write(self)
    }

    /// Indicates polarity of DEN signal on OIS chain.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `DenLhOis`: Get the values of den_lh_ois in reg INT_OIS.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn aux_den_polarity_get(&mut self) -> Result<DenLhOis, Error<B::Error>> {
        let int_ois = IntOis::read(self)?;
        Ok(DenLhOis::try_from(int_ois.den_lh_ois()).unwrap_or_default())
    }

    /// Configure DEN mode on the OIS chain.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of lvl2_ois in reg INT_OIS.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`.
    pub fn aux_den_mode_set(&mut self, val: Lvl2Ois) -> Result<(), Error<B::Error>> {
        let mut int_ois = IntOis::read(self)?;
        int_ois.set_lvl2_ois((val as u8) & 0x01);
        int_ois.write(self)?;

        let mut ctrl1_ois = Ctrl1Ois::read(self)?;
        ctrl1_ois.set_lvl1_ois(((val as u8) & 0x02) >> 1);
        ctrl1_ois.write(self)
    }

    /// Configure DEN mode on the OIS chain.
    ///
    /// # Arguments
    ///
    /// * `val`: Get the values of lvl2_ois in reg INT_OIS.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    pub fn aux_den_mode_get(&mut self) -> Result<Lvl2Ois, Error<B::Error>> {
        let int_ois = IntOis::read(self)?;
        let ctrl1_ois = Ctrl1Ois::read(self)?;

        let value = (ctrl1_ois.lvl1_ois() << 1) + int_ois.lvl2_ois();
        Ok(Lvl2Ois::try_from(value).unwrap_or_default())
    }

    /// Enables/Disable OIS chain DRDY on INT2 pin. This setting has
    /// priority over all other INT2 settings.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of int2_drdy_ois in reg INT_OIS.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn aux_drdy_on_int2_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut int_ois = IntOis::read(self)?;
        int_ois.set_int2_drdy_ois(val);
        int_ois.write(self)
    }

    /// Enables/Disable OIS chain DRDY on INT2 pin. This setting has
    /// priority over all other INT2 settings.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of int2_drdy_ois in reg INT_OIS.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn aux_drdy_on_int2_get(&mut self) -> Result<u8, Error<B::Error>> {
        Ok(IntOis::read(self)?.int2_drdy_ois())
    }

    /// Enables OIS chain data processing for gyro in Mode 3 and Mode 4
    /// (mode4_en = 1) and accelerometer data in Mode 4 (mode4_en = 1).
    /// When the OIS chain is enabled, the OIS outputs are available
    /// through the SPI2 in registers OUTX_L_G (22h) through OUTZ_H_G(27h)
    /// and STATUS_REG (1Eh) / STATUS_SPIAux, and LPF1 is dedicated to
    /// this chain.
    pub fn aux_mode_set(&mut self, val: OisEnSpi2) -> Result<(), Error<B::Error>> {
        let mut ctrl1_ois = Ctrl1Ois::read(self)?;

        ctrl1_ois.set_ois_en_spi2((val as u8) & 0x01);
        ctrl1_ois.set_mode4_en(((val as u8) & 0x02) >> 1);

        ctrl1_ois.write(self)
    }

    /// Enables OIS chain data processing for gyro in Mode 3 and Mode 4
    /// (mode4_en = 1) and accelerometer data in Mode 4 (mode4_en = 1).
    /// When the OIS chain is enabled, the OIS outputs are available
    /// through the SPI2 in registers OUTX_L_G (22h) through OUTZ_H_G(27h)
    /// and STATUS_REG (1Eh) / STATUS_SPIAux, and LPF1 is dedicated to
    /// this chain.
    pub fn aux_mode_get(&mut self) -> Result<OisEnSpi2, Error<B::Error>> {
        let ctrl1_ois = Ctrl1Ois::read(self)?;
        let val = (ctrl1_ois.mode4_en() << 1) + ctrl1_ois.ois_en_spi2();
        Ok(OisEnSpi2::try_from(val).unwrap_or_default())
    }

    /// Selects gyroscope OIS chain full-scale.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of fs_g_ois in reg CTRL1_OIS.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn aux_gy_full_scale_set(&mut self, val: FsGOis) -> Result<(), Error<B::Error>> {
        let mut ctrl1_ois = Ctrl1Ois::read(self)?;

        ctrl1_ois.set_fs_g_ois((val as u8) & 0x03);
        ctrl1_ois.set_fs_125_ois(((val as u8) & 0x04) >> 2);

        ctrl1_ois.write(self)
    }

    /// Selects gyroscope OIS chain full-scale.
    ///
    /// # Arguments
    ///
    /// * `val`: Get the values of fs_g_ois in reg CTRL1_OIS.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`: No Error.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn aux_gy_full_scale_get(&mut self) -> Result<FsGOis, Error<B::Error>> {
        let ctrl1_ois = Ctrl1Ois::read(self)?;
        let value = (ctrl1_ois.fs_125_ois() << 2) + ctrl1_ois.fs_g_ois();
        Ok(FsGOis::try_from(value).unwrap_or_default())
    }

    /// SPI2 3- or 4-wire interface.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of sim_ois in reg CTRL1_OIS.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn aux_spi_mode_set(&mut self, val: SimOis) -> Result<(), Error<B::Error>> {
        let mut ctrl1_ois = Ctrl1Ois::read(self)?;
        ctrl1_ois.set_sim_ois(val as u8);
        ctrl1_ois.write(self)
    }

    /// SPI2 3- or 4-wire interface.
    ///
    /// # Arguments
    ///
    /// * `val`: Get the values of sim_ois in reg CTRL1_OIS.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn aux_spi_mode_get(&mut self) -> Result<SimOis, Error<B::Error>> {
        let ctrl1_ois = Ctrl1Ois::read(self)?;
        Ok(SimOis::try_from(ctrl1_ois.sim_ois()).unwrap_or_default())
    }

    /// Selects gyroscope digital LPF1 filter bandwidth.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of ftype_ois in reg CTRL2_OIS.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn aux_gy_bandwidth_set(&mut self, val: FtypeOis) -> Result<(), Error<B::Error>> {
        let mut ctrl2_ois = Ctrl2Ois::read(self)?;
        ctrl2_ois.set_ftype_ois(val as u8);
        ctrl2_ois.write(self)
    }

    /// Selects gyroscope digital LPF1 filter bandwidth.
    ///
    /// # Returns
    //lp1_filter
    /// * `Result`
    ///     * `FtypeOis`: Get the values of ftype_ois in reg CTRL2_OIS.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn aux_gy_bandwidth_get(&mut self) -> Result<FtypeOis, Error<B::Error>> {
        let ctrl2_ois = Ctrl2Ois::read(self)?;
        Ok(FtypeOis::try_from(ctrl2_ois.ftype_ois()).unwrap_or_default())
    }

    /// Selects gyroscope OIS chain digital high-pass filter cutoff.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of hpm_ois in reg CTRL2_OIS.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn aux_gy_hp_filter_set(&mut self, val: HpmOis) -> Result<(), Error<B::Error>> {
        let mut ctrl2_ois = Ctrl2Ois::read(self)?;

        ctrl2_ois.set_hpm_ois((val as u8) & 0x03);
        ctrl2_ois.set_hp_en_ois(((val as u8) & 0x10) >> 4);

        ctrl2_ois.write(self)
    }

    /// Selects gyroscope OIS chain digital high-pass filter cutoff.
    ///
    /// # Arguments
    ///
    /// * `val`: Get the values of hpm_ois in reg CTRL2_OIS.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`: No Error.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn aux_gy_hp_filter_get(&mut self) -> Result<HpmOis, Error<B::Error>> {
        let ctrl2_ois = Ctrl2Ois::read(self)?;
        let val = (ctrl2_ois.hp_en_ois() << 4) + ctrl2_ois.hpm_ois();
        Ok(HpmOis::try_from(val).unwrap_or_default())
    }

    /// Enable / Disables OIS chain clamp. Enable: All OIS chain
    /// outputs = 8000h during self-test; Disable: OIS chain self-test
    /// outputs dependent from the aux gyro full scale selected.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of st_ois_clampdis in reg CTRL3_OIS.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`.
    pub fn aux_gy_clamp_set(&mut self, val: StOisClampdis) -> Result<(), Error<B::Error>> {
        let mut ctrl3_ois = Ctrl3Ois::read(self)?;
        ctrl3_ois.set_st_ois_clampdis(val as u8);
        ctrl3_ois.write(self)
    }

    /// Enable / Disables OIS chain clamp. Enable: All OIS chain
    /// outputs = 8000h during self-test; Disable: OIS chain self-test
    /// outputs dependent from the aux gyro full scale selected.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `StOisClampdis`: Get the values of st_ois_clampdis in reg CTRL3_OIS.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn aux_gy_clamp_get(&mut self) -> Result<StOisClampdis, Error<B::Error>> {
        let ctrl3_ois = Ctrl3Ois::read(self)?;
        Ok(StOisClampdis::try_from(ctrl3_ois.st_ois_clampdis()).unwrap_or_default())
    }

    /// Selects gyroscope OIS chain self-test.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of st_ois in reg CTRL3_OIS.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn aux_gy_self_test_set(&mut self, val: StOis) -> Result<(), Error<B::Error>> {
        let mut ctrl3_ois = Ctrl3Ois::read(self)?;
        ctrl3_ois.set_st_ois(val as u8);
        ctrl3_ois.write(self)
    }

    /// Selects gyroscope OIS chain self-test.
    ///
    /// # Arguments
    ///
    /// * `val`: Get the values of st_ois in reg CTRL3_OIS.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`: Interface status (MANDATORY: return Ok(()) -> no Error).
    pub fn aux_gy_self_test_get(&mut self) -> Result<StOis, Error<B::Error>> {
        let ctrl3_ois = Ctrl3Ois::read(self)?;
        Ok(StOis::try_from(ctrl3_ois.st_ois()).unwrap_or_default())
    }

    /// Selects accelerometer OIS channel bandwidth.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of filter_xl_conf_ois in reg CTRL3_OIS.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn aux_xl_bandwidth_set(&mut self, val: FilterXlConfOis) -> Result<(), Error<B::Error>> {
        let mut ctrl3_ois = Ctrl3Ois::read(self)?;
        ctrl3_ois.set_filter_xl_conf_ois(val as u8);
        ctrl3_ois.write(self)
    }

    /// Selects accelerometer OIS channel bandwidth.
    ///
    /// # Arguments
    ///
    /// * `val`: Get the values of filter_xl_conf_ois in reg CTRL3_OIS.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn aux_xl_bandwidth_get(&mut self) -> Result<FilterXlConfOis, Error<B::Error>> {
        let ctrl3_ois = Ctrl3Ois::read(self)?;
        Ok(FilterXlConfOis::try_from(ctrl3_ois.filter_xl_conf_ois()).unwrap_or_default())
    }

    /// Selects accelerometer OIS channel full-scale.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of fs_xl_ois in reg CTRL3_OIS.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn aux_xl_full_scale_set(&mut self, val: FsXlOis) -> Result<(), Error<B::Error>> {
        let mut ctrl3_ois = Ctrl3Ois::read(self)?;
        ctrl3_ois.set_fs_xl_ois(val as u8);
        ctrl3_ois.write(self)
    }

    /// Selects accelerometer OIS channel full-scale.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `FsXlOis`: Get the values of fs_xl_ois in reg CTRL3_OIS.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn aux_xl_full_scale_get(&mut self) -> Result<FsXlOis, Error<B::Error>> {
        let ctrl3_ois = Ctrl3Ois::read(self)?;
        Ok(FsXlOis::try_from(ctrl3_ois.fs_xl_ois()).unwrap_or_default())
    }

    /// Connect/Disconnect SDO/SA0 internal pull-up.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of sdo_pu_en in reg PIN_CTRL.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn sdo_sa0_mode_set(&mut self, val: SdoPuEn) -> Result<(), Error<B::Error>> {
        let mut pin_ctrl = PinCtrl::read(self)?;
        pin_ctrl.set_sdo_pu_en(val as u8);
        pin_ctrl.write(self)
    }

    /// Connect/Disconnect SDO/SA0 internal pull-up.
    ///
    /// # Arguments
    ///
    /// * `val`: Get the values of sdo_pu_en in reg PIN_CTRL.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `SdoPuEn`: The value of sdo_pu_en.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn sdo_sa0_mode_get(&mut self) -> Result<SdoPuEn, Error<B::Error>> {
        let pin_ctrl = PinCtrl::read(self)?;
        Ok(SdoPuEn::try_from(pin_ctrl.sdo_pu_en()).unwrap_or_default())
    }

    /// SPI Serial Interface Mode selection.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of sim in reg CTRL3_C.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`: No Error.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn spi_mode_set(&mut self, val: Sim) -> Result<(), Error<B::Error>> {
        let mut ctrl3_c = Ctrl3C::read(self)?;
        ctrl3_c.set_sim(val as u8);
        ctrl3_c.write(self)
    }

    /// SPI Serial Interface Mode selection.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `Sim`: Get the values of sim in reg CTRL3_C.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn spi_mode_get(&mut self) -> Result<Sim, Error<B::Error>> {
        let ctrl3_c = Ctrl3C::read(self)?;
        Ok(Sim::try_from(ctrl3_c.sim()).unwrap_or_default())
    }

    /// Disable / Enable I2C interface.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of i2c_disable in reg CTRL4_C.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn i2c_interface_set(&mut self, val: I2cDisable) -> Result<(), Error<B::Error>> {
        let mut ctrl4_c = Ctrl4C::read(self)?;
        ctrl4_c.set_i2c_disable(val as u8);
        ctrl4_c.write(self)
    }

    /// Disable / Enable I2C interface.
    ///
    /// # Arguments
    ///
    /// * `val`: Get the values of i2c reg CTRL4_C.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`: Returns no error if successful.
    pub fn i2c_interface_get(&mut self) -> Result<I2cDisable, Error<B::Error>> {
        let ctrl4_c = Ctrl4C::read(self)?;
        Ok(I2cDisable::try_from(ctrl4_c.i2c_disable()).unwrap_or_default())
    }

    /// Select the signal that need to route on int1 pad.
    ///
    /// # Arguments
    ///
    /// * `val`: Structure of registers: INT1_CTRL, MD1_CFG,
    ///   EMB_FUNC_INT1, FSM_INT1_A, FSM_INT1_B.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn pin_int1_route_set(&mut self, val: &mut PinInt1Route) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_emb(self, |state| {
            val.mlc_int1.write(state)?;
            val.emb_func_int1.write(state)?;
            val.fsm_int1_a.write(state)?;
            val.fsm_int1_b.write(state)
        })?;

        if (val.emb_func_int1.into_bits()
            | val.fsm_int1_a.into_bits()
            | val.fsm_int1_b.into_bits()
            | val.mlc_int1.into_bits())
            != PROPERTY_DISABLE
        {
            val.md1_cfg.set_int1_emb_func(PROPERTY_ENABLE);
        } else {
            val.md1_cfg.set_int1_emb_func(PROPERTY_DISABLE);
        }

        val.int1_ctrl.write(self)?;
        val.md1_cfg.write(self)?;

        let mut tap_cfg2 = TapCfg2::read(self)?;

        if (val.int1_ctrl.into_bits() | val.md1_cfg.into_bits()) != PROPERTY_DISABLE {
            tap_cfg2.set_interrupts_enable(PROPERTY_ENABLE);
        } else {
            tap_cfg2.set_interrupts_enable(PROPERTY_DISABLE);
        }

        tap_cfg2.write(self)
    }

    /// Select the signal that need to route on int1 pad.
    ///
    /// # Arguments
    ///
    /// * `val`: Structure of registers: INT1_CTRL, MD1_CFG,
    ///   EMB_FUNC_INT1, FSM_INT1_A, FSM_INT1_B.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    pub fn pin_int1_route_get(&mut self) -> Result<PinInt1Route, Error<B::Error>> {
        let mut route = PinInt1Route::default();

        MemBank::operate_over_emb(self, |state| {
            route.mlc_int1 = MlcInt1::read(state)?;
            route.emb_func_int1 = EmbFuncInt1::read(state)?;
            route.fsm_int1_a = FsmInt1A::read(state)?;
            route.fsm_int1_b = FsmInt1B::read(state)?;
            Ok(())
        })?;

        route.int1_ctrl = Int1Ctrl::read(self)?;
        route.md1_cfg = Md1Cfg::read(self)?;

        Ok(route)
    }

    /// Select the signal that need to route on int2 pad.
    ///
    /// # Arguments
    ///
    /// * `val`: Structure of registers INT2_CTRL, MD2_CFG, EMB_FUNC_INT2, FSM_INT2_A, FSM_INT2_B.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`.
    pub fn pin_int2_route_set(&mut self, val: &mut PinInt2Route) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_emb(self, |state| {
            val.mlc_int2.write(state)?;
            val.emb_func_int2.write(state)?;
            val.fsm_int2_a.write(state)?;
            val.fsm_int2_b.write(state)
        })?;

        if (val.emb_func_int2.into_bits()
            | val.fsm_int2_a.into_bits()
            | val.fsm_int2_b.into_bits()
            | val.mlc_int2.into_bits())
            != PROPERTY_DISABLE
        {
            val.md2_cfg.set_int2_emb_func(PROPERTY_ENABLE);
        } else {
            val.md2_cfg.set_int2_emb_func(PROPERTY_DISABLE);
        }

        val.int2_ctrl.write(self)?;
        val.md2_cfg.write(self)?;

        let mut tap_cfg2 = TapCfg2::read(self)?;

        if (val.int2_ctrl.into_bits() | val.md2_cfg.into_bits()) != PROPERTY_DISABLE {
            tap_cfg2.set_interrupts_enable(PROPERTY_ENABLE);
        } else {
            tap_cfg2.set_interrupts_enable(PROPERTY_DISABLE);
        }

        tap_cfg2.write(self)
    }

    /// Select the signal that need to route on int2 pad.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `PinInt2Route`: Structure of registers INT2_CTRL,  MD2_CFG,
    ///       EMB_FUNC_INT2, FSM_INT2_A, FSM_INT2_B.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn pin_int2_route_get(&mut self) -> Result<PinInt2Route, Error<B::Error>> {
        let mut route = PinInt2Route::default();

        MemBank::operate_over_emb(self, |state| {
            route.mlc_int2 = MlcInt2::read(state)?;
            route.emb_func_int2 = EmbFuncInt2::read(state)?;
            route.fsm_int2_a = FsmInt2A::read(state)?;
            route.fsm_int2_b = FsmInt2B::read(state)?;
            Ok(())
        })?;

        route.int2_ctrl = Int2Ctrl::read(self)?;
        route.md2_cfg = Md2Cfg::read(self)?;

        Ok(route)
    }

    /// Push-pull/open drain selection on interrupt pads.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of pp_od in reg CTRL3_C.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn pin_mode_set(&mut self, val: PpOd) -> Result<(), Error<B::Error>> {
        let mut ctrl3_c = Ctrl3C::read(self)?;
        ctrl3_c.set_pp_od(val as u8);
        ctrl3_c.write(self)
    }

    /// Push-pull/open drain selection on interrupt pads.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `PpOd`: Get the values of pp_od in reg CTRL3_C.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn pin_mode_get(&mut self) -> Result<PpOd, Error<B::Error>> {
        let ctrl3_c = Ctrl3C::read(self)?;
        Ok(PpOd::try_from(ctrl3_c.pp_od()).unwrap_or_default())
    }

    /// Interrupt active-high/low.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of h_lactive in reg CTRL3_C.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn pin_polarity_set(&mut self, val: HLactive) -> Result<(), Error<B::Error>> {
        let mut ctrl3_c = Ctrl3C::read(self)?;
        ctrl3_c.set_h_lactive(val as u8);
        ctrl3_c.write(self)
    }

    /// Interrupt active-high/low.
    ///
    /// # Arguments
    ///
    /// * `val`: Get the values of h_lactive in reg CTRL3_C.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn pin_polarity_get(&mut self) -> Result<HLactive, Error<B::Error>> {
        let ctrl3_c = Ctrl3C::read(self)?;
        Ok(HLactive::try_from(ctrl3_c.h_lactive()).unwrap_or_default())
    }

    /// All interrupt signals become available on INT1 pin.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of int2_on_int1 in reg CTRL4_C.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`.
    pub fn all_on_int1_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl4_c = Ctrl4C::read(self)?;
        ctrl4_c.set_int2_on_int1(val);
        ctrl4_c.write(self)
    }

    /// All interrupt signals become available on INT1 pin.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of int2_on_int1 in reg CTRL4_C.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn all_on_int1_get(&mut self) -> Result<u8, Error<B::Error>> {
        let ctrl4_c = Ctrl4C::read(self)?;
        Ok(ctrl4_c.int2_on_int1())
    }

    /// All interrupt signals notification mode.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of lir in reg TAP_CFG0.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    pub fn int_notification_set(&mut self, val: Lir) -> Result<(), Error<B::Error>> {
        let mut tap_cfg0 = TapCfg0::read(self)?;
        tap_cfg0.set_lir((val as u8) & 0x01);
        tap_cfg0.set_int_clr_on_read((val as u8) & 0x01);
        tap_cfg0.write(self)?;

        MemBank::operate_over_emb(self, |state| {
            let mut page_rw = PageRw::read(state)?;
            page_rw.set_emb_func_lir(((val as u8) & 0x02) >> 1);
            page_rw.write(state)
        })
    }

    /// All interrupt signals notification mode.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `Lir`: Get the values of lir in reg TAP_CFG0.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn int_notification_get(&mut self) -> Result<Lir, Error<B::Error>> {
        let tap_cfg0 = TapCfg0::read(self)?;

        MemBank::operate_over_emb(self, |state| {
            let page_rw = PageRw::read(state)?;
            let val = (page_rw.emb_func_lir() << 1) + tap_cfg0.lir();
            Ok(Lir::try_from(val).unwrap_or_default())
        })
    }

    /// Weight of 1 LSB of wakeup threshold.
    /// 0: 1 LSB =FS_XL  /  64
    /// 1: 1 LSB = FS_XL / 256
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of wake_ths_w in reg WAKE_UP_DUR.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn wkup_ths_weight_set(&mut self, val: WakeThsW) -> Result<(), Error<B::Error>> {
        let mut wake_up_dur = WakeUpDur::read(self)?;
        wake_up_dur.set_wake_ths_w(val as u8);
        wake_up_dur.write(self)
    }

    /// Weight of 1 LSB of wakeup threshold.
    ///
    /// 0: 1 LSB = FS_XL  /  64
    /// 1: 1 LSB = FS_XL / 256
    ///
    /// # Arguments
    ///
    /// * `val`: Get the values of wake_ths_w in reg WAKE_UP_DUR.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn wkup_ths_weight_get(&mut self) -> Result<WakeThsW, Error<B::Error>> {
        let wake_up_dur = WakeUpDur::read(self)?;
        Ok(WakeThsW::try_from(wake_up_dur.wake_ths_w()).unwrap_or_default())
    }

    /// Threshold for wakeup: 1 LSB weight depends on WAKE_THS_W in
    /// WAKE_UP_DUR.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of wk_ths in reg WAKE_UP_THS.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn wkup_threshold_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut wake_up_ths = WakeUpThs::read(self)?;
        wake_up_ths.set_wk_ths(val);
        wake_up_ths.write(self)
    }

    /// Threshold for wakeup: 1 LSB weight depends on WAKE_THS_W in WAKE_UP_DUR.
    pub fn wkup_threshold_get(&mut self) -> Result<u8, Error<B::Error>> {
        let wake_up_ths = WakeUpThs::read(self)?;
        Ok(wake_up_ths.wk_ths())
    }

    /// Wake up duration event (1LSb = 1 / ODR).
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of usr_off_on_wu in reg WAKE_UP_THS.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn xl_usr_offset_on_wkup_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut wake_up_ths = WakeUpThs::read(self)?;
        wake_up_ths.set_usr_off_on_wu(val);
        wake_up_ths.write(self)
    }

    /// Wake up duration event (1LSb = 1 / ODR). Get the values of usr_off_on_wu in reg WAKE_UP_THS.
    pub fn xl_usr_offset_on_wkup_get(&mut self) -> Result<u8, Error<B::Error>> {
        Ok(WakeUpThs::read(self)?.usr_off_on_wu())
    }

    /// Wake up duration event(1LSb = 1 / ODR).
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of wake_dur in reg WAKE_UP_DUR.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn wkup_dur_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut wake_up_dur = WakeUpDur::read(self)?;
        wake_up_dur.set_wake_dur(val);
        wake_up_dur.write(self)
    }

    /// Wake up duration event (1LSb = 1 / ODR).
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of wake_dur in reg WAKE_UP_DUR.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn wkup_dur_get(&mut self) -> Result<u8, Error<B::Error>> {
        Ok(WakeUpDur::read(self)?.wake_dur())
    }

    /// Enables gyroscope Sleep mode.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of sleep_g in reg CTRL4_C.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn gy_sleep_mode_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl4_c = Ctrl4C::read(self)?;
        ctrl4_c.set_sleep_g(val);
        ctrl4_c.write(self)
    }

    /// Enables gyroscope Sleep mode.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of sleep_g in reg CTRL4_C.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn gy_sleep_mode_get(&mut self) -> Result<u8, Error<B::Error>> {
        let ctrl4_c = Ctrl4C::read(self)?;
        Ok(ctrl4_c.sleep_g())
    }

    /// Drives the sleep status instead of sleep change on INT pins
    /// (only if INT1_SLEEP_CHANGE or INT2_SLEEP_CHANGE bits
    /// are enabled).
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of sleep_status_on_int in reg TAP_CFG0.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn act_pin_notification_set(
        &mut self,
        val: SleepStatusOnInt,
    ) -> Result<(), Error<B::Error>> {
        let mut tap_cfg0 = TapCfg0::read(self)?;
        tap_cfg0.set_sleep_status_on_int(val as u8);
        tap_cfg0.write(self)
    }

    /// Drives the sleep status instead of sleep change on INT pins
    /// (only if INT1_SLEEP_CHANGE or INT2_SLEEP_CHANGE bits
    /// are enabled).
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `SleepStatusOnInt`: Get the values of sleep_status_on_int in reg TAP_CFG0.
    pub fn act_pin_notification_get(&mut self) -> Result<SleepStatusOnInt, Error<B::Error>> {
        let tap_cfg0 = TapCfg0::read(self)?;
        Ok(SleepStatusOnInt::try_from(tap_cfg0.sleep_status_on_int()).unwrap_or_default())
    }

    /// Enable inactivity function.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of inact_en in reg TAP_CFG2.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn act_mode_set(&mut self, val: InactEn) -> Result<(), Error<B::Error>> {
        let mut tap_cfg2 = TapCfg2::read(self)?;
        tap_cfg2.set_inact_en(val as u8);
        tap_cfg2.write(self)
    }

    /// Enable inactivity function.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `InactEn`: Get the values of inact_en in reg TAP_CFG2.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn act_mode_get(&mut self) -> Result<InactEn, Error<B::Error>> {
        let tap_cfg2 = TapCfg2::read(self)?;
        Ok(InactEn::try_from(tap_cfg2.inact_en()).unwrap_or_default())
    }

    /// Duration to go in sleep mode (1 LSb = 512 / ODR).
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of sleep_dur in reg WAKE_UP_DUR.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`.
    pub fn act_sleep_dur_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut wake_up_dur = WakeUpDur::read(self)?;
        wake_up_dur.set_sleep_dur(val);
        wake_up_dur.write(self)
    }

    /// Duration to go in sleep mode.(1 LSb = 512 / ODR).
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of sleep_dur in reg WAKE_UP_DUR.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn act_sleep_dur_get(&mut self) -> Result<u8, Error<B::Error>> {
        Ok(WakeUpDur::read(self)?.sleep_dur())
    }

    /// Enable Z direction in tap recognition.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of tap_z_en in reg TAP_CFG0.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`.
    pub fn tap_detection_on_z_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut tap_cfg0 = TapCfg0::read(self)?;
        tap_cfg0.set_tap_z_en(val);
        tap_cfg0.write(self)
    }

    /// Enable Z direction in tap recognition.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of tap_z_en in reg TAP_CFG0.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn tap_detection_on_z_get(&mut self) -> Result<u8, Error<B::Error>> {
        let tap_cfg0 = TapCfg0::read(self)?;
        Ok(tap_cfg0.tap_z_en())
    }

    /// Enable Y direction in tap recognition.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of tap_y_en in reg TAP_CFG0.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn tap_detection_on_y_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut tap_cfg0 = TapCfg0::read(self)?;
        tap_cfg0.set_tap_y_en(val);
        tap_cfg0.write(self)
    }

    /// Enable Y direction in tap recognition.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of tap_y_en in reg TAP_CFG0.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn tap_detection_on_y_get(&mut self) -> Result<u8, Error<B::Error>> {
        let tap_cfg0 = TapCfg0::read(self)?;
        Ok(tap_cfg0.tap_y_en())
    }

    /// Enable X direction in tap recognition.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of tap_x_en in reg TAP_CFG0.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn tap_detection_on_x_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut tap_cfg0 = TapCfg0::read(self)?;
        tap_cfg0.set_tap_x_en(val);
        tap_cfg0.write(self)
    }

    /// Enable X direction in tap recognition.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of tap_x_en in reg TAP_CFG0.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn tap_detection_on_x_get(&mut self) -> Result<u8, Error<B::Error>> {
        let tap_cfg0 = TapCfg0::read(self)?;
        Ok(tap_cfg0.tap_x_en())
    }

    /// X-axis tap recognition threshold.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of tap_ths_x in reg TAP_CFG1.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn tap_threshold_x_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut tap_cfg1 = TapCfg1::read(self)?;
        tap_cfg1.set_tap_ths_x(val);
        tap_cfg1.write(self)
    }

    /// X-axis tap recognition threshold.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of tap_ths_x in reg TAP_CFG1.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn tap_threshold_x_get(&mut self) -> Result<u8, Error<B::Error>> {
        let tap_cfg1 = TapCfg1::read(self)?;
        Ok(tap_cfg1.tap_ths_x())
    }

    /// Selection of axis priority for TAP detection.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of tap_priority in reg TAP_CFG1.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn tap_axis_priority_set(&mut self, val: TapPriority) -> Result<(), Error<B::Error>> {
        let mut tap_cfg1 = TapCfg1::read(self)?;
        tap_cfg1.set_tap_priority(val as u8);
        tap_cfg1.write(self)
    }

    /// Selection of axis priority for TAP detection.
    ///
    /// # Arguments
    ///
    /// * `val`: Get the values of tap_priority in reg TAP_CFG1.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`.
    pub fn tap_axis_priority_get(&mut self) -> Result<TapPriority, Error<B::Error>> {
        let tap_cfg1 = TapCfg1::read(self)?;
        Ok(TapPriority::try_from(tap_cfg1.tap_priority()).unwrap_or_default())
    }

    /// Y-axis tap recognition threshold.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of tap_ths_y in reg TAP_CFG2.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn tap_threshold_y_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut tap_cfg2 = TapCfg2::read(self)?;
        tap_cfg2.set_tap_ths_y(val);
        tap_cfg2.write(self)
    }

    /// Y-axis tap recognition threshold.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of tap_ths_y in reg TAP_CFG2.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn tap_threshold_y_get(&mut self) -> Result<u8, Error<B::Error>> {
        let tap_cfg2 = TapCfg2::read(self)?;
        Ok(tap_cfg2.tap_ths_y())
    }

    /// Z-axis recognition threshold.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of tap_ths_z in reg TAP_THS_6D.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn tap_threshold_z_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut tap_ths_6d = TapThs6d::read(self)?;
        tap_ths_6d.set_tap_ths_z(val);
        tap_ths_6d.write(self)
    }

    /// Z-axis recognition threshold.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of tap_ths_z in reg TAP_THS_6D.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn tap_threshold_z_get(&mut self) -> Result<u8, Error<B::Error>> {
        let tap_ths_6d = TapThs6d::read(self)?;
        Ok(tap_ths_6d.tap_ths_z())
    }

    /// Maximum duration is the maximum time of an overthreshold signal
    /// detection to be recognized as a tap event. The default value of
    /// these bits is 00b which corresponds to 4*ODR_XL time.
    /// If the SHOCK\[1:0\] bits are set to a different value, 1LSB
    /// corresponds to 8*ODR_XL time.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of shock in reg INT_DUR2.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn tap_shock_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut int_dur2 = IntDur2::read(self)?;
        int_dur2.set_shock(val);
        int_dur2.write(self)
    }

    /// Maximum duration is the maximum time of an overthreshold signal
    /// detection to be recognized as a tap event. The default value of
    /// these bits is 00b which corresponds to 4*ODR_XL time.
    /// If the SHOCK\[1:0\] bits are set to a different value, 1LSB
    /// corresponds to 8*ODR_XL time.
    pub fn tap_shock_get(&mut self) -> Result<u8, Error<B::Error>> {
        let int_dur2 = IntDur2::read(self)?;
        Ok(int_dur2.shock())
    }

    /// Quiet time is the time after the first detected tap in which
    /// there must not be any overthreshold event.
    /// The default value of these bits is 00b which corresponds to
    /// 2*ODR_XL time. If the QUIET\[1:0\] bits are set to a different
    /// value, 1LSB corresponds to 4*ODR_XL time.
    pub fn tap_quiet_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut int_dur2 = IntDur2::read(self)?;
        int_dur2.set_quiet(val);
        int_dur2.write(self)
    }

    /// Quiet time is the time after the first detected tap in which
    /// there must not be any overthreshold event.
    /// The default value of these bits is 00b which corresponds to
    /// 2*ODR_XL time. If the QUIET\[1:0\] bits are set to a different
    /// value, 1LSB corresponds to 4*ODR_XL time.
    pub fn tap_quiet_get(&mut self) -> Result<u8, Error<B::Error>> {
        let int_dur2 = IntDur2::read(self)?;
        Ok(int_dur2.quiet())
    }

    /// When double tap recognition is enabled, this register expresses
    /// the maximum time between two consecutive detected taps to
    /// determine a double tap event.
    /// The default value of these bits is 0000b which corresponds to
    /// 16*ODR_XL time.
    /// If the DUR\[3:0\] bits are set to a different value, 1LSB
    /// corresponds to 32*ODR_XL time.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of dur in reg INT_DUR2.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn tap_dur_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut int_dur2 = IntDur2::read(self)?;
        int_dur2.set_dur(val);
        int_dur2.write(self)
    }

    /// When double tap recognition is enabled, this register expresses the
    /// maximum time between two consecutive detected taps to determine
    /// a double tap event. The default value of these bits is 0000b which
    /// corresponds to 16*ODR_XL time. If the DUR\[3:0\] bits are set to
    /// a different value, 1LSB corresponds to 32*ODR_XL time.
    pub fn tap_dur_get(&mut self) -> Result<u8, Error<B::Error>> {
        let int_dur2 = IntDur2::read(self)?;
        Ok(int_dur2.dur())
    }

    /// Single/double-tap event enable.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of single_double_tap in reg WAKE_UP_THS.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn tap_mode_set(&mut self, val: SingleDoubleTap) -> Result<(), Error<B::Error>> {
        let mut wake_up_ths = WakeUpThs::read(self)?;
        wake_up_ths.set_single_double_tap(val as u8);
        wake_up_ths.write(self)
    }

    /// Single/double-tap event enable.
    ///
    /// # Arguments
    ///
    /// * `val`: Get the values of single_double_tap in reg WAKE_UP_THS.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`.
    pub fn tap_mode_get(&mut self) -> Result<SingleDoubleTap, Error<B::Error>> {
        let wake_up_ths = WakeUpThs::read(self)?;
        Ok(SingleDoubleTap::try_from(wake_up_ths.single_double_tap()).unwrap_or_default())
    }

    /// Threshold for 4D/6D function.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of sixd_ths in reg TAP_THS_6D.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn six_d_threshold_set(&mut self, val: SixdThs) -> Result<(), Error<B::Error>> {
        let mut tap_ths_6d = TapThs6d::read(self)?;
        tap_ths_6d.set_sixd_ths(val as u8);
        tap_ths_6d.write(self)
    }

    /// Threshold for 4D/6D function.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `SixdThs`: Get the values of sixd_ths in reg TAP_THS_6D.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn six_d_threshold_get(&mut self) -> Result<SixdThs, Error<B::Error>> {
        let tap_ths_6d = TapThs6d::read(self)?;
        Ok(SixdThs::try_from(tap_ths_6d.sixd_ths()).unwrap_or_default())
    }

    /// 4D orientation detection enable.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of d4d_en in reg TAP_THS_6D.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`: No Error.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn four_d_mode_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut tap_ths_6d = TapThs6d::read(self)?;
        tap_ths_6d.set_d4d_en(val);
        tap_ths_6d.write(self)
    }

    /// 4D orientation detection enable.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of d4d_en in reg TAP_THS_6D.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn four_d_mode_get(&mut self) -> Result<u8, Error<B::Error>> {
        let tap_ths_6d = TapThs6d::read(self)?;
        Ok(tap_ths_6d.d4d_en())
    }

    /// Free fall threshold setting.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of ff_ths in reg FREE_FALL.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn ff_threshold_set(&mut self, val: FfThs) -> Result<(), Error<B::Error>> {
        let mut free_fall = FreeFall::read(self)?;
        free_fall.set_ff_ths(val as u8);
        free_fall.write(self)
    }

    /// Free fall threshold setting.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `FfThs`: Get the values of ff_ths in reg FREE_FALL.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn ff_threshold_get(&mut self) -> Result<FfThs, Error<B::Error>> {
        let free_fall = FreeFall::read(self)?;
        Ok(FfThs::try_from(free_fall.ff_ths()).unwrap_or_default())
    }

    /// Free-fall duration event(1LSb = 1 / ODR).
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of ff_dur in reg FREE_FALL.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn ff_dur_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut wake_up_dur = WakeUpDur::read(self)?;
        wake_up_dur.set_ff_dur((val & 0x20) >> 5);
        wake_up_dur.write(self)?;

        let mut free_fall = FreeFall::read(self)?;
        free_fall.set_ff_dur(val & 0x1F);
        free_fall.write(self)
    }

    /// Free-fall duration event(1LSb = 1 / ODR).
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of ff_dur in reg FREE_FALL.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn ff_dur_get(&mut self) -> Result<u8, Error<B::Error>> {
        let wake_up_dur = WakeUpDur::read(self)?;
        let free_fall = FreeFall::read(self)?;

        let val = (wake_up_dur.ff_dur() << 5) + free_fall.ff_dur();
        Ok(val)
    }

    /// FIFO watermark level selection.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of wtm in reg FIFO_CTRL1.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn fifo_watermark_set(&mut self, val: u16) -> Result<(), Error<B::Error>> {
        let mut fifo_ctrl1 = FifoCtrl1::read(self)?;
        let mut fifo_ctrl2 = FifoCtrl2::read(self)?;

        let [val_l, val_h] = val.to_le_bytes();
        fifo_ctrl1.set_wtm(val_l);
        fifo_ctrl2.set_wtm(val_h & 0x01);

        fifo_ctrl1.write(self)?;
        fifo_ctrl2.write(self)
    }

    /// FIFO watermark level selection.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u16`: Change the values of wtm in reg FIFO_CTRL1.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn fifo_watermark_get(&mut self) -> Result<u16, Error<B::Error>> {
        let fifo_ctrl1 = FifoCtrl1::read(self)?;
        let fifo_ctrl2 = FifoCtrl2::read(self)?;
        Ok(u16::from_le_bytes([fifo_ctrl1.wtm(), fifo_ctrl2.wtm()]))
    }

    /// FIFO compression feature initialization request..
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of FIFO_COMPR_INIT in reg EMB_FUNC_INIT_B.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn compression_algo_init_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_emb(self, |state| {
            let mut reg = EmbFuncInitB::read(state)?;
            reg.set_fifo_compr_init(val);
            reg.write(state)
        })
    }

    /// FIFO compression feature initialization request.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: change the values of FIFO_COMPR_INIT in reg EMB_FUNC_INIT_B.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn compression_algo_init_get(&mut self) -> Result<u8, Error<B::Error>> {
        MemBank::operate_over_emb(self, |state| {
            let emb_func_init_b = EmbFuncInitB::read(state)?;
            Ok(emb_func_init_b.fifo_compr_init())
        })
    }

    /// Enable and configure compression algo.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of uncoptr_rate in reg FIFO_CTRL2.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`.
    pub fn compression_algo_set(&mut self, val: UncoptrRate) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_emb(self, |state| {
            let mut emb_func_en_b = EmbFuncEnB::read(state)?;
            emb_func_en_b.set_fifo_compr_en(((val as u8) & 0x04) >> 2);
            emb_func_en_b.write(state)
        })?;

        let mut fifo_ctrl2 = FifoCtrl2::read(self)?;
        fifo_ctrl2.set_fifo_compr_rt_en(((val as u8) & 0x04) >> 2);
        fifo_ctrl2.set_uncoptr_rate((val as u8) & 0x03);
        fifo_ctrl2.write(self)
    }

    /// Enable and configure compression algo.
    ///
    /// # Arguments
    ///
    /// * `val`: Get the values of uncoptr_rate in reg FIFO_CTRL2.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn compression_algo_get(&mut self) -> Result<UncoptrRate, Error<B::Error>> {
        let fifo_ctrl2 = FifoCtrl2::read(self)?;
        let val = (fifo_ctrl2.fifo_compr_rt_en() << 2) + fifo_ctrl2.uncoptr_rate();
        Ok(UncoptrRate::try_from(val).unwrap_or_default())
    }

    /// Enables ODR CHANGE virtual sensor to be batched in FIFO.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of odrchg_en in reg FIFO_CTRL2.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn fifo_virtual_sens_odr_chg_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut fifo_ctrl2 = FifoCtrl2::read(self)?;
        fifo_ctrl2.set_odrchg_en(val);
        fifo_ctrl2.write(self)
    }

    /// Enables ODR CHANGE virtual sensor to be batched in FIFO.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of odrchg_en in reg FIFO_CTRL2.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn fifo_virtual_sens_odr_chg_get(&mut self) -> Result<u8, Error<B::Error>> {
        let fifo_ctrl2 = FifoCtrl2::read(self)?;
        Ok(fifo_ctrl2.odrchg_en())
    }

    /// Enables/Disables compression algorithm runtime.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of fifo_compr_rt_en in reg FIFO_CTRL2.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn compression_algo_real_time_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut fifo_ctrl2 = FifoCtrl2::read(self)?;
        fifo_ctrl2.set_fifo_compr_rt_en(val);
        fifo_ctrl2.write(self)
    }

    /// Enables/Disables compression algorithm runtime.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of fifo_compr_rt_en in reg FIFO_CTRL2.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn compression_algo_real_time_get(&mut self) -> Result<u8, Error<B::Error>> {
        let fifo_ctrl2 = FifoCtrl2::read(self)?;
        Ok(fifo_ctrl2.fifo_compr_rt_en())
    }

    /// Sensing chain FIFO stop values memorization at threshold level.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of stop_on_wtm in reg FIFO_CTRL2.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    pub fn fifo_stop_on_wtm_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut fifo_ctrl2 = FifoCtrl2::read(self)?;
        fifo_ctrl2.set_stop_on_wtm(val);
        fifo_ctrl2.write(self)
    }

    /// Sensing chain FIFO stop values memorization at threshold level.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of stop_on_wtm in reg FIFO_CTRL2.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn fifo_stop_on_wtm_get(&mut self) -> Result<u8, Error<B::Error>> {
        let fifo_ctrl2 = FifoCtrl2::read(self)?;
        Ok(fifo_ctrl2.stop_on_wtm())
    }

    /// Selects Batching Data Rate (writing frequency in FIFO)
    /// for accelerometer data.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of bdr_xl in reg FIFO_CTRL3.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn fifo_xl_batch_set(&mut self, val: BdrXl) -> Result<(), Error<B::Error>> {
        let mut fifo_ctrl3 = FifoCtrl3::read(self)?;
        fifo_ctrl3.set_bdr_xl(val as u8);
        fifo_ctrl3.write(self)
    }

    /// Selects Batching Data Rate (writing frequency in FIFO)
    /// for accelerometer data.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `BdrXl`: Get the values of bdr_xl in reg FIFO_CTRL3.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn fifo_xl_batch_get(&mut self) -> Result<BdrXl, Error<B::Error>> {
        let fifo_ctrl3 = FifoCtrl3::read(self)?;
        Ok(BdrXl::try_from(fifo_ctrl3.bdr_xl()).unwrap_or_default())
    }

    /// Selects Batching Data Rate (writing frequency in FIFO) for gyroscope data.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of bdr_gy in reg FIFO_CTRL3.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn fifo_gy_batch_set(&mut self, val: BdrGy) -> Result<(), Error<B::Error>> {
        let mut fifo_ctrl3 = FifoCtrl3::read(self)?;
        fifo_ctrl3.set_bdr_gy(val as u8);
        fifo_ctrl3.write(self)
    }

    /// Selects Batching Data Rate (writing frequency in FIFO) for gyroscope data.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `BdrGy`: Get the values of bdr_gy in reg FIFO_CTRL3.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn fifo_gy_batch_get(&mut self) -> Result<BdrGy, Error<B::Error>> {
        let fifo_ctrl3 = FifoCtrl3::read(self)?;
        Ok(BdrGy::try_from(fifo_ctrl3.bdr_gy()).unwrap_or_default())
    }

    /// FIFO mode selection.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of fifo_mode in reg FIFO_CTRL4.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn fifo_mode_set(&mut self, val: FifoMode) -> Result<(), Error<B::Error>> {
        let mut fifo_ctrl4 = FifoCtrl4::read(self)?;
        fifo_ctrl4.set_fifo_mode(val as u8);
        fifo_ctrl4.write(self)
    }

    /// FIFO mode selection.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `FifoMode`: Get the values of fifo_mode in reg FIFO_CTRL4.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn fifo_mode_get(&mut self) -> Result<FifoMode, Error<B::Error>> {
        let fifo_ctrl4 = FifoCtrl4::read(self)?;
        Ok(FifoMode::try_from(fifo_ctrl4.fifo_mode()).unwrap_or_default())
    }

    /// Selects Batching Data Rate (writing frequency in FIFO) for temperature data.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of odr_t_batch in reg FIFO_CTRL4.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn fifo_temp_batch_set(&mut self, val: OdrTBatch) -> Result<(), Error<B::Error>> {
        let mut ctrl4 = FifoCtrl4::read(self)?;
        ctrl4.set_odr_t_batch(val as u8);
        ctrl4.write(self)
    }

    /// Selects Batching Data Rate (writing frequency in FIFO) for temperature data.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `OdrTBatch`: Get the values of odr_t_batch in reg FIFO_CTRL4.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn fifo_temp_batch_get(&mut self) -> Result<OdrTBatch, Error<B::Error>> {
        let fifo_ctrl4 = FifoCtrl4::read(self)?;
        Ok(OdrTBatch::try_from(fifo_ctrl4.odr_t_batch()).unwrap_or_default())
    }

    /// Selects decimation for timestamp batching in FIFO.
    /// Writing rate will be the maximum rate between XL and
    /// GYRO BDR divided by decimation decoder.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of odr_ts_batch in reg FIFO_CTRL4.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`: No error.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn fifo_timestamp_decimation_set(
        &mut self,
        val: OdrTsBatch,
    ) -> Result<(), Error<B::Error>> {
        let mut fifo_ctrl4 = FifoCtrl4::read(self)?;
        fifo_ctrl4.set_odr_ts_batch(val as u8);
        fifo_ctrl4.write(self)
    }

    /// Selects decimation for timestamp batching in FIFO.
    /// Writing rate will be the maximum rate between XL and
    /// GYRO BDR divided by decimation decoder.
    pub fn fifo_timestamp_decimation_get(&mut self) -> Result<OdrTsBatch, Error<B::Error>> {
        let fifo_ctrl4 = FifoCtrl4::read(self)?;
        Ok(OdrTsBatch::try_from(fifo_ctrl4.odr_ts_batch()).unwrap_or_default())
    }

    /// Selects the trigger for the internal counter of batching events
    /// between XL and gyro.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of trig_counter_bdr in reg COUNTER_BDR_REG1.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn fifo_cnt_event_batch_set(&mut self, val: TrigCounterBdr) -> Result<(), Error<B::Error>> {
        let mut counter_bdr_reg1 = CounterBdrReg::read(self)?;
        counter_bdr_reg1.set_trig_counter_bdr(val as u8);
        counter_bdr_reg1.write(self)
    }

    /// Selects the trigger for the internal counter of batching events
    /// between XL and gyro.
    ///
    /// # Arguments
    ///
    /// * `val`: Get the values of trig_counter_bdr
    ///   in reg COUNTER_BDR_REG1.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`: Interface status (MANDATORY: return 0 -> no Error).
    pub fn fifo_cnt_event_batch_get(&mut self) -> Result<TrigCounterBdr, Error<B::Error>> {
        let counter_bdr_reg1 = CounterBdrReg::read(self)?;
        Ok(TrigCounterBdr::try_from(counter_bdr_reg1.trig_counter_bdr()).unwrap_or_default())
    }

    /// Resets the internal counter of batching events for a single sensor.
    /// This bit is automatically reset to zero if it was set to '1'.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of rst_counter_bdr in reg COUNTER_BDR_REG1.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn rst_batch_counter_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut counter_bdr_reg1 = CounterBdrReg::read(self)?;
        counter_bdr_reg1.set_rst_counter_bdr(val);
        counter_bdr_reg1.write(self)
    }

    /// Resets the internal counter of batching events for a single sensor.
    /// This bit is automatically reset to zero if it was set to '1'.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of rst_counter_bdr in reg COUNTER_BDR_REG1.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn rst_batch_counter_get(&mut self) -> Result<u8, Error<B::Error>> {
        let counter_bdr_reg1 = CounterBdrReg::read(self)?;
        Ok(counter_bdr_reg1.rst_counter_bdr())
    }

    /// Batch data rate counter.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of cnt_bdr_th in reg COUNTER_BDR_REG2 and COUNTER_BDR_REG1.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn batch_counter_threshold_set(&mut self, val: u16) -> Result<(), Error<B::Error>> {
        let mut counter_bdr_reg = CounterBdrReg::read(self)?;
        counter_bdr_reg.set_cnt_bdr_th(val);
        counter_bdr_reg.write(self)
    }

    /// Batch data rate counter.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u16`: Change the values of cnt_bdr_th in reg COUNTER_BDR_REG2 and COUNTER_BDR_REG1.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn batch_counter_threshold_get(&mut self) -> Result<u16, Error<B::Error>> {
        Ok(CounterBdrReg::read(self)?.cnt_bdr_th())
    }

    /// Number of unread sensor data (TAG + 6 bytes) stored in FIFO.
    ///
    /// # Arguments
    ///
    /// * `val`: Read the value of diff_fifo in reg FIFO_STATUS1 and FIFO_STATUS2.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn fifo_data_level_get(&mut self) -> Result<u16, Error<B::Error>> {
        // read both FIFO_STATUS1 + FIFO_STATUS2 regs
        let fifo_status1 = FifoStatus1::read(self)?;
        let fifo_status2 = FifoStatus2::read(self)?;

        Ok(u16::from_le_bytes([
            fifo_status1.diff_fifo(),
            fifo_status2.diff_fifo(),
        ]))
    }

    /// Smart FIFO status.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `fifo_status2_t`: Read registers FIFO_STATUS2.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn fifo_status_get(&mut self) -> Result<FifoStatus2, Error<B::Error>> {
        FifoStatus2::read(self)
    }

    /// Smart FIFO full status.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Read the values of fifo_full_ia in reg FIFO_STATUS2.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn fifo_full_flag_get(&mut self) -> Result<u8, Error<B::Error>> {
        Ok(self.fifo_status_get()?.fifo_full_ia())
    }

    /// FIFO overrun status.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Read the values of fifo_over_run_latched in reg FIFO_STATUS2.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn fifo_ovr_flag_get(&mut self) -> Result<u8, Error<B::Error>> {
        Ok(self.fifo_status_get()?.fifo_ovr_ia())
    }

    /// FIFO watermark status.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Read the values of fifo_wtm_ia in reg FIFO_STATUS2.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn fifo_wtm_flag_get(&mut self) -> Result<u8, Error<B::Error>> {
        Ok(self.fifo_status_get()?.fifo_wtm_ia())
    }

    /// Identifies the sensor in FIFO_DATA_OUT.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of tag_sensor in reg FIFO_DATA_OUT_TAG.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn fifo_sensor_tag_get(&mut self) -> Result<FifoTag, Error<B::Error>> {
        let fifo_data_out_tag = FifoDataOutTag::read(self)?;
        let val = fifo_data_out_tag.tag_sensor();
        Ok(FifoTag::try_from(val).unwrap_or_default())
    }

    /// Enable FIFO batching of pedometer embedded function values.
    pub fn fifo_pedo_batch_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_emb(self, |state| {
            let mut emb_func_fifo_cfg = EmbFuncFifoCfg::read(state)?;
            emb_func_fifo_cfg.set_pedo_fifo_en(val);
            emb_func_fifo_cfg.write(state)
        })
    }

    /// Enable FIFO batching of pedometer embedded function values.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of pedo_fifo_en in reg ISM330DHCX_EMB_FUNC_FIFO_CFG.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn fifo_pedo_batch_get(&mut self) -> Result<u8, Error<B::Error>> {
        MemBank::operate_over_emb(self, |state| {
            let emb_func_fifo_cfg = EmbFuncFifoCfg::read(state)?;
            Ok(emb_func_fifo_cfg.pedo_fifo_en())
        })
    }

    /// Enable FIFO batching data of first slave.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of batch_ext_sens_0_en in reg SLV0_CONFIG.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn sh_batch_slave_0_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_sh(self, |state| {
            let mut slv0_config = Slv0Config::read(state)?;
            slv0_config.set_batch_ext_sens_0_en(val);
            slv0_config.write(state)
        })
    }

    /// Enable FIFO batching data of first slave.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of `batch_ext_sens_0_en` in reg `SLV0_CONFIG`.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn sh_batch_slave_0_get(&mut self) -> Result<u8, Error<B::Error>> {
        MemBank::operate_over_sh(self, |state| {
            let val = Slv0Config::read(state)?.batch_ext_sens_0_en();
            Ok(val)
        })
    }

    /// Enable FIFO batching data of second slave.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of batch_ext_sens_1_en in reg SLV1_CONFIG.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn sh_batch_slave_1_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_sh(self, |state| {
            let mut slv1_config = Slv1Config::read(state)?;
            slv1_config.set_batch_ext_sens_1_en(val);
            slv1_config.write(state)
        })
    }

    /// Enable FIFO batching data of second slave.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn sh_batch_slave_1_get(&mut self) -> Result<u8, Error<B::Error>> {
        MemBank::operate_over_sh(self, |state| {
            let val = Slv1Config::read(state)?.batch_ext_sens_1_en();
            Ok(val)
        })
    }

    /// Enable FIFO batching data of third slave.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of batch_ext_sens_2_en in reg SLV2_CONFIG.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`.
    pub fn sh_batch_slave_2_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_sh(self, |state| {
            let mut slv2_config = Slv2Config::read(state)?;
            slv2_config.set_batch_ext_sens_2_en(val);
            slv2_config.write(state)
        })
    }

    /// Enable FIFO batching data of third slave.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of `batch_ext_sens_2_en` in reg `SLV2_CONFIG`.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn sh_batch_slave_2_get(&mut self) -> Result<u8, Error<B::Error>> {
        MemBank::operate_over_sh(self, |state| {
            let val = Slv2Config::read(state)?.batch_ext_sens_2_en();
            Ok(val)
        })
    }

    /// Enable FIFO batching data of fourth slave.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of `batch_ext_sens_3_en` in reg `SLV3_CONFIG`.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn sh_batch_slave_3_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_sh(self, |state| {
            let mut slv3_config = Slv3Config::read(state)?;
            slv3_config.set_batch_ext_sens_3_en(val);
            slv3_config.write(state)
        })
    }

    /// Enable FIFO batching data of fourth slave.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of `batch_ext_sens_3_en` in reg `SLV3_CONFIG`.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn sh_batch_slave_3_get(&mut self) -> Result<u8, Error<B::Error>> {
        MemBank::operate_over_sh(self, |state| {
            let val = Slv3Config::read(state)?.batch_ext_sens_3_en();
            Ok(val)
        })
    }

    /// DEN functionality marking mode.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of den_mode in reg CTRL6_C.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn den_mode_set(&mut self, val: DenMode) -> Result<(), Error<B::Error>> {
        let mut ctrl6_c = Ctrl6C::read(self)?;
        ctrl6_c.set_den_mode(val as u8);
        ctrl6_c.write(self)
    }

    /// DEN functionality marking mode.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `DenMode`: Get the values of den_mode in reg CTRL6_C.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn den_mode_get(&mut self) -> Result<DenMode, Error<B::Error>> {
        let ctrl6_c = Ctrl6C::read(self)?;
        Ok(DenMode::try_from(ctrl6_c.den_mode()).unwrap_or_default())
    }

    /// DEN active level configuration.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of den_lh in reg CTRL9_XL.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn den_polarity_set(&mut self, val: DenLh) -> Result<(), Error<B::Error>> {
        let mut ctrl9_xl = Ctrl9Xl::read(self)?;
        ctrl9_xl.set_den_lh(val as u8);
        ctrl9_xl.write(self)
    }

    /// DEN active level configuration.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `DenLh`: Get the values of den_lh in reg CTRL9_XL.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn den_polarity_get(&mut self) -> Result<DenLh, Error<B::Error>> {
        let ctrl9_xl = Ctrl9Xl::read(self)?;
        Ok(DenLh::try_from(ctrl9_xl.den_lh()).unwrap_or_default())
    }

    /// DEN configuration.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of den_xl_g in reg CTRL9_XL.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn den_enable_set(&mut self, val: DenXlG) -> Result<(), Error<B::Error>> {
        let mut ctrl9_xl = Ctrl9Xl::read(self)?;
        ctrl9_xl.set_den_xl_g(val as u8);
        ctrl9_xl.write(self)
    }

    /// DEN configuration.
    ///
    /// # Arguments
    ///
    /// * `val`: Get the values of den_xl_g in reg CTRL9_XL.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn den_enable_get(&mut self) -> Result<DenXlG, Error<B::Error>> {
        let ctrl9_xl = Ctrl9Xl::read(self)?;
        Ok(DenXlG::try_from(ctrl9_xl.den_xl_g()).unwrap_or_default())
    }

    /// DEN value stored in LSB of X-axis.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of den_z in reg CTRL9_XL.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn den_mark_axis_x_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl9_xl = Ctrl9Xl::read(self)?;
        ctrl9_xl.set_den_x(val);
        ctrl9_xl.write(self)
    }

    /// DEN value stored in LSB of X-axis.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of den_z in reg CTRL9_XL.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn den_mark_axis_x_get(&mut self) -> Result<u8, Error<B::Error>> {
        let ctrl9_xl = Ctrl9Xl::read(self)?;
        Ok(ctrl9_xl.den_x())
    }

    /// DEN value stored in LSB of Y-axis.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of den_y in reg CTRL9_XL.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn den_mark_axis_y_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl9_xl = Ctrl9Xl::read(self)?;
        ctrl9_xl.set_den_y(val);
        ctrl9_xl.write(self)
    }

    /// DEN value stored in LSB of Y-axis.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of den_y in reg CTRL9_XL.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn den_mark_axis_y_get(&mut self) -> Result<u8, Error<B::Error>> {
        let ctrl9_xl = Ctrl9Xl::read(self)?;
        Ok(ctrl9_xl.den_y())
    }

    /// DEN value stored in LSB of Z-axis.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of den_x in reg CTRL9_XL.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn den_mark_axis_z_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl9_xl = Ctrl9Xl::read(self)?;
        ctrl9_xl.set_den_z(val);
        ctrl9_xl.write(self)
    }

    /// DEN value stored in LSB of Z-axis.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of den_x in reg CTRL9_XL.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn den_mark_axis_z_get(&mut self) -> Result<u8, Error<B::Error>> {
        let ctrl9_xl = Ctrl9Xl::read(self)?;
        Ok(ctrl9_xl.den_z())
    }

    /// Enable pedometer algorithm.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of pedo_en in reg EMB_FUNC_EN_A.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn pedo_sens_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_emb(self, |state| {
            let mut emb_func_en_a = EmbFuncEnA::read(state)?;
            emb_func_en_a.set_pedo_en(val);
            emb_func_en_a.write(state)
        })
    }

    /// Enable pedometer algorithm.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of pedo_en in reg EMB_FUNC_EN_A.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn pedo_sens_get(&mut self) -> Result<u8, Error<B::Error>> {
        MemBank::operate_over_emb(self, |state| {
            let emb_func_en_a = EmbFuncEnA::read(state)?;
            Ok(emb_func_en_a.pedo_en())
        })
    }

    /// Interrupt status bit for step detection.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of is_step_det in reg EMB_FUNC_STATUS.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn pedo_step_detect_get(&mut self) -> Result<u8, Error<B::Error>> {
        Ok(EmbFuncStatusMainpage::read(self)?.is_step_det())
    }

    /// Pedometer debounce configuration register (r/w).
    ///
    /// # Arguments
    ///
    /// * `val`: Buffer that contains data to write.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn pedo_debounce_steps_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        PedoDebStepsConf::from_bits(val).write(self)
    }

    /// Pedometer debounce configuration register (r/w).
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Buffer that stores data read.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn pedo_debounce_steps_get(&mut self) -> Result<u8, Error<B::Error>> {
        Ok(PedoDebStepsConf::read(self)?.deb_step())
    }

    /// Time period register for step detection on delta time (r/w).
    ///
    /// # Arguments
    ///
    /// * `val`: Buffer that contains data to write.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn pedo_steps_period_set(&mut self, val: u16) -> Result<(), Error<B::Error>> {
        PedoScDeltat::from_bits(val).write(self)
    }

    /// Time period register for step detection on delta time (r/w).
    ///
    /// # Arguments
    ///
    /// * `val`: Buffer that stores data read.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u16`: Interface status (MANDATORY: return 0 -> no Error).
    pub fn pedo_steps_period_get(&mut self) -> Result<u16, Error<B::Error>> {
        Ok(PedoScDeltat::read(self)?.pd_sc())
    }

    /// Set when user wants to generate interrupt on count overflow
    /// event/every step.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of carry_count_en in reg PEDO_CMD_REG.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn pedo_int_mode_set(&mut self, val: CarryCountEn) -> Result<(), Error<B::Error>> {
        let mut pedo_cmd_reg = PedoCmdReg::read(self)?;
        pedo_cmd_reg.set_carry_count_en(val as u8);
        pedo_cmd_reg.write(self)
    }

    /// Set when user wants to generate interrupt on count overflow
    /// event/every step.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `CarryCountEn`: Get the values of carry_count_en in reg PEDO_CMD_REG.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn pedo_int_mode_get(&mut self) -> Result<CarryCountEn, Error<B::Error>> {
        let pedo_cmd_reg = PedoCmdReg::read(self)?;
        Ok(CarryCountEn::try_from(pedo_cmd_reg.carry_count_en()).unwrap_or_default())
    }

    /// Enable significant motion detection function.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of `sign_motion_en` in register `EMB_FUNC_EN_A`.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn motion_sens_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_emb(self, |state| {
            let mut emb_func_en_a = EmbFuncEnA::read(state)?;
            emb_func_en_a.set_sign_motion_en(val);
            emb_func_en_a.write(state)
        })
    }

    /// Enable significant motion detection function.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of sign_motion_en in reg EMB_FUNC_EN_A.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn motion_sens_get(&mut self) -> Result<u8, Error<B::Error>> {
        MemBank::operate_over_emb(self, |state| {
            let emb_func_en_a = EmbFuncEnA::read(state)?;
            Ok(emb_func_en_a.sign_motion_en())
        })
    }

    /// Interrupt status bit for significant motion detection.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of is_sigmot in reg EMB_FUNC_STATUS.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn motion_flag_data_ready_get(&mut self) -> Result<u8, Error<B::Error>> {
        MemBank::operate_over_emb(self, |state| {
            let emb_func_status = EmbFuncStatus::read(state)?;
            Ok(emb_func_status.is_sigmot())
        })
    }

    /// Enable tilt calculation.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of tilt_en in reg EMB_FUNC_EN_A.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`.
    pub fn tilt_sens_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_emb(self, |state| {
            let mut emb_func_en_a = EmbFuncEnA::read(state)?;
            emb_func_en_a.set_tilt_en(val);
            emb_func_en_a.write(state)
        })
    }

    /// Enable tilt calculation.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of tilt_en in reg EMB_FUNC_EN_A.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn tilt_sens_get(&mut self) -> Result<u8, Error<B::Error>> {
        MemBank::operate_over_emb(self, |state| {
            let emb_func_en_a = EmbFuncEnA::read(state)?;
            Ok(emb_func_en_a.tilt_en())
        })
    }

    /// Interrupt status bit for tilt detection.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of is_tilt in reg EMB_FUNC_STATUS.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    pub fn tilt_flag_data_ready_get(&mut self) -> Result<u8, Error<B::Error>> {
        MemBank::operate_over_emb(self, |state| {
            let emb_func_status = EmbFuncStatus::read(state)?;
            Ok(emb_func_status.is_tilt())
        })
    }

    /// External magnetometer sensitivity value register.
    ///
    /// # Arguments
    ///
    /// * `val`: Buffer that contains data to write.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn mag_sensitivity_set(&mut self, val: u16) -> Result<(), Error<B::Error>> {
        MagSensitivity::from_bits(val).write(self)
    }

    /// External magnetometer sensitivity value register.
    ///
    /// # Arguments
    ///
    /// * `buff`: Buffer that stores data read.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u16`: Magnetic sensitivity value.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn mag_sensitivity_get(&mut self) -> Result<u16, Error<B::Error>> {
        Ok(MagSensitivity::read(self)?.mag_s())
    }

    /// Offset for hard-iron compensation register (r/w).
    ///
    /// # Arguments
    ///
    /// * `buff`: Buffer that contains data to write.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn mag_offset_set(&mut self, val: &[i16; 3]) -> Result<(), Error<B::Error>> {
        MagOffXYZ(*val).write(self)
    }

    /// Offset for hard-iron compensation register (r/w).
    ///
    /// # Arguments
    ///
    /// * `buff`: Buffer that stores data read.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn mag_offset_get(&mut self) -> Result<[i16; 3], Error<B::Error>> {
        Ok(MagOffXYZ::read(self)?.0)
    }

    /// Soft-iron (3x3 symmetric) matrix correction register (r/w).
    /// The value is expressed as half-precision floating-point format:
    /// SEEEEEFFFFFFFFFF
    /// S: 1 sign bit;
    /// E: 5 exponent bits;
    /// F: 10 fraction bits.
    ///
    /// # Arguments
    ///
    /// * `buff`: Buffer that contains data to write.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`.
    pub fn mag_soft_iron_set(&mut self, val: &[f16; 6]) -> Result<(), Error<B::Error>> {
        let mut raw: [u16; 6] = [0; 6];
        for (dst, src) in raw.iter_mut().zip(val.iter()) {
            *dst = src.to_bits();
        }

        MagSiMatrix(raw).write(self)
    }

    /// Soft-iron (3x3 symmetric) matrix correction register (r/w).
    /// The value is expressed as half-precision floating-point format:
    /// SEEEEEFFFFFFFFFF
    /// S: 1 sign bit;
    /// E: 5 exponent bits;
    /// F: 10 fraction bits.
    ///
    /// # Arguments
    ///
    /// * `buff`: Buffer that stores data read.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn mag_soft_iron_get(&mut self) -> Result<[f16; 6], Error<B::Error>> {
        // Read raw u16 matrix
        let raw = MagSiMatrix::read(self)?.0; // raw: [u16; 6]

        // Convert to f16
        let mut vals: [f16; 6] = [f16::from_bits(0); 6];
        for (dst, src) in vals.iter_mut().zip(raw.iter()) {
            *dst = f16::from_bits(*src);
        }

        Ok(vals)
    }

    /// Magnetometer Z-axis coordinates rotation (to be aligned to
    /// accelerometer/gyroscope axes orientation.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of mag_z_axis in reg MAG_CFG_A.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn mag_z_orient_set(&mut self, val: MagZAxis) -> Result<(), Error<B::Error>> {
        let mut reg = MagCfgA::read(self)?;
        reg.set_mag_z_axis(val as u8);
        reg.write(self)
    }

    /// Magnetometer Z-axis coordinates rotation (to be aligned to
    /// accelerometer/gyroscope axes orientation.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `MagZAxis`: Get the values of mag_z_axis in reg MAG_CFG_A.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn mag_z_orient_get(&mut self) -> Result<MagZAxis, Error<B::Error>> {
        let mag_cfg_a = MagCfgA::read(self)?;
        Ok(MagZAxis::try_from(mag_cfg_a.mag_z_axis()).unwrap_or(MagZAxis::ZEqY)) // note: other != default
    }

    /// Magnetometer Y-axis coordinates rotation (to be aligned to
    /// accelerometer/gyroscope axes orientation).
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of `mag_y_axis` in `reg MAG_CFG_A`.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn mag_y_orient_set(&mut self, val: MagYAxis) -> Result<(), Error<B::Error>> {
        let mut reg = MagCfgA::read(self)?;
        reg.set_mag_y_axis(val as u8);
        reg.write(self)
    }

    /// Magnetometer Y-axis coordinates rotation (to be aligned to
    /// accelerometer/gyroscope axes orientation).
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `MagYAxis`: Get the values of mag_y_axis in reg MAG_CFG_A.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn mag_y_orient_get(&mut self) -> Result<MagYAxis, Error<B::Error>> {
        let mag_cfg_a = MagCfgA::read(self)?;
        Ok(MagYAxis::try_from(mag_cfg_a.mag_y_axis()).unwrap_or_default())
    }

    /// Magnetometer X-axis coordinates rotation (to be aligned to
    /// accelerometer/gyroscope axes orientation.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of mag_x_axis in reg MAG_CFG_B.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn mag_x_orient_set(&mut self, val: MagXAxis) -> Result<(), Error<B::Error>> {
        let mut reg = MagCfgB::read(self)?;
        reg.set_mag_x_axis(val as u8);
        reg.write(self)
    }

    /// Magnetometer X-axis coordinates rotation (to be aligned to
    /// accelerometer/gyroscope axes orientation.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `MagXAxis`: Get the values of mag_x_axis in reg MAG_CFG_B.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn mag_x_orient_get(&mut self) -> Result<MagXAxis, Error<B::Error>> {
        let mag_cfg_b = MagCfgB::read(self)?;
        Ok(MagXAxis::try_from(mag_cfg_b.mag_x_axis()).unwrap_or(MagXAxis::XEqY)) // note: other != default
    }

    /// Interrupt status bit for FSM long counter timeout interrupt event.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of is_fsm_lc in reg EMB_FUNC_STATUS.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn long_cnt_flag_data_ready_get(&mut self) -> Result<u8, Error<B::Error>> {
        MemBank::operate_over_emb(self, |state| {
            let emb_func_status = EmbFuncStatus::read(state)?;
            Ok(emb_func_status.is_fsm_lc())
        })
    }

    /// Embedded final state machine functions mode.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of fsm_en in reg EMB_FUNC_EN_B.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn emb_fsm_en_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_emb(self, |state| {
            let mut emb_func_en_b = EmbFuncEnB::read(state)?;
            emb_func_en_b.set_fsm_en(val);
            emb_func_en_b.write(state)
        })
    }

    /// Embedded final state machine functions mode.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Get the values of fsm_en in reg EMB_FUNC_EN_B.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn emb_fsm_en_get(&mut self) -> Result<u8, Error<B::Error>> {
        MemBank::operate_over_emb(self, |state| {
            let emb_func_en_b = EmbFuncEnB::read(state)?;
            Ok(emb_func_en_b.fsm_en())
        })
    }

    /// Embedded final state machine functions mode.
    ///
    /// # Arguments
    ///
    /// * `val`: Structure of registers from FSM_ENABLE_A to FSM_ENABLE_B.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    pub fn fsm_enable_set(&mut self, val: EmbFsmEnable) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_emb(self, |state| {
            val.fsm_enable_a.write(state)?;
            val.fsm_enable_b.write(state)?;

            let mut emb_func_en_b = EmbFuncEnB::read(state)?;
            if (val.fsm_enable_a.into_bits() | val.fsm_enable_b.into_bits()) != PROPERTY_DISABLE {
                emb_func_en_b.set_fsm_en(PROPERTY_ENABLE);
            } else {
                emb_func_en_b.set_fsm_en(PROPERTY_DISABLE);
            }

            emb_func_en_b.write(state)
        })
    }

    /// Embedded final state machine functions mode.
    ///
    /// # Arguments
    ///
    /// * `val`: Structure of registers from FSM_ENABLE_A to FSM_ENABLE_B.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `EmbFsmEnable`: Structure of registers from FSM_ENABLE_A to FSM_ENABLE_B.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn fsm_enable_get(&mut self) -> Result<EmbFsmEnable, Error<B::Error>> {
        MemBank::operate_over_emb(self, |state| {
            let fsm_enable_a = FsmEnableA::read(state)?;
            let fsm_enable_b = FsmEnableB::read(state)?;

            Ok(EmbFsmEnable {
                fsm_enable_a,
                fsm_enable_b,
            })
        })
    }

    /// FSM long counter status register. Long counter value is an
    /// unsigned integer value (16-bit format).
    ///
    /// # Arguments
    ///
    /// * `buff`: Buffer that contains data to write.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn long_cnt_set(&mut self, val: u16) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_emb(self, |state| FsmLongCounter::from_bits(val).write(state))
    }

    /// FSM long counter status register. Long counter value is an
    /// unsigned integer value (16-bit format).
    ///
    /// # Arguments
    ///
    /// * `buff`: Buffer that stores data read.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn long_cnt_get(&mut self) -> Result<u16, Error<B::Error>> {
        MemBank::operate_over_emb(self, |state| Ok(FsmLongCounter::read(state)?.fsm_lc()))
    }

    /// Clear FSM long counter value.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of fsm_lc_clr in reg FSM_LONG_COUNTER_CLEAR.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn long_clr_set(&mut self, val: FsmLcClr) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_emb(self, |state| {
            let mut fsm_long_counter_clear = FsmLongCounterClear::read(state)?;
            fsm_long_counter_clear.set_fsm_lc_clr(val as u8);
            fsm_long_counter_clear.write(state)
        })
    }

    /// Clear FSM long counter value.
    ///
    /// # Arguments
    ///
    /// * `val`: Get the values of fsm_lc_clr in reg FSM_LONG_COUNTER_CLEAR.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn long_clr_get(&mut self) -> Result<FsmLcClr, Error<B::Error>> {
        MemBank::operate_over_emb(self, |state| {
            let reg = FsmLongCounterClear::read(state)?;
            Ok(FsmLcClr::try_from(reg.fsm_lc_clr()).unwrap_or_default())
        })
    }

    /// FSM output registers.
    ///
    /// # Arguments
    ///
    /// * `val`: Structure of registers from FSM_OUTS1 to FSM_OUTS16.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`: Interface status (MANDATORY: return Ok(()) -> no Error).
    pub fn fsm_out_get(&mut self, buf: &mut [u8], len: u8) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_emb(self, |state| {
            if len > buf.len() as u8 {
                return Err(Error::BufferTooSmall);
            }

            FsmOutsReg::read_more(state, &mut buf[0..len as usize])
        })
    }

    /// Finite State Machine ODR configuration.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of fsm_odr in reg EMB_FUNC_ODR_CFG_B.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn fsm_data_rate_set(&mut self, val: FsmOdr) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_emb(self, |state| {
            let mut emb_func_odr_cfg_b = EmbFuncOdrCfgB::read(state)?;
            emb_func_odr_cfg_b.set_fsm_odr(val as u8);
            emb_func_odr_cfg_b.write(state)
        })
    }

    /// Finite State Machine ODR configuration.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `FsmOdr`: Get the values of fsm_odr in reg EMB_FUNC_ODR_CFG_B.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn fsm_data_rate_get(&mut self) -> Result<FsmOdr, Error<B::Error>> {
        MemBank::operate_over_emb(self, |state| {
            let emb_func_odr_cfg_b = EmbFuncOdrCfgB::read(state)?;
            Ok(FsmOdr::try_from(emb_func_odr_cfg_b.fsm_odr()).unwrap_or_default())
        })
    }

    /// FSM initialization request.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of fsm_init in reg FSM_INIT.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn fsm_init_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_emb(self, |state| {
            let mut reg = EmbFuncInitB::read(state)?;
            reg.set_fsm_init(val);
            reg.write(state)
        })
    }

    /// FSM initialization request.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of fsm_init in reg FSM_INIT.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn fsm_init_get(&mut self) -> Result<u8, Error<B::Error>> {
        MemBank::operate_over_emb(self, |state| {
            let reg = EmbFuncInitB::read(state)?;
            Ok(reg.fsm_init())
        })
    }

    /// FSM long counter timeout register (r/w). The long counter
    /// timeout value is an unsigned integer value (16-bit format).
    /// When the long counter value reached this value, the FSM
    /// generates an interrupt.
    ///
    /// # Arguments
    ///
    /// * `val`: Buffer that contains data to write.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn fsm_lc_int_value_set(&mut self, val: u16) -> Result<(), Error<B::Error>> {
        FsmLcTimeout::from_bits(val).write(self)
    }

    /// FSM long counter timeout register (r/w). The long counter
    /// timeout value is an unsigned integer value (16-bit format).
    /// When the long counter value reached this value, the FSM generates
    /// an interrupt.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u16`: Buffer that stores data read.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn fsm_lc_int_value_get(&mut self) -> Result<u16, Error<B::Error>> {
        Ok(FsmLcTimeout::read(self)?.fsm_lc_timeout())
    }

    /// FSM number of programs register.
    ///
    /// # Arguments
    ///
    /// * `val`: Buffer that contains data to write.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn fsm_number_of_programs_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        if val > 16 {
            return Err(Error::InvalidFsmNumber);
        }
        FsmPrograms::from_bits(val).write(self)
    }

    /// FSM number of programs register.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Buffer that stores data read.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn fsm_number_of_programs_get(&mut self) -> Result<u8, Error<B::Error>> {
        Ok(FsmPrograms::read(self)?.fsm_prog())
    }

    /// FSM start address register (r/w). First available address is
    /// 0x033C.
    ///
    /// # Arguments
    ///
    /// * `val`: Buffer that contains data to write.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn fsm_start_address_set(&mut self, val: u16) -> Result<(), Error<B::Error>> {
        FsmStartAdd::from_bits(val).write(self)
    }

    /// FSM start address register (r/w). First available address
    /// is 0x033C.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u16`: Buffer that stores data read.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn fsm_start_address_get(&mut self) -> Result<u16, Error<B::Error>> {
        Ok(FsmStartAdd::read(self)?.fsm_start())
    }

    /// Enable Machine Learning Core.
    ///
    /// # Arguments
    ///
    /// * `val`: change the values of mlc_en in reg EMB_FUNC_EN_B and mlc_init in EMB_FUNC_INIT_B.
    pub fn mlc_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_emb(self, |state| {
            let mut reg = EmbFuncEnB::read(state)?;
            reg.set_mlc_en(val);
            reg.write(state)?;

            if val != PROPERTY_DISABLE {
                let mut init_reg = EmbFuncInitB::read(state)?;
                init_reg.set_mlc_init(val);
                init_reg.write(state)?;
            }

            Ok(())
        })
    }

    /// Enable Machine Learning Core.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Get the values of mlc_en in reg EMB_FUNC_EN_B.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn mlc_get(&mut self) -> Result<u8, Error<B::Error>> {
        MemBank::operate_over_emb(self, |state| Ok(EmbFuncEnB::read(state)?.mlc_en()))
    }

    /// Machine Learning Core status register
    ///
    /// # Arguments
    ///
    /// * `val`: register MLC_STATUS_MAINPAGE.
    pub fn mlc_status_get(&mut self) -> Result<MlcStatusMainpage, Error<B::Error>> {
        MlcStatusMainpage::read(self)
    }

    /// Machine Learning Core data rate selection.
    ///
    /// # Arguments
    ///
    /// * `val`: get the values of mlc_odr in reg EMB_FUNC_ODR_CFG_C.
    pub fn mlc_data_rate_set(&mut self, val: MlcOdr) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_emb(self, |state| {
            let mut reg = EmbFuncOdrCfgC::read(state)?;
            reg.set_mlc_odr(val as u8);
            reg.write(state)
        })
    }

    /// Machine Learning Core data rate selection.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `MlcOdr`: change the values of mlc_odr in
    ///     * `Err`: Returns an error if the operation fails.
    pub fn mlc_data_rate_get(&mut self) -> Result<MlcOdr, Error<B::Error>> {
        MemBank::operate_over_emb(self, |state| {
            let reg = EmbFuncOdrCfgC::read(state)?;
            Ok(MlcOdr::try_from(reg.mlc_odr()).unwrap_or_default())
        })
    }

    /// prgsens_out:  Output value of all MLCx decision trees.
    ///
    /// # Arguments
    ///
    /// * `buff`: buffer that stores data read.
    pub fn mlc_out_get(&mut self, buf: &mut [u8]) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_emb(self, |state| MlcSrcReg::read_more(state, buf))
    }

    /// External magnetometer sensitivity value register for
    /// Machine Learning Core.
    ///
    /// # Arguments
    ///
    /// * `buff`: buffer that contains data to write.
    pub fn mlc_mag_sensitivity_set(&mut self, val: f16) -> Result<(), Error<B::Error>> {
        MlcMagSensitivity::from_bits(val.to_bits()).write(self)
    }

    /// External magnetometer sensitivity value register for
    /// Machine Learning Core.
    ///
    /// # Arguments
    ///
    /// * `buff`: buffer that stores data read.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `f16`: Sensitivity value.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn mlc_mag_sensitivity_get(&mut self) -> Result<f16, Error<B::Error>> {
        Ok(f16::from_bits(MlcMagSensitivity::read(self)?.mlc_mag_s()))
    }

    /// Sensor hub output registers.
    ///
    /// # Arguments
    ///
    /// * `len`: Length of the registers to read.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn sh_read_data_raw_get(&mut self, buf: &mut [u8], len: u8) -> Result<(), Error<B::Error>> {
        if len > buf.len() as u8 {
            return Err(Error::BufferTooSmall);
        }

        MemBank::operate_over_sh(self, |state| {
            SensorHubReg::read_more(state, &mut buf[0..len as usize])
        })
    }

    /// Number of external sensors to be read by the sensor hub.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of aux_sens_on in reg MASTER_CONFIG.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn sh_slave_connected_set(&mut self, val: AuxSensOn) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_sh(self, |state| {
            let mut master_config = MasterConfig::read(state)?;
            master_config.set_aux_sens_on(val as u8);
            master_config.write(state)
        })
    }

    /// Number of external sensors to be read by the sensor hub.
    ///
    /// # Arguments
    ///
    /// * `val`: Get the values of aux_sens_on in reg MASTER_CONFIG.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn sh_slave_connected_get(&mut self) -> Result<AuxSensOn, Error<B::Error>> {
        MemBank::operate_over_sh(self, |state| {
            let val = MasterConfig::read(state)?.aux_sens_on();
            Ok(AuxSensOn::try_from(val).unwrap_or_default())
        })
    }

    /// Sensor hub I2C master enable.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of master_on in reg MASTER_CONFIG.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn sh_master_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_sh(self, |state| {
            let mut master_config = MasterConfig::read(state)?;
            master_config.set_master_on(val);
            master_config.write(state)
        })
    }

    /// Sensor hub I2C master enable.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of master_on in reg MASTER_CONFIG.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn sh_master_get(&mut self) -> Result<u8, Error<B::Error>> {
        MemBank::operate_over_sh(self, |state| {
            let val = MasterConfig::read(state)?.master_on();
            Ok(val)
        })
    }

    /// Master I2C pull-up enable.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of shub_pu_en in reg MASTER_CONFIG.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn sh_pin_mode_set(&mut self, val: ShubPuEn) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_sh(self, |state| {
            let mut master_config = MasterConfig::read(state)?;
            master_config.set_shub_pu_en(val as u8);
            master_config.write(state)
        })
    }

    /// Master I2C pull-up enable.
    ///
    /// # Arguments
    ///
    /// * `val`: Get the values of shub_pu_en in reg MASTER_CONFIG.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    pub fn sh_pin_mode_get(&mut self) -> Result<ShubPuEn, Error<B::Error>> {
        MemBank::operate_over_sh(self, |state| {
            let val = MasterConfig::read(state)?.shub_pu_en();
            Ok(ShubPuEn::try_from(val).unwrap_or_default())
        })
    }

    /// I2C interface pass-through.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of pass_through_mode in reg MASTER_CONFIG.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn sh_pass_through_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_sh(self, |state| {
            let mut master_config = MasterConfig::read(state)?;
            master_config.set_pass_through_mode(val);
            master_config.write(state)
        })
    }

    /// I2C interface pass-through.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Change the values of pass_through_mode in reg MASTER_CONFIG.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn sh_pass_through_get(&mut self) -> Result<u8, Error<B::Error>> {
        MemBank::operate_over_sh(self, |state| {
            let val = MasterConfig::read(state)?.pass_through_mode();
            Ok(val)
        })
    }

    /// Sensor hub trigger signal selection.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of start_config in reg MASTER_CONFIG.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn sh_syncro_mode_set(&mut self, val: StartConfig) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_sh(self, |state| {
            let mut master_config = MasterConfig::read(state)?;
            master_config.set_start_config(val as u8);
            master_config.write(state)
        })
    }

    /// Sensor hub trigger signal selection.
    ///
    /// # Arguments
    ///
    /// * `val`: Get the values of start_config in reg MASTER_CONFIG.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn sh_syncro_mode_get(&mut self) -> Result<StartConfig, Error<B::Error>> {
        MemBank::operate_over_sh(self, |state| {
            let val = MasterConfig::read(state)?.start_config();
            Ok(StartConfig::try_from(val).unwrap_or_default())
        })
    }

    /// Slave 0 write operation is performed only at the first sensor
    /// hub cycle.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of write_once in reg MASTER_CONFIG.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn sh_write_mode_set(&mut self, val: WriteOnce) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_sh(self, |state| {
            let mut master_config = MasterConfig::read(state)?;
            master_config.set_write_once(val as u8);
            master_config.write(state)
        })
    }

    /// Slave 0 write operation is performed only at the first sensor
    /// hub cycle.
    ///
    /// # Arguments
    ///
    /// * `val`: Get the values of write_once in reg MASTER_CONFIG.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`: No Error.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn sh_write_mode_get(&mut self) -> Result<WriteOnce, Error<B::Error>> {
        MemBank::operate_over_sh(self, |state| {
            let val = MasterConfig::read(state)?.write_once();
            Ok(WriteOnce::try_from(val).unwrap_or_default())
        })
    }

    /// Reset Master logic and output registers.
    pub fn sh_reset_set(&mut self) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_sh(self, |state| {
            let mut master_config = MasterConfig::read(state)?;

            master_config.set_rst_master_regs(PROPERTY_ENABLE);
            master_config.write(state)?;

            master_config.set_rst_master_regs(PROPERTY_DISABLE);
            master_config.write(state)
        })
    }

    /// Reset Master logic and output registers.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of rst_master_regs in reg MASTER_CONFIG.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`: Interface status (MANDATORY: return Ok(()) -> no Error).
    pub fn sh_reset_get(&mut self) -> Result<u8, Error<B::Error>> {
        MemBank::operate_over_sh(self, |state| {
            let val = MasterConfig::read(state)?.rst_master_regs();
            Ok(val)
        })
    }

    /// Rate at which the master communicates.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of `shub_odr` in register `SLV0_CONFIG`.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn sh_data_rate_set(&mut self, val: ShubOdr) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_sh(self, |state| {
            let mut slv0_config = Slv0Config::read(state)?;
            slv0_config.set_shub_odr(val as u8);
            slv0_config.write(state)
        })
    }

    /// Rate at which the master communicates.
    ///
    /// # Arguments
    ///
    /// * `val`: Get the values of shub_odr in reg slv1_CONFIG.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`.
    pub fn sh_data_rate_get(&mut self) -> Result<ShubOdr, Error<B::Error>> {
        MemBank::operate_over_sh(self, |state| {
            let val = Slv0Config::read(state)?.shub_odr();
            Ok(ShubOdr::try_from(val).unwrap_or_default())
        })
    }

    /// Configure slave 0 for perform a write.
    ///
    /// # Arguments
    ///
    /// * `val`: Structure that contains
    ///     - `slv0_add`: 8 bit i2c device address
    ///     - `slv0_subadd`: 8 bit register device address
    ///     - `slv0_data`: 8 bit data to write.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn sh_cfg_write(&mut self, val: &ShCfgWrite) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_sh(self, |state| {
            let mut slv0_add = Slv0Add::default();
            slv0_add.set_slave0(val.slv0_add >> 1);
            slv0_add.set_rw_0(0);

            slv0_add.write(state)?;
            Slv0Subadd::from_bits(val.slv0_subadd).write(state)?;
            DatawriteSlv0::from_bits(val.slv0_data).write(state)
        })
    }

    /// Configure slave 0 for perform a write/read.
    ///
    /// # Arguments
    ///
    /// * `val`: Structure that contains
    ///     - `slv_add`: 8 bit i2c device address
    ///     - `slv_subadd`: 8 bit register device address
    ///     - `slv_len`: num of bit to read.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`: Interface status (MANDATORY: return Ok(()) -> no Error).
    pub fn sh_slv0_cfg_read(&mut self, val: &ShCfgRead) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_sh(self, |state| {
            let mut slv0_add = Slv0Add::default();
            slv0_add.set_slave0(val.slv_add >> 1);
            slv0_add.set_rw_0(1);

            slv0_add.write(state)?;
            Slv0Subadd::from_bits(val.slv_subadd).write(state)?;

            let mut slv0_config = Slv0Config::read(state)?;
            slv0_config.set_slave0_numop(val.slv_len);
            slv0_config.write(state)
        })
    }

    /// Configure slave 1 for perform a write/read.
    ///
    /// # Arguments
    ///
    /// * `val`: Structure that contains
    ///     - `slv_add`: 8 bit i2c device address
    ///     - `slv_subadd`: 8 bit register device address
    ///     - `slv_len`: number of bits to read.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`: No Error.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn sh_slv1_cfg_read(&mut self, val: &ShCfgRead) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_sh(self, |state| {
            let mut slv1_add = Slv1Add::default();
            slv1_add.set_slave1_add(val.slv_add >> 1);
            slv1_add.set_r_1(1);

            slv1_add.write(state)?;
            Slv1Subadd::from_bits(val.slv_subadd).write(state)?;

            let mut slv1_config = Slv1Config::read(state)?;
            slv1_config.set_slave1_numop(val.slv_len);
            slv1_config.write(state)
        })
    }

    /// Configure slave 2 for perform a write/read.
    ///
    /// # Arguments
    ///
    /// * `val`: Structure that contains
    ///     - `slv_add`: 8 bit i2c device address
    ///     - `slv_subadd`: 8 bit register device address
    ///     - `slv_len`: number of bits to read.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn sh_slv2_cfg_read(&mut self, val: &ShCfgRead) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_sh(self, |state| {
            let mut slv2_add = Slv2Add::default();
            slv2_add.set_slave2_add(val.slv_add >> 1);
            slv2_add.set_r_2(1);

            slv2_add.write(state)?;
            Slv2Subadd::from_bits(val.slv_subadd).write(state)?;

            let mut slv2_config = Slv2Config::read(state)?;
            slv2_config.set_slave2_numop(val.slv_len);
            slv2_config.write(state)
        })
    }

    /// Configure slave 3 for perform a write/read.
    ///
    /// # Arguments
    ///
    /// * `val`: Structure that contains
    ///     - `uint8_t slv_add`: 8 bit i2c device address
    ///     - `uint8_t slv_subadd`: 8 bit register device address
    ///     - `uint8_t slv_len`: num of bit to read
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn sh_slv3_cfg_read(&mut self, val: &ShCfgRead) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_sh(self, |state| {
            let mut slv3_add = Slv3Add::default();
            slv3_add.set_slave3_add(val.slv_add >> 1);
            slv3_add.set_r_3(1);

            slv3_add.write(state)?;
            Slv3Subadd::from_bits(val.slv_subadd).write(state)?;

            let mut slv3_config = Slv3Config::read(state)?;
            slv3_config.set_slave3_numop(val.slv_len);
            slv3_config.write(state)
        })
    }

    /// Sensor hub source register.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn sh_status_get(&mut self) -> Result<StatusMaster, Error<B::Error>> {
        MemBank::operate_over_sh(self, |state| StatusMaster::read(state))
    }
}

/// @brief  Convert raw data from full-scale �2g to milligrams.
///
/// # Arguments
///
/// * `lsb`: Raw data in LSB.
pub fn from_fs2g_to_mg(lsb: i16) -> f32 {
    (lsb as f32) * 0.061
}

/// @brief  Converts a raw value from 4g full-scale to milligals.
///
/// # Arguments
///
/// * `lsb`: Raw value in 16-bit signed integer format.
///
/// # Returns
///
/// * `f32`: Converted value in milligals.
pub fn from_fs4g_to_mg(lsb: i16) -> f32 {
    (lsb as f32) * 0.122
}

/// @brief  Convert from full-scale 8g to mg.
///
/// # Arguments
///
/// * `lsb`: Value in LSB to convert.
///
/// # Returns
///
/// * `f32`: Converted value in mg.
pub fn from_fs8g_to_mg(lsb: i16) -> f32 {
    (lsb as f32) * 0.244
}

/// @brief  Convert from full-scale 16g to milligrams.
///
/// # Arguments
///
/// * `lsb`: Value in LSB (Least Significant Bit).
///
/// # Returns
///
/// * `Result`
///     * `f32`: Value converted to milligrams.
pub fn from_fs16g_to_mg(lsb: i16) -> f32 {
    (lsb as f32) * 0.488
}

/// Convert from full scale 125 dps to milli degrees per second.
///
/// # Arguments
///
/// * `lsb`: The value in LSB to be converted.
///
/// # Returns
///
/// * `Result<f32, Error<B::Error>>`: The converted value in milli degrees per second.
pub fn from_fs125dps_to_mdps(lsb: i16) -> f32 {
    (lsb as f32) * 4.375
}

/// @brief  Convert from full-scale 250 dps to milli-degrees per second.
///
/// # Arguments
///
/// * `lsb`: The value in LSB to convert.
///
/// # Returns
///
/// * `Result`
///     * `f32`: The converted value in milli-degrees per second.
pub fn from_fs250dps_to_mdps(lsb: i16) -> f32 {
    (lsb as f32) * 8.75
}

/// @brief  Convert from full-scale 500 dps to milli-degrees per second.
///
/// # Arguments
///
/// * `lsb`: Value in LSB to be converted.
///
/// # Returns
///
/// * `Result`
///     * `f32`: Converted value in milli-degrees per second.
pub fn from_fs500dps_to_mdps(lsb: i16) -> f32 {
    (lsb as f32) * 17.50
}

/// @brief  Convert from full-scale 1000 dps to milli-degrees per second.
///
/// # Arguments
///
/// * `lsb`: Value in LSB to be converted.
///
/// # Returns
///
/// * `Result`
///     * `f32`: Converted value in milli-degrees per second.
pub fn from_fs1000dps_to_mdps(lsb: i16) -> f32 {
    (lsb as f32) * 35.0
}

/// @brief  Convert from full-scale 2000 dps to milli-degrees per second.
///
/// # Arguments
///
/// * `lsb`: The value in LSB to convert.
///
/// # Returns
///
/// * `Result`
///     * `f32`: The converted value in milli-degrees per second.
pub fn from_fs2000dps_to_mdps(lsb: i16) -> f32 {
    (lsb as f32) * 70.0
}

/// @brief  Convert from full scale 4000 dps to milli degrees per second.
///
/// # Arguments
///
/// * `lsb`: Value in LSB.
pub fn from_fs4000dps_to_mdps(lsb: i16) -> f32 {
    (lsb as f32) * 140.0
}

/// @brief  Convert LSB to Celsius.
///
/// # Arguments
///
/// * `lsb`: The value in LSB to convert.
///
/// # Returns
///
/// * `f32`: The temperature in Celsius.
pub fn from_lsb_to_celsius(lsb: i16) -> f32 {
    (lsb as f32 / 256.0) + 25.0
}

/// @brief  Convert LSB value to nanoseconds.
///
/// # Arguments
///
/// * `lsb`: The LSB value to convert.
///
/// # Returns
///
/// * `u64`: The converted value in nanoseconds.
pub fn from_lsb_to_nsec(lsb: u32) -> u64 {
    (lsb as u64) * 25000
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq)]
pub enum I2CAddress {
    /// I²C address when the SA0 pin is low.
    I2cAddL = 0x6A,

    /// I²C address when the SA0 pin is high.
    I2cAddH = 0x6B,
}

///
/// ISM330DHCX Device ID.
///
pub const ISM330DHCX_ID: u8 = 0x6B;

pub const PROPERTY_ENABLE: u8 = 1;
pub const PROPERTY_DISABLE: u8 = 0;
