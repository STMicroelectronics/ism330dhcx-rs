use crate::BusOperation;
use crate::MemBankFunctions;
use crate::{Error, Ism330dhcx};
use derive_more::TryFrom;
use embedded_hal::delay::DelayNs;
use st_mem_bank_macro::mem_bank;

use crate::register::{
    embedded::{
        EmbFuncInt1, EmbFuncInt2, EmbFuncStatus, FsmInt1A, FsmInt1B, FsmInt2A, FsmInt2B,
        FsmStatusA, FsmStatusB, MlcInt1, MlcInt2,
    },
    main::{
        AllIntSrc, D6dSrc, Int1Ctrl, Int2Ctrl, Md1Cfg, Md2Cfg, MlcStatusMainpage, StatusReg,
        TapSrc, WakeUpSrc,
    },
};

pub mod advanced;
pub mod embedded;
pub mod main;
pub mod sensor_hub;

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom, Debug)]
#[try_from(repr)]
#[mem_bank(Ism330dhcx, generics = 2)]
pub enum MemBank {
    #[default]
    #[main]
    UserBank = 0,
    #[state(SensHubBankState, fn_name = "operate_over_sh")]
    SensorHubBank = 1,
    #[state(EmbFuncBankState, fn_name = "operate_over_emb")]
    EmbeddedFuncBank = 2,
}

#[derive(Default)]
pub struct AllSources {
    pub all_int_src: AllIntSrc,
    pub wake_up_src: WakeUpSrc,
    pub tap_src: TapSrc,
    pub d6d_src: D6dSrc,
    pub status_reg: StatusReg,
    pub emb_func_status: EmbFuncStatus,
    pub fsm_status_a: FsmStatusA,
    pub fsm_status_b: FsmStatusB,
    pub mlc_status: MlcStatusMainpage,
}

#[derive(Default)]
pub struct PinInt1Route {
    pub int1_ctrl: Int1Ctrl,
    pub md1_cfg: Md1Cfg,
    pub emb_func_int1: EmbFuncInt1,
    pub fsm_int1_a: FsmInt1A,
    pub fsm_int1_b: FsmInt1B,
    pub mlc_int1: MlcInt1,
}

#[derive(Default)]
pub struct PinInt2Route {
    pub int2_ctrl: Int2Ctrl,
    pub md2_cfg: Md2Cfg,
    pub emb_func_int2: EmbFuncInt2,
    pub fsm_int2_a: FsmInt2A,
    pub fsm_int2_b: FsmInt2B,
    pub mlc_int2: MlcInt2,
}
