use st_mems_reg_config_conv::parser;
use std::path::Path;

fn main() {
    // Source file:
    // https://github.com/STMicroelectronics/st-mems-machine-learning-core/blob/main/examples/6d_position_recognition/ism330dhcx/ism330dhcx_six_d_position.json
    let input_file = Path::new("ism330dhcx_six_d_position.json");
    let output_file = Path::new("src/mlc_config.rs");
    parser::generate_rs_from_json(input_file, output_file, "SIX_D", "ISM330DHCX", false);
    println!("cargo:rerun-if-changed=ism330dhcx_six_d_position.json");

    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}
