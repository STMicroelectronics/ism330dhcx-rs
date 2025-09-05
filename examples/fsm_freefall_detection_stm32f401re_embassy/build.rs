use st_mems_reg_config_conv::parser;
use std::path::Path;

fn main() {
    // Source file:
    // https://github.com/STMicroelectronics/st-mems-finite-state-machine/blob/main/examples/free_fall_detection/ism330dhcx/ism330dhcx_free_fall.json
    let input_file = Path::new("ism330dhcx_free_fall.json");
    let output_file = Path::new("src/mlc_config.rs");
    parser::generate_rs_from_json(input_file, output_file, "FREE_FALL", "ISM330DHCX", false);
    println!("cargo:rerun-if-changed=ism330dhcx_free_fall.json");

    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}
