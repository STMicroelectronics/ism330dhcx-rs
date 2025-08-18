use st_mems_reg_config_conv::parser;
use std::path::Path;

fn main() {
    // Parse ucf file
    let input_file = Path::new("ism330dhcx_six_d_position.ucf");
    let output_file = Path::new("src/mlc_config.rs");
    parser::generate_rs_from_ucf(input_file, output_file, "SIX_D");
    println!("cargo:rerun-if-changed=ism330dhcx_six_d_position.ucf");

    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}
