use std::path::Path;

#[test]
fn cpp() -> Result<(), Box<dyn std::error::Error>> {
    let target = env!("CARGO_BIN_EXE_redf");
    let outdir = format!("{}/cpp", env!("CARGO_TARGET_TMPDIR"));

    let result = std::process::Command::new(target)
        .args([
            "--out",
            &outdir,
            "--gen=cpp",
            "tests/test_api.yaml",
        ])
        .output()?;
    if !result.status.success() {
        eprintln!("{}", String::from_utf8(result.stderr)?);
        panic!();
    }

    let package_xml = std::fs::read_to_string(Path::new(&outdir).join("package.xml"))?;
    let package_xml = package_xml.replace("[PUT MAINTAINER EMAIL HERE]", "test@test.com");
    let package_xml = package_xml.replace("[PUT MAINTAINER NAME HERE]", "test");
    let package_xml = package_xml.replace("[PUT LICENSE HERE]", "test");
    std::fs::write(Path::new(&outdir).join("package.xml"), package_xml)?;
    let result = std::process::Command::new("colcon")
        .arg("build")
        .current_dir(outdir)
        .output()?;
    if !result.status.success() {
        eprintln!("{}", String::from_utf8(result.stderr)?);
        panic!();
    }

    Ok(())
}
