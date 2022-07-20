fn assert_command_success(result: std::process::Output) {
    if !result.status.success() {
        eprintln!("{}", String::from_utf8(result.stderr).unwrap());
        panic!();
    }
}

#[test]
fn cpp() -> Result<(), Box<dyn std::error::Error>> {
    let target = env!("CARGO_BIN_EXE_redf");
    let outdir = format!("{}/cpp/redf", env!("CARGO_TARGET_TMPDIR"));
    let srcdir = format!("{}/cpp/src", env!("CARGO_TARGET_TMPDIR"));

    let result = std::process::Command::new(target)
        .args(["--out", &outdir, "--gen=cpp", "tests/test_api.redf.yaml"])
        .output()?;
    assert_command_success(result);

    let result = std::process::Command::new("cp")
        .args(["-r", "tests/cpp", &srcdir])
        .output()?;
    assert_command_success(result);

    let result = std::process::Command::new("colcon")
        .arg("build")
        .current_dir(&outdir)
        .output()?;
    assert_command_success(result);

    let result = std::process::Command::new("bash")
        .args([
            "-c",
            &format!(". '{}/install/setup.bash' && colcon build", &outdir),
        ])
        .current_dir(&srcdir)
        .output()?;
    if !result.status.success() {
        eprintln!("{}", String::from_utf8(result.stderr)?);
        panic!();
    }

    Ok(())
}
