use std::path::Path;

fn assert_command_success(result: std::process::Output) {
    if !result.status.success() {
        eprintln!("{}", String::from_utf8(result.stderr).unwrap());
        panic!();
    }
}

#[test]
fn cpp() -> Result<(), Box<dyn std::error::Error>> {
    let target = env!("CARGO_BIN_EXE_redf");
    let testdir = Path::new(env!("CARGO_TARGET_TMPDIR")).join("cpp");
    let outdir = testdir.join("redf");
    let srcdir = testdir.join("src");

    if testdir.exists() {
        std::fs::remove_dir_all(testdir)?;
    }

    let result = std::process::Command::new(target)
        .args([
            "--out",
            outdir.to_str().unwrap(),
            "--gen=cpp",
            "tests/test_api.redf.yaml",
        ])
        .output()?;
    assert_command_success(result);

    let result = std::process::Command::new("cp")
        .args(["-r", "tests/cpp", srcdir.to_str().unwrap()])
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
            &format!(
                ". '{}/install/setup.bash' && colcon build",
                outdir.to_str().unwrap()
            ),
        ])
        .current_dir(&srcdir)
        .output()?;
    if !result.status.success() {
        eprintln!("{}", String::from_utf8(result.stderr)?);
        panic!();
    }

    Ok(())
}
