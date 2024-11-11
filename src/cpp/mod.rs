use crate::helpers::register_helpers;
use crate::redf::{Endpoint, Redf};
use crate::utils::get_ros_types;
use convert_case::{Case, Casing};
use serde::Serialize;
use std::collections::BTreeSet;
use std::env;
use std::error::Error;
use std::path::Path;
use tera::Tera;

const SUPPORTED_DISTRO: &[&str] = &["humble", "iron", "jazzy"];

#[derive(Serialize)]
struct Context<'a> {
    redf: &'a Redf,
    distro: String,
    project_name: String,
    packages: BTreeSet<String>,
    ros_types: BTreeSet<String>,
    namespace: String,
    includes: Vec<String>,
    have_actions: bool,
}

pub fn generate(redf: &Redf, outdir: &Path) -> Result<(), Box<dyn Error>> {
    let mut tera = Tera::default();
    register_helpers(&mut tera);
    tera.add_raw_template("package.xml", include_str!("package.xml.j2"))?;
    tera.add_raw_template("CMakeLists.txt", include_str!("CMakeLists.txt.j2"))?;
    tera.add_raw_template("header.hpp", include_str!("header.hpp.j2"))?;

    let project_name = redf.title.to_case(Case::Snake).clone();
    let ros_types: BTreeSet<String> = get_ros_types(&redf);
    let packages: BTreeSet<String> = ros_types
        .iter()
        .map(|msg| match msg.split_once("/") {
            Some((prefix, _)) => prefix.to_string(),
            None => msg.clone(),
        })
        .collect();
    let namespace = match &redf.cpp_namespace {
        Some(ns) => ns.clone(),
        None => redf.title.to_case(Case::Snake),
    };
    let includes: Vec<String> = ros_types
        .iter()
        .map(|msg| format!("#include \"{}.hpp\"", msg))
        .collect();

    let have_actions = redf.endpoints.iter().any(|i| match i {
        Endpoint::Action(_) => true,
        _ => false,
    });

    let distro = match env::var("ROS_DISTRO") {
        Ok(val) => val,
        Err(_) => panic!("Fail to detect ROS version, make sure ROS is sourced"),
    };
    if !SUPPORTED_DISTRO.contains(&distro.as_str()) {
        println!("WARN: [${distro}] is not a supported ROS distro");
    }

    let ctx = tera::Context::from_serialize(Context {
        redf,
        distro,
        project_name: project_name.clone(),
        packages,
        ros_types,
        namespace,
        includes,
        have_actions,
    })?;

    let f = std::fs::File::create(outdir.join("package.xml"))?;
    tera.render_to("package.xml", &ctx, f)?;
    let f = std::fs::File::create(outdir.join("CMakeLists.txt"))?;
    tera.render_to("CMakeLists.txt", &ctx, f)?;
    let f = std::fs::File::create(outdir.join(format!("{}.hpp", &project_name)))?;
    tera.render_to("header.hpp", &ctx, f)?;

    Ok(())
}
