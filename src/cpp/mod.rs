use crate::helpers::register_helpers;
use crate::redf::{Endpoint, Redf};
use convert_case::{Case, Casing};
use serde::Serialize;
use std::collections::BTreeSet;
use std::error::Error;
use std::path::Path;
use tera::Tera;

#[derive(Serialize)]
struct Context<'a> {
    redf: &'a Redf,
    project_name: String,
    packages: Vec<String>,
    messages: BTreeSet<String>,
    namespace: String,
    includes: Vec<String>,
}

pub fn generate(redf: &Redf, outdir: &Path) -> Result<(), Box<dyn Error>> {
    let mut tera = Tera::default();
    register_helpers(&mut tera);
    tera.add_raw_template("package.xml", include_str!("package.xml.j2"))?;
    tera.add_raw_template("CMakeLists.txt", include_str!("CMakeLists.txt.j2"))?;
    tera.add_raw_template("header.hpp", include_str!("header.hpp.j2"))?;

    let project_name = redf.title.to_case(Case::Snake).clone();
    let messages: BTreeSet<String> = redf
        .endpoints
        .iter()
        .map(|ep| match ep {
            Endpoint::Topic(ep) => ep.message_type.clone(),
            Endpoint::Service(ep) => ep.service_type.clone(),
        })
        .collect();
    let packages: Vec<String> = messages
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
    let includes: Vec<String> = messages
        .iter()
        .map(|msg| format!("#include \"{}.hpp\"", msg))
        .collect();

    let ctx = tera::Context::from_serialize(Context {
        redf,
        project_name: project_name.clone(),
        packages,
        messages,
        namespace,
        includes,
    })?;

    let f = std::fs::File::create(outdir.join("package.xml"))?;
    tera.render_to("package.xml", &ctx, f)?;
    let f = std::fs::File::create(outdir.join("CMakeLists.txt"))?;
    tera.render_to("CMakeLists.txt", &ctx, f)?;
    let f = std::fs::File::create(outdir.join(format!("{}.hpp", &project_name)))?;
    tera.render_to("header.hpp", &ctx, f)?;

    Ok(())
}
