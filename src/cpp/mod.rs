use crate::helpers::register_helpers;
use crate::redf::{
    DurabilityPolicy, Endpoint, LivelinessPolicy, Qos, QosPreset, QosProfile, Redf,
    ReliabilityPolicy,
};
use convert_case::{Case, Casing};
use handlebars::Handlebars;
use serde::Serialize;
use std::collections::BTreeSet;
use std::error::Error;
use std::path::Path;

fn qos_profile(profile: QosProfile) -> String {
    let reliability = match profile.reliability {
        Some(ReliabilityPolicy::BestEffort) => "rclcpp::ReliabilityPolicy::BestEffort",
        Some(ReliabilityPolicy::Reliable) => "rclcpp::ReliabilityPolicy::Reliable",
        _ => "rclcpp::ReliabilityPolicy::SystemDefault",
    };
    let durability = match profile.durability {
        Some(DurabilityPolicy::TransientLocal) => "rclcpp::DurabilityPolicy::TransientLocal",
        Some(DurabilityPolicy::Volatile) => "rclcpp::DurabilityPolicy::Volatile",
        _ => "rclcpp::DurabilityPolicy::SystemDefault",
    };
    let liveliness = match profile.liveliness {
        Some(LivelinessPolicy::Automatic) => "rclcpp::LivelinessPolicy::Automatic",
        Some(LivelinessPolicy::ManualByTopic) => "rclcpp::LivelinessPolicy::ManualByTopic",
        _ => "rclcpp::LivelinessPolicy::SystemDefault",
    };
    let deadline = match profile.deadline {
        Some(d) => format!("rclcpp::Duration{{{}, {}}}", d.secs, d.nanosecs),
        None => "".to_string(),
    };
    let lifespan = match profile.lifespan {
        Some(d) => format!("rclcpp::Duration{{{}, {}}}", d.secs, d.nanosecs),
        None => "".to_string(),
    };
    let lease_duration = match profile.lease_duration {
        Some(d) => format!("rclcpp::Duration{{{}, {}}}", d.secs, d.nanosecs),
        None => "".to_string(),
    };
    let avoid_ros_namespace_conventions = match profile.avoid_ros_namespace_conventions {
        Some(b) => format!("{b}"),
        None => "".to_string(),
    };

    format!(
        r#"[history_policy, depth]() {{
      rclcpp::QoS qos{{depth}};
      qos.history(history_policy);
      qos.reliability({reliability});
      qos.durability({durability});
      qos.liveliness({liveliness});
      qos.deadline({deadline});
      qos.lifespan({lifespan});
      qos.liveliness_lease_duration({lease_duration});
      qos.avoid_ros_namespace_conventions({avoid_ros_namespace_conventions});
      return qos;
    }}()"#
    )
}

fn qos_preset(preset: QosPreset) -> String {
    match preset {
        QosPreset::Clock => "rclcpp::ClockQoS{}".to_string(),
        QosPreset::ParameterEvents => "rclcpp::ParameterEventsQoS{}".to_string(),
        QosPreset::Parameters => "rclcpp::ParametersQoS{}".to_string(),
        QosPreset::Rosout => "rclcpp::RosoutQoS{}".to_string(),
        QosPreset::SensorData => "rclcpp::SensorDataQoS{}".to_string(),
        QosPreset::Services => "rclcpp::ServicesQoS{}".to_string(),
        QosPreset::SystemDefaults => "rclcpp::SystemDefaultsQoS{}".to_string(),
    }
}

fn qos_helper(
    h: &handlebars::Helper,
    _: &Handlebars,
    _: &handlebars::Context,
    _: &mut handlebars::RenderContext,
    out: &mut dyn handlebars::Output,
) -> Result<(), handlebars::RenderError> {
    let qos: Option<Qos> = serde_json::from_value(h.param(0).unwrap().value().clone())?;
    let rendered = match qos {
        Some(Qos::Profile(profile)) => qos_profile(profile),
        Some(Qos::Preset(preset)) => qos_preset(preset),
        _ => format!(
            r#"[history_policy, depth]() {{
      rclcpp::QoS qos{{depth}};
      qos.history(history_policy);
      return qos;
    }}()"#
        ),
    };
    out.write(&rendered)?;
    Ok(())
}

fn cpp_type_helper(
    h: &handlebars::Helper,
    _: &Handlebars,
    _: &handlebars::Context,
    _: &mut handlebars::RenderContext,
    out: &mut dyn handlebars::Output,
) -> Result<(), handlebars::RenderError> {
    let msg_type = h.param(0).unwrap().render();
    out.write(&msg_type.replace("/", "::"))?;
    Ok(())
}

#[derive(Serialize)]
struct Context<'a> {
    redf: &'a Redf,
    project_name: String,
    packages: Vec<String>,
    namespace: String,
    includes: Vec<String>,
}

pub fn generate(redf: &Redf, outdir: &Path) -> Result<(), Box<dyn Error>> {
    let mut hbs = Handlebars::new();
    register_helpers(&mut hbs);
    hbs.register_helper("cpp_type", Box::new(cpp_type_helper));
    hbs.register_helper("qos", Box::new(qos_helper));
    hbs.register_template_string("package.xml", include_str!("package.xml.tmpl"))?;
    hbs.register_template_string("CMakeLists.txt", include_str!("CMakeLists.txt.tmpl"))?;
    hbs.register_template_string("header", include_str!("header.hpp.tmpl"))?;

    let messages: BTreeSet<String> = redf
        .endpoints
        .iter()
        .map(|ep| match ep {
            Endpoint::Topic(ep) => ep.message_type.clone(),
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

    let ctx = Context {
        redf,
        project_name: redf.title.to_case(Case::Snake).clone(),
        packages,
        namespace,
        includes,
    };

    let f = std::fs::File::create(outdir.join("package.xml"))?;
    hbs.render_to_write("package.xml", &ctx, f)?;
    let f = std::fs::File::create(outdir.join("CMakeLists.txt"))?;
    hbs.render_to_write("CMakeLists.txt", &ctx, f)?;
    let f = std::fs::File::create(outdir.join(format!("{}.hpp", &ctx.project_name)))?;
    hbs.render_to_write("header", &ctx, f)?;

    Ok(())
}
