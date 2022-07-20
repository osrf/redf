use convert_case::{Case, Casing};
use std::collections::HashMap;
use tera::{Filter, Result, Tera, Test, Value};

struct Snake;

impl Filter for Snake {
    fn filter(&self, value: &Value, _: &HashMap<String, Value>) -> Result<Value> {
        Ok(Value::String(value.as_str().unwrap().to_case(Case::Snake)))
    }
}

struct Pascal;

impl Filter for Pascal {
    fn filter(&self, value: &Value, _: &HashMap<String, Value>) -> Result<Value> {
        Ok(Value::String(value.as_str().unwrap().to_case(Case::Pascal)))
    }
}

struct IsQosPreset;

impl Test for IsQosPreset {
    fn test(&self, value: Option<&Value>, _: &[Value]) -> Result<bool> {
        if let Some(v) = value {
            Ok(v.is_string())
        } else {
            Ok(false)
        }
    }
}

struct IsQosProfile;

impl Test for IsQosProfile {
    fn test(&self, value: Option<&Value>, _: &[Value]) -> Result<bool> {
        if let Some(v) = value {
            Ok(v.is_object())
        } else {
            Ok(false)
        }
    }
}

struct IsQosDefault;

impl Test for IsQosDefault {
    fn test(&self, value: Option<&Value>, _: &[Value]) -> Result<bool> {
        if let Some(v) = value {
            Ok(v.is_null())
        } else {
            Ok(true)
        }
    }
}

pub fn register_helpers(tera: &mut Tera) -> () {
    tera.register_filter("snake", Snake);
    tera.register_filter("pascal", Pascal);
    tera.register_tester("qos_preset", IsQosPreset);
    tera.register_tester("qos_profile", IsQosProfile);
    tera.register_tester("qos_default", IsQosDefault);
}
