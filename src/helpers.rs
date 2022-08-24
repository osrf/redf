use convert_case::{Case, Casing};
use regex::Regex;
use serde_json::Map;
use std::collections::HashMap;
use tera::{Filter, Function, Result, Tera, Test, Value};

struct Snake;

impl Filter for Snake {
    fn filter(&self, value: &Value, _: &HashMap<String, Value>) -> Result<Value> {
        Ok(Value::String(value.as_str().unwrap().to_case(Case::Snake)))
    }
}

struct ScreamingSnake;

impl Filter for ScreamingSnake {
    fn filter(&self, value: &Value, _: &HashMap<String, Value>) -> Result<Value> {
        Ok(Value::String(value.as_str().unwrap().to_case(Case::ScreamingSnake)))
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

struct VarSub;

/// Destructure a string with the format `{var}` into parts. Each part is a map which contains
/// they key `var` if it corresponds to a subsituted variable, else it contains the key `raw`
/// if that part is not subsituted.
///
/// tera args:
///   str: the string to destructure
///   vars_only: if true, only return the variable names, this will skip all `raw` items.
impl Function for VarSub {
    fn call(&self, args: &HashMap<String, Value>) -> Result<Value> {
        let v = args
            .get("str")
            .ok_or("Missing parameter 'str'")?
            .as_str()
            .ok_or("Expected a string")?;
        let vars_only = match args.get("vars_only") {
            Some(v) => v.as_bool().ok_or("Expected vars_only to be a boolean")?,
            None => false,
        };
        let re = Regex::new("\\{(\\w+)\\}").unwrap();

        if vars_only {
            let mut parts: Vec<Value> = Vec::new();
            re.captures_iter(v).for_each(|m| {
                parts.push(Value::Object(Map::from_iter([(
                    "var".to_string(),
                    Value::String(m.get(1).unwrap().as_str().to_string()),
                )])));
            });
            return Ok(Value::Array(parts));
        }

        let mut parts: Vec<Value> = Vec::new();
        let mut cur = 0;
        re.captures_iter(v).for_each(|m| {
            let raw = v[cur..m.get(0).unwrap().start()].to_string();
            if raw.len() > 0 {
                parts.push(Value::Object(Map::from_iter([(
                    "raw".to_string(),
                    Value::String(raw),
                )])));
            }
            let var = m.get(1).unwrap().as_str().to_string();
            parts.push(Value::Object(Map::from_iter([(
                "var".to_string(),
                Value::String(var),
            )])));
            cur = m.get(0).unwrap().end();
        });
        let last = v[cur..].to_string();
        if last.len() > 0 {
            parts.push(Value::Object(Map::from_iter([(
                "raw".to_string(),
                Value::String(last),
            )])));
        }
        return Ok(Value::Array(parts));
    }
}

pub fn register_helpers(tera: &mut Tera) -> () {
    tera.register_filter("snake", Snake);
    tera.register_filter("screaming_snake", ScreamingSnake);
    tera.register_filter("pascal", Pascal);
    tera.register_tester("qos_preset", IsQosPreset);
    tera.register_tester("qos_profile", IsQosProfile);
    tera.register_tester("qos_default", IsQosDefault);
    tera.register_function("varsub", VarSub);
}
