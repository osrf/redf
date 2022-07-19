use serde::{Deserialize, Serialize};

#[cfg(feature = "json_schema")]
use schemars::JsonSchema;

#[derive(Serialize, Deserialize)]
#[cfg_attr(feature = "json_schema", derive(JsonSchema))]
pub struct Duration {
    pub secs: i32,
    pub nanosecs: u32,
}

#[derive(Serialize, Deserialize)]
#[cfg_attr(feature = "json_schema", derive(JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum ReliabilityPolicy {
    BestEffort,
    Reliable,
    SystemDefault,
}

#[derive(Serialize, Deserialize)]
#[cfg_attr(feature = "json_schema", derive(JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum DurabilityPolicy {
    Volatile,
    TransientLocal,
    SystemDefault,
}

#[derive(Serialize, Deserialize)]
#[cfg_attr(feature = "json_schema", derive(JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum LivelinessPolicy {
    Automatic,
    ManualByTopic,
    SystemDefault,
}

#[derive(Serialize, Deserialize)]
#[cfg_attr(feature = "json_schema", derive(JsonSchema))]
pub struct QosProfile {
    pub reliability: Option<ReliabilityPolicy>,
    pub durability: Option<DurabilityPolicy>,
    pub liveliness: Option<LivelinessPolicy>,
    pub deadline: Option<Duration>,
    pub lifespan: Option<Duration>,
    pub lease_duration: Option<Duration>,
    pub avoid_ros_namespace_conventions: Option<bool>,
}

#[derive(Serialize, Deserialize)]
#[cfg_attr(feature = "json_schema", derive(JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum QosPreset {
    Clock,
    SensorData,
    Parameters,
    Services,
    ParameterEvents,
    Rosout,
    SystemDefaults,
}

#[derive(Serialize, Deserialize)]
#[cfg_attr(feature = "json_schema", derive(JsonSchema))]
#[serde(untagged)]
pub enum Qos {
    Profile(QosProfile),
    Preset(QosPreset),
}

#[derive(Serialize, Deserialize)]
#[cfg_attr(feature = "json_schema", derive(JsonSchema))]
pub struct TopicEndpoint {
    pub title: String,
    pub topic: String,
    pub message_type: String,
    pub description: String,
    pub qos: Option<Qos>,
}

#[derive(Serialize, Deserialize)]
#[cfg_attr(feature = "json_schema", derive(JsonSchema))]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum Endpoint {
    Topic(TopicEndpoint),
}

#[derive(Serialize, Deserialize)]
#[cfg_attr(feature = "json_schema", derive(JsonSchema))]
pub struct Maintainer {
    pub name: String,
    pub email: String,
}

#[derive(Serialize, Deserialize)]
#[cfg_attr(feature = "json_schema", derive(JsonSchema))]
pub struct Redf {
    pub title: String,
    pub description: String,
    pub version: String,
    pub maintainers: Vec<Maintainer>,
    pub license: String,
    pub endpoints: Vec<Endpoint>,
    /// if not provided, the namespace will be derived from the title
    pub cpp_namespace: Option<String>,
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_deserialize() -> Result<(), Box<dyn std::error::Error>> {
        let f = std::fs::File::open("tests/test_api.yaml")?;
        serde_yaml::from_reader::<_, Redf>(f)?;
        Ok(())
    }
}
