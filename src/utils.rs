use crate::redf::{Endpoint, Redf};
use std::collections::BTreeSet;

/// Get a set of unique ros types used in the endpoints.
pub fn get_ros_types(redf: &Redf) -> BTreeSet<String> {
    redf.endpoints
        .iter()
        .map(|ep| match ep {
            Endpoint::Topic(ep) => ep.message_type.clone(),
            Endpoint::Service(ep) => ep.service_type.clone(),
        })
        .collect()
}
