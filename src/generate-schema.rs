mod redf;

use redf::Redf;
use schemars::schema_for;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let schema = schema_for!(Redf);
    let f = std::fs::OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open("redf.schema.json")
        .unwrap();
    serde_json::to_writer_pretty(f, &schema).unwrap();
    Ok(())
}

#[cfg(test)]
mod test {
    use super::*;
    use std::iter::zip;

    #[test]
    fn check_schema_changes() -> Result<(), String> {
        let cur_schema_json = std::fs::read("redf.schema.json").unwrap();
        let schema = schema_for!(Redf);
        let new_schema_json = serde_json::to_vec_pretty(&schema).unwrap();

        if cur_schema_json.len() != new_schema_json.len()
            || zip(cur_schema_json, new_schema_json).any(|(a, b)| a != b)
        {
            return Err(String::from("There are changes in the json schema, please run `cargo run -F json_schema --bin generate-schema` to regenerate it"));
        }
        Ok(())
    }
}
