mod redf;

use redf::Redf;
use schemars::schema_for;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let schema = schema_for!(Redf);
    std::fs::write("redf.schema.json", serde_json::to_string_pretty(&schema)?)?;
    Ok(())
}
