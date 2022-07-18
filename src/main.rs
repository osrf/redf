mod cpp;
mod redf;

use clap::{ArgEnum, Parser};

#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord, ArgEnum)]
enum Generator {
    Cpp,
}

#[derive(Parser)]
pub struct Args {
    #[clap(help = "Input redf file")]
    input: String,

    #[clap(help = "Output path", short, long)]
    out: String,

    #[clap(help = "Generator to use", short, long, arg_enum)]
    gen: Generator,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();
    let redf = serde_yaml::from_reader(std::fs::File::open(&args.input)?)?;
    std::fs::create_dir_all(&args.out)?;
    match args.gen {
        Generator::Cpp => cpp::codegen(args, redf)?,
    }
    Ok(())
}
