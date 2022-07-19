use convert_case::{Case, Casing};
use handlebars::{Context, Handlebars, Helper, Output, RenderContext, RenderError};

fn snake(
    h: &Helper,
    _: &Handlebars,
    _: &Context,
    _: &mut RenderContext,
    out: &mut dyn Output,
) -> Result<(), RenderError> {
    out.write(&h.param(0).unwrap().render().to_case(Case::Snake))?;
    Ok(())
}

fn pascal(
    h: &Helper,
    _: &Handlebars,
    _: &Context,
    _: &mut RenderContext,
    out: &mut dyn Output,
) -> Result<(), RenderError> {
    out.write(&h.param(0).unwrap().render().to_case(Case::Pascal))?;
    Ok(())
}

pub fn register_helpers(hbs: &mut Handlebars) -> () {
    hbs.register_helper("snake", Box::new(snake));
    hbs.register_helper("pascal", Box::new(pascal));
}
