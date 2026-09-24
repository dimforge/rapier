//! Export naming for the dimension-independent Rapier C implementation.
use proc_macro::{TokenStream, TokenTree};

fn pascal_case(name: &str) -> Result<String, &'static str> {
    let mut result = String::new();
    for word in name.split('_') {
        if word.is_empty()
            || !word
                .bytes()
                .all(|c| c.is_ascii_lowercase() || c.is_ascii_digit())
        {
            return Err("expected a nonempty snake_case function name after rpr_");
        }
        let mut chars = word.chars();
        result.push(chars.next().unwrap().to_ascii_uppercase());
        result.extend(chars);
    }
    Ok(result)
}

fn name_suffix(name: &str, receiver: Option<&str>) -> Result<String, &'static str> {
    let suffix = name
        .strip_prefix("rpr_")
        .ok_or("expected an rpr_ function name")?;
    match receiver {
        Some(receiver) => {
            let method = suffix
                .strip_prefix(receiver)
                .and_then(|method| method.strip_prefix('_'))
                .ok_or(
                    "method name must start with the receiver prefix followed by an underscore",
                )?;
            Ok(format!(
                "{}_{}",
                pascal_case(receiver)?,
                pascal_case(method)?
            ))
        }
        None => pascal_case(suffix),
    }
}

/// Export a dimension-specific C symbol, keeping the Rust identifier unchanged.
///
/// `#[rapier_export]` exports `rpr_new_world` as `r2NewWorld` / `r3NewWorld`.
/// Instance methods specify their receiver prefix: `#[rapier_export(rigid_body)]`
/// exports `rpr_rigid_body_position` as `r2RigidBody_Position` / `r3RigidBody_Position`.
/// Constructors, destructors, static helpers, and short world functions omit the receiver.
#[proc_macro_attribute]
pub fn rapier_export(args: TokenStream, item: TokenStream) -> TokenStream {
    fn expand(args: TokenStream, item: TokenStream) -> Result<TokenStream, &'static str> {
        let mut args = args.into_iter();
        let receiver = match (args.next(), args.next()) {
            (None, None) => None,
            (Some(TokenTree::Ident(receiver)), None) => Some(receiver.to_string()),
            _ => return Err("expected no arguments or one snake_case receiver prefix"),
        };
        let mut tokens = item.clone().into_iter();
        // Groups are opaque here, so a `fn` token in a body or attribute cannot match.
        let found =
            tokens.any(|token| matches!(token, TokenTree::Ident(id) if id.to_string() == "fn"));
        if !found {
            return Err("rapier_export requires a function");
        }
        let Some(TokenTree::Ident(name)) = tokens.next() else {
            return Err("expected a function name");
        };
        let suffix = name_suffix(&name.to_string(), receiver.as_deref())?;
        let attributes = format!(
            r#"#[cfg_attr(feature = "dim2", unsafe(export_name = "r2{suffix}"))]
               #[cfg_attr(feature = "dim3", unsafe(export_name = "r3{suffix}"))]"#
        );
        let mut output: TokenStream = attributes.parse().unwrap();
        output.extend(item);
        Ok(output)
    }
    expand(args, item)
        .unwrap_or_else(|message| format!("compile_error!({message:?});").parse().unwrap())
}

#[cfg(test)]
mod tests {
    use super::name_suffix;

    #[test]
    fn lifecycle_functions_and_helpers_use_natural_word_order() {
        for (rust, c) in [
            ("rpr_new_world", "NewWorld"),
            ("rpr_free_world", "FreeWorld"),
            ("rpr_cuboid_collider_desc", "CuboidColliderDesc"),
            ("rpr_default_query_options", "DefaultQueryOptions"),
            ("rpr_urdf_robot_from_file", "UrdfRobotFromFile"),
            ("rpr_dynamic_rigid_body_desc", "DynamicRigidBodyDesc"),
            ("rpr_ball_shared_shape", "BallSharedShape"),
            ("rpr_step", "Step"),
            ("rpr_insert_rigid_body", "InsertRigidBody"),
            ("rpr_check_abi", "CheckAbi"),
            ("rpr_matrix_3x3", "Matrix3x3"),
        ] {
            assert_eq!(name_suffix(rust, None), Ok(c.into()));
        }
    }

    #[test]
    fn instance_methods_separate_the_receiver_from_the_method() {
        for (rust, receiver, c) in [
            (
                "rpr_rigid_body_is_ccd_enabled",
                "rigid_body",
                "RigidBody_IsCcdEnabled",
            ),
            (
                "rpr_joint_desc_set_local_frame1",
                "joint_desc",
                "JointDesc_SetLocalFrame1",
            ),
            (
                "rpr_soft_body_tear_event_bodies",
                "soft_body_tear_event",
                "SoftBodyTearEvent_Bodies",
            ),
            (
                "rpr_read_rigid_body_position",
                "read_rigid_body",
                "ReadRigidBody_Position",
            ),
        ] {
            assert_eq!(name_suffix(rust, Some(receiver)), Ok(c.into()));
        }
    }

    #[test]
    fn reject_names_outside_the_export_convention() {
        for name in [
            "physics_world_new",
            "rpr_",
            "rpr__new",
            "rpr_world_",
            "rpr_World",
        ] {
            assert!(name_suffix(name, None).is_err(), "{name}");
        }
        for (name, receiver) in [
            ("rpr_free_world", "rigid_body"),
            ("rpr_free_world", "wor"),
            ("rpr_free_world", ""),
            ("rpr_free_world", "World"),
            ("rpr_world_", "world"),
            ("rpr_world", "world"),
        ] {
            assert!(
                name_suffix(name, Some(receiver)).is_err(),
                "{name}: {receiver}"
            );
        }
    }
}
