/// A color for debug-rendering.
///
/// The default colors are provided in HSLA (Hue Saturation Lightness Alpha) format.
pub type DebugColor = [f32; 4];

#[derive(Clone, Debug, PartialEq)]
pub struct DebugRenderStyle {
    pub subdivisions: u32,
    pub collider_dynamic_color: DebugColor,
    pub collider_fixed_color: DebugColor,
    pub collider_kinematic_color: DebugColor,
    pub collider_parentless_color: DebugColor,
    pub impulse_joint_anchor_color: DebugColor,
    pub impulse_joint_separation_color: DebugColor,
    pub multibody_joint_anchor_color: DebugColor,
    pub multibody_joint_separation_color: DebugColor,
    pub sleep_color_multiplier: [f32; 4],
    pub rigid_body_axes_length: f32,
}

impl Default for DebugRenderStyle {
    fn default() -> Self {
        Self {
            subdivisions: 20,
            collider_dynamic_color: [340.0, 1.0, 0.3, 1.0],
            collider_kinematic_color: [20.0, 1.0, 0.3, 1.0],
            collider_fixed_color: [30.0, 1.0, 0.4, 1.0],
            collider_parentless_color: [30.0, 1.0, 0.4, 1.0],
            impulse_joint_anchor_color: [240.0, 0.5, 0.4, 1.0],
            impulse_joint_separation_color: [0.0, 0.5, 0.4, 1.0],
            multibody_joint_anchor_color: [300.0, 1.0, 0.4, 1.0],
            multibody_joint_separation_color: [0.0, 1.0, 0.4, 1.0],
            sleep_color_multiplier: [1.0, 1.0, 0.2, 1.0],
            rigid_body_axes_length: 0.5,
        }
    }
}
