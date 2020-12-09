use nalgebra::{Vector2, Vector3};

pub struct Movement {
    pub linear_velocity: Vector3<f32>,
    pub z_axis_tilt_xy_direction: Vector2<f32>,
    pub z_axis_tilt_angle: f32,
    pub z_axis_tilt_distance: f32,
    pub z_axis_tilt_easing_range: f32,
}