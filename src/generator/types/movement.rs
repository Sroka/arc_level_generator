use nalgebra::{Vector3, Unit};

#[derive(Clone, PartialEq, Debug)]
pub struct Movement {
    pub baseline_velocity: Vector3<f32>,
    pub arcs_plane_normal: Unit<Vector3<f32>>,
    pub approach_arc_angle: f32,
    pub approach_arc_center_distance: f32,
    pub approach_arc_radius: f32,
    pub approach_rotation_strength: f32,
    pub departure_arc_angle: f32,
    pub departure_arc_center_distance: f32,
    pub departure_arc_radius: f32,
    pub departure_rotation_strength: f32,
}
