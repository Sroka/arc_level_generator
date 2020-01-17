use nalgebra::Vector3;
use ncollide3d::bounding_volume::AABB;
/// Represents single smallest piece of a generated level
#[derive(Clone, PartialEq, Debug)]
pub struct Prefab {
    pub prefab_id: i32,
    pub position: Vector3<f32>,
    pub bounding_box: AABB<f32>,
    pub velocity: Vector3<f32>,
}

