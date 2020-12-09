use nalgebra::{Vector3, UnitQuaternion};
use ncollide3d::bounding_volume::AABB;
use crate::generator::types::movement::Movement;

/// Represents single smallest piece of a generated level
#[derive(Clone, PartialEq, Debug)]
pub struct Prefab {
    pub prefab_id: i32,
    pub position: Vector3<f32>,
    pub rotation: UnitQuaternion<f32>,
    pub bounding_box: AABB<f32>,
    pub movement: Movement,
}

