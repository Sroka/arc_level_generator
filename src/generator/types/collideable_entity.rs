use nalgebra::{Vector3, UnitQuaternion};
use crate::Prefab;

// Represents a spawned entity
#[derive(Clone, PartialEq, Debug)]
pub struct CollidableEntity {
    pub movement_start_parameter: f32,
    pub movement_end_parameter: f32,
    pub spawn_position: Vector3<f32>,
    pub spawn_rotation: UnitQuaternion<f32>,
    pub spawn_time: f32,
    pub prefab: Prefab,
    pub priority: i32,
}

impl CollidableEntity {
    pub fn position(&self, time: f32) -> Vector3<f32> {
        self.spawn_position + self.prefab.movement.baseline_velocity * (time - self.spawn_time)
    }
}
