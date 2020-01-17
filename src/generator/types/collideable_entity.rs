use nalgebra::Vector3;
use ncollide3d::bounding_volume::AABB;

// Represents a spawned entity
#[derive(Clone, PartialEq, Debug)]
pub struct CollideableEntity {
    pub spawn_position: Vector3<f32>,
    pub spawn_time: f32,
    pub velocity: Vector3<f32>,
    pub bounding_box: AABB<f32>,
    pub prefab_id: i32,
    pub priority: i32,
}

impl CollideableEntity {
    pub fn position(&self, time: f32) -> Vector3<f32> {
        self.spawn_position + self.velocity * (time - self.spawn_time)
    }
}
