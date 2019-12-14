use nalgebra::Vector3;
use ncollide3d::bounding_volume::AABB;

#[derive(Clone, PartialEq, Debug)]
pub struct CollideableEntity {
    pub spawn_position: Vector3<f32>,
    pub spawn_time: f32,
    pub velocity: Vector3<f32>,
    pub bounding_box: AABB<f32>,
    pub prefab_id: i32,
    pub priority: i32,
}

#[derive(Clone, PartialEq, Debug)]
pub struct Feature<'a> {
    pub translate_x: bool,
    pub translate_z: bool,
    pub prefabs: &'a [Prefab],
    pub spawns_per_second: f32,
    pub spawn_count: i32,
    pub trigger_position: f32,
    pub priority: f32,

}

#[derive(PartialEq, Debug)]
pub struct Prefab {
    pub prefab_id: i32,
    pub position: Vector3<f32>,
    pub bounding_box: AABB<f32>,
    pub velocity: Vector3<f32>,
}

pub struct VisibleWorld {
    pub world_bounds: AABB<f32>,
    pub travel_speed: f32,
    pub spawn_barrier: f32,
}


impl CollideableEntity {
    pub fn position(&self, time: f32) -> Vector3<f32> {
        self.spawn_position + self.velocity * (time - self.spawn_time)
    }
}
