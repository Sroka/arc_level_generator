use nalgebra::Vector3;
use super::types;

pub struct CollideableEntity {
    pub spawn_position: Vector3<f32>,
    pub spawn_time: f32,
    pub velocity: Vector3<f32>,
    pub bounding_box: Vector3<f32>,
    pub prefab_id: i32,
}

#[derive(Clone)]
pub struct Feature<'a> {
    pub translate_x: bool,
    pub translate_z: bool,
    pub prefabs: &'a [Prefab],
    pub spawns_per_second: f32,
    pub spawn_count: i32,
    pub trigger_position: f32,
    pub priority: f32,

}

pub struct Prefab {
    pub prefab_id: i32,
    pub position: Vector3<f32>,
    pub bounding_box: Vector3<f32>,
    pub velocity: Vector3<f32>,
}

pub struct VisibleWorld {
    pub half_extents: Vector3<f32>,
    pub travel_speed: f32,
    pub spawn_barrier: f32,
}

