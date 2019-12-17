use ncollide3d::bounding_volume::AABB;
use nalgebra::{Vector3, Point3};
use crate::generate::types::prefab::Prefab;

#[derive(Clone, PartialEq, Debug)]
pub struct Feature<'a> {
    pub prefabs: &'a [Prefab],
    pub spawns_per_second: f32,
    pub spawn_count: i32,
    pub trigger_position: f32,
    pub priority: i32,
    pub translate_x: bool,
    pub translate_z: bool,
    pub missed_spawns: i32,
}

impl<'a> Feature<'a> {
    pub fn new(
        prefabs: &'a [Prefab],
        spawns_per_second: f32,
        spawn_count: i32,
        trigger_position: f32,
        priority: i32,
        translate_x: bool,
        translate_z: bool,
    ) -> Self {
        Feature {
            prefabs,
            spawns_per_second,
            spawn_count,
            trigger_position,
            priority,
            translate_x,
            translate_z,
            missed_spawns: 0,
        }
    }
}
