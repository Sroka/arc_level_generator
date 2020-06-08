use crate::generator::types::prefab::Prefab;
use nalgebra::Vector2;

/// Represents a description of a feature that can be spawned in a generated world. Single feature
/// can consist of a multiple prefabs. The feature can only be spawned if all of its prefabs can be
/// spawned so that they won't collide with any of other already spawned entities
#[derive(Clone, PartialEq, Debug)]
pub struct Feature {
    pub prefabs: Vec<Prefab>,
    pub spawn_period: f32,
    pub is_spawn_period_strict: bool,
    pub spawn_count: i32,
    pub trigger_time: f32,
    pub priority: i32,
    pub translate_x: bool,
    pub translate_x_using_bounds: bool,
    pub translate_x_bounds: Vector2<f32>,
    pub translate_z: bool,
    pub translate_z_using_bounds: bool,
    pub translate_z_bounds: Vector2<f32>,
    pub missed_spawns: i32,
    pub last_spawn_attempt: f32,
}
