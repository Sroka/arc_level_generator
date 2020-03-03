use crate::generator::types::prefab::Prefab;

/// Represents a description of a feature that can be spawned in a generated world. Single feature
/// can consist of a multiple prefabs. The feature can only be spawned if all of its prefabs can be
/// spawned so that they won't collide with any of other already spawned entities
#[derive(Clone, PartialEq, Debug)]
pub struct Feature {
    pub prefabs: Vec<Prefab>,
    pub spawns_per_second: f32,
    pub spawn_count: i32,
    pub trigger_position: f32,
    pub priority: i32,
    pub translate_x: bool,
    pub translate_x_out_of_bounds: bool,
    pub translate_z: bool,
    pub translate_z_out_of_bounds: bool,
    pub missed_spawns: i32,
}
