use crate::generator::types::prefab::Prefab;

#[derive(Clone, PartialEq, Debug)]
pub struct Feature {
    pub prefabs: Vec<Prefab>,
    pub spawns_per_second: f32,
    pub spawn_count: i32,
    pub trigger_position: f32,
    pub priority: i32,
    pub translate_x: bool,
    pub translate_z: bool,
    pub missed_spawns: i32,
}

impl Feature {
    pub fn new(
        prefabs: Vec<Prefab>,
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
