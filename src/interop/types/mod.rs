use nalgebra::{Vector3, Vector2};

#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct PrefabDescription {
    pub prefab_id: i32,
    pub position: Vector3<f32>,
    pub velocity: Vector3<f32>,
    pub half_extents: Vector3<f32>,
}

#[derive(Debug, Clone)]
#[repr(C)]
pub struct FeatureDescription {
    pub prefabs: *const PrefabDescription,
    pub prefabs_count: i32,
    pub spawns_per_second: f32,
    pub spawn_count: i32,
    pub trigger_position: f32,
    pub priority: i32,
    pub translate_x: bool,
    pub translate_x_using_bounds: bool,
    pub translate_x_bounds: Vector2<f32>,
    pub translate_z: bool,
    pub translate_z_using_bounds: bool,
    pub translate_z_bounds: Vector2<f32>,
}

#[derive(Debug, Clone)]
#[repr(C)]
pub struct VisibleWorldDescription {
    pub position: Vector3<f32>,
    pub half_extents: Vector3<f32>,
    pub travel_speed: f32,
}

#[derive(Debug, Clone)]
#[repr(C)]
pub struct EntityDescription {
    pub prefab_id: i32,
    pub spawn_position: Vector3<f32>,
    pub spawn_time: f32,
    pub velocity: Vector3<f32>,
}

#[repr(C)]
pub struct EntitiesArrayDescription {
    pub pointer: *mut EntityDescription,
    pub length: i32,
}
