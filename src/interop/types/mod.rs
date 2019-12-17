use nalgebra::Vector3;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PrefabDescription {
    pub prefab_id: i32,
    pub position: Vector3<f32>,
    pub velocity: Vector3<f32>,
    pub half_extents: Vector3<f32>,
}

#[derive(Debug, Clone)]
pub struct FeatureDescription {
    pub translate_x: bool,
    pub translate_z: bool,
    pub prefabs_ids: *const i32,
    pub prefabs_ids_count: i32,
    pub spawn_count: i32,
    pub spawn_start_distance: f32,
}

#[derive(Debug, Clone)]
pub struct VisibleWorldDescription {
    pub position: Vector3<f32>,
    pub half_extents: Vector3<f32>,
    pub travel_speed: f32,
    pub spawn_barrier_y_coord: f32,
}

pub struct EntityDescription {}


pub struct EntitiesArrayDescription {
    pub pointer: *const EntityDescription,
    pub length: i32,
}
