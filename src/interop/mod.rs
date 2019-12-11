use nalgebra::{Vector3, Vector2};

pub struct PrefabDescription {
    prefab_id: i32,
    position: Vector3<f32>,
    bounding_box: Vector3<f32>,
    velocity: Vector3<f32>,
}

pub struct FeatureDescription {
    translate_x: bool,
    translate_z: bool,
    prefabs_ids: *const i32,
    prefabs_ids_count: i32,
    spawn_count: i32,
    spawn_start_distance: f32,
}

pub struct VisibleWorldDescription {
    bounds: Vector3<f32>,
    travel_speed: f32,
    spawn_distance: f32,
}
