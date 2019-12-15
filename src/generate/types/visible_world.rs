use ncollide3d::bounding_volume::AABB;

pub struct VisibleWorld {
    pub world_bounds: AABB<f32>,
    pub travel_speed: f32,
    pub spawn_barrier: f32,
}


