use ncollide3d::bounding_volume::AABB;

#[derive(Debug)]
pub struct VisibleWorld {
    pub world_bounds: AABB<f32>,
}


