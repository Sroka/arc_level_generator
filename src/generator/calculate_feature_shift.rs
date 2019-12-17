use rand::{RngCore, Rng};
use crate::generator::types::{VisibleWorld, Feature, calculate_prefabs_spawn_bounds};
use nalgebra::Vector3;

pub fn calculate_feature_shift(rng: &mut impl RngCore, world: &VisibleWorld, feature: &Feature) -> Vector3<f32> {
    let mut shift = Vector3::new(0., 0., 0.);
    if feature.translate_x || feature.translate_z {
        let feature_spawn_bounds = calculate_prefabs_spawn_bounds(feature.prefabs);
        if feature.translate_x {
            shift.x = rng.gen_range(world.world_bounds.mins().x + feature_spawn_bounds.half_extents().x, world.world_bounds.maxs().x - feature_spawn_bounds.half_extents().x);
        }
        if feature.translate_z {
            shift.z = rng.gen_range(world.world_bounds.mins().z + feature_spawn_bounds.half_extents().z, world.world_bounds.maxs().z - feature_spawn_bounds.half_extents().z);
        }
    }
    shift
}
