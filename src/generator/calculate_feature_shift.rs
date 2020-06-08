use rand::{RngCore, Rng};
use crate::generator::types::{VisibleWorld, Feature};
use nalgebra::{Vector3};
use crate::generator::calculate_prefabs_spawn_bounds::{calculate_prefabs_spawn_bounds, calculate_prefabs_centers_bounds};
use std::cmp::{max, min};

/// Randomizes a shift with which all the entities belonging to this feature will be spawned. This
/// function makes sure that after applying the shift no entity will be spawned outside visible
/// world bounds
///
/// * `rng` - random number generator
/// * `world` - visible world
/// * `feature` - feature that the possible shift is calculated for
///
pub fn calculate_feature_shift(rng: &mut impl RngCore, world: &VisibleWorld, feature: &Feature) -> Vector3<f32> {
    let mut shift = Vector3::new(0., 0., 0.);
    if feature.translate_x_using_bounds || feature.translate_z_using_bounds {
        let feature_spawn_bounds = calculate_prefabs_spawn_bounds(feature.prefabs.as_slice());
        if feature.translate_x_using_bounds {
            let mut min_x = (feature.translate_x_bounds.x.min(feature.translate_x_bounds.y) + feature_spawn_bounds.half_extents().x);
            let mut max_x = (feature.translate_x_bounds.x.max(feature.translate_x_bounds.y) - feature_spawn_bounds.half_extents().x);
            if min_x >= max_x {
                let half_way = (min_x + max_x) / 2.;
                min_x = half_way - 0.001;
                max_x = half_way + 0.001;
            }
            shift.x = rng.gen_range(
                min_x,
                max_x,
            );
        }
        if feature.translate_z_using_bounds {
            let mut min_z = (feature.translate_z_bounds.x.min(feature.translate_z_bounds.y) + feature_spawn_bounds.half_extents().z);
            let mut max_z = (feature.translate_z_bounds.x.max(feature.translate_z_bounds.y) - feature_spawn_bounds.half_extents().z);
            if min_z >= max_z {
                let half_way = (min_z + max_z) / 2.;
                min_z = half_way - 0.001;
                max_z = half_way + 0.001;
            }
            shift.z = rng.gen_range(
                min_z,
                max_z,
            );
        }
    }
    if (feature.translate_x && !feature.translate_x_using_bounds)
        || (feature.translate_z && !feature.translate_z_using_bounds) {
        let feature_spawn_bounds = calculate_prefabs_spawn_bounds(feature.prefabs.as_slice());
        if feature.translate_x && !feature.translate_x_using_bounds {
            let mut min_x = (world.world_bounds.mins().x + feature_spawn_bounds.half_extents().x);
            let mut max_x = (world.world_bounds.maxs().x - feature_spawn_bounds.half_extents().x);
            if min_x >= max_x {
                let half_way = (min_x + max_x) / 2.;
                min_x = half_way - 0.001;
                max_x = half_way + 0.001;
            }
            shift.x = rng.gen_range(
                min_x,
                max_x,
            );
        }
        if feature.translate_z && !feature.translate_z_using_bounds {
            let mut min_z = (world.world_bounds.mins().z + feature_spawn_bounds.half_extents().z);
            let mut max_z = (world.world_bounds.maxs().z - feature_spawn_bounds.half_extents().z);
            if min_z >= max_z {
                let half_way = (min_z + max_z) / 2.;
                min_z = half_way - 0.001;
                max_z = half_way + 0.001;
            }
            shift.z = rng.gen_range(
                min_z,
                max_z,
            );
        }
    }
    shift
}

#[cfg(test)]
mod tests {
    use nalgebra::{Vector3, Point3, Vector2, UnitQuaternion};
    use crate::{Prefab, Feature, VisibleWorld};
    use ncollide3d::bounding_volume::AABB;
    use crate::generator::calculate_feature_shift::calculate_feature_shift;
    use rand::thread_rng;
    use rand::rngs::mock::StepRng;


    struct RngTest(Vec<u64>);

    #[test]
    fn test_feature_using_bounds() {
        let prefab0 = Prefab {
            prefab_id: 0,
            position: Vector3::new(19.5, 0., 0.),
            rotation: UnitQuaternion::identity(),
            bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(48., 0.5, 0.5)),
            velocity: Vector3::new(0., -1., 0.),
        };
        let feature = Feature {
            translate_x: true,
            translate_x_using_bounds: true,
            translate_x_bounds: Vector2::new(10., 50.),
            translate_z: true,
            translate_z_using_bounds: false,
            translate_z_bounds: Vector2::new(0., 0.),
            prefabs: vec![prefab0],
            spawn_count: 1,
            spawns_per_second: 1.,
            trigger_time: 10.,
            priority: 0,
            missed_spawns: 0,
        };
        let world = VisibleWorld {
            world_bounds: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(30., 30., 30.)),
            travel_speed: 4.0,
        };
        // let mut step_rng = StepRng::new(1000, 100);
        let feature_shift = calculate_feature_shift(
            &mut thread_rng(),
            &world,
            &feature,
        );
        dbg!(feature_shift);
        // TODO Actually test instead of just printing
    }
}
