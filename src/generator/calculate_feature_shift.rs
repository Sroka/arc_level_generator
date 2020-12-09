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
    let mut shift = Vector3::new(0., 0., feature.translate_z);
    if feature.translate_x_using_bounds || feature.translate_y_using_bounds {
        let feature_spawn_bounds = calculate_prefabs_spawn_bounds(feature.prefabs.as_slice());
        if feature.translate_x_using_bounds {
            let mut min_x = (feature.translate_x_bounds.x.min(feature.translate_x_bounds.y) - feature_spawn_bounds.center().x + feature_spawn_bounds.half_extents().x);
            let mut max_x = (feature.translate_x_bounds.x.max(feature.translate_x_bounds.y) - feature_spawn_bounds.center().x - feature_spawn_bounds.half_extents().x);
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
        if feature.translate_y_using_bounds {
            let mut min_y = (feature.translate_y_bounds.x.min(feature.translate_y_bounds.y) - feature_spawn_bounds.center().y + feature_spawn_bounds.half_extents().y);
            let mut max_y = (feature.translate_y_bounds.x.max(feature.translate_y_bounds.y) - feature_spawn_bounds.center().y - feature_spawn_bounds.half_extents().y);
            if min_y >= max_y {
                let half_way = (min_y + max_y) / 2.;
                min_y = half_way - 0.001;
                max_y = half_way + 0.001;
            }
            shift.y = rng.gen_range(
                min_y,
                max_y,
            );
        }
    }
    if (feature.translate_x && !feature.translate_x_using_bounds)
        || (feature.translate_y && !feature.translate_y_using_bounds) {
        let feature_spawn_bounds = calculate_prefabs_spawn_bounds(feature.prefabs.as_slice());
        if feature.translate_x && !feature.translate_x_using_bounds {
            let mut min_x = (world.world_bounds.mins().x - feature_spawn_bounds.center().x + feature_spawn_bounds.half_extents().x);
            let mut max_x = (world.world_bounds.maxs().x - feature_spawn_bounds.center().x - feature_spawn_bounds.half_extents().x);
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
        if feature.translate_y && !feature.translate_y_using_bounds {
            let mut min_y = (world.world_bounds.mins().y - feature_spawn_bounds.center().y + feature_spawn_bounds.half_extents().y);
            let mut max_y = (world.world_bounds.maxs().y - feature_spawn_bounds.center().y - feature_spawn_bounds.half_extents().y);
            if min_y >= max_y {
                let half_way = (min_y + max_y) / 2.;
                min_y = half_way - 0.001;
                max_y = half_way + 0.001;
            }
            shift.y = rng.gen_range(
                min_y,
                max_y,
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
    use crate::generator::Movement;


    struct RngTest(Vec<u64>);

    #[test]
    fn test_feature_using_bounds() {
        let prefab0 = Prefab {
            prefab_id: 0,
            position: Vector3::new(19.5, 0., 0.),
            rotation: UnitQuaternion::identity(),
            bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(48., 0.5, 0.5)),
            movement: Movement {
                linear_velocity: Vector3::new(0., -1., 0.),
                z_axis_tilt_xy_direction: nalgebra::zero(),
                z_axis_tilt_angle: 0.0,
                z_axis_tilt_distance: 0.0,
                z_axis_tilt_easing_range: 0.0
            },
        };
        let feature = Feature {
            translate_x: true,
            translate_x_using_bounds: true,
            translate_x_bounds: Vector2::new(10., 50.),
            translate_y: true,
            translate_y_using_bounds: false,
            translate_y_bounds: Vector2::new(0., 0.),
            prefabs: vec![prefab0],
            spawn_count: 1,
            spawn_period: 1.,
            trigger_time: 10.,
            priority: 0,
            missed_spawns: 0,
            is_spawn_period_strict: false,
            last_spawn_attempt: 0.0,
            translate_z: 0.0
        };
        let world = VisibleWorld {
            world_bounds: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(30., 30., 30.)),
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
