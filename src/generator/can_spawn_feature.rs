use super::types::{CollideableEntity, VisibleWorld};
use std::collections::VecDeque;
use ncollide3d::shape::Cuboid;
use ncollide3d::query;
use ncollide3d::query::{RayCast, Ray};
use nalgebra::{Isometry3, Vector3, Point3, Translation3};
use crate::generator::types::Feature;
use ncollide3d::bounding_volume::BoundingVolume;
use std::cmp::Ordering::Equal;
use itertools::Itertools;
use std::iter;
use float_cmp::{ApproxEq, F32Margin};

/// Checks if a feature can be safely spawn so that it won't collide with any existing entities in
/// a visible world
///
pub fn can_spawn_feature(
    feature: &Feature,
    obstacles: &VecDeque<CollideableEntity>,
    world: &VisibleWorld,
    time_travelled: f32,
    feature_shift: &Vector3<f32>,
) -> bool {
    let min_z_velocity_in_a_feature = feature.prefabs
        .iter()
        .map(|prefab| prefab.velocity.z)
        .sorted_by(|a, b| { a.partial_cmp(b).unwrap_or(Equal) })
        .last()
        .unwrap();
    let time_to_travel_to_origin_plane_from_worlds_start = world.world_bounds.maxs().z / -min_z_velocity_in_a_feature;
    'prefabs_loop: for prefab in &feature.prefabs {
        'obstacles_loop: for obstacle in obstacles {
            let prefab_spawn_position = prefab.position
                - prefab.velocity * time_to_travel_to_origin_plane_from_worlds_start
                - prefab.velocity * (feature.priority as f32)
                + feature_shift;
            let obstacle_spawn_position = obstacle.spawn_position
                + obstacle.velocity * (time_travelled - obstacle.spawn_time);

            let prefab_spawn_position_isometry = Isometry3::from_parts(Translation3::from(prefab_spawn_position), prefab.rotation);
            let prefab_world_bounds_toi = world.world_bounds
                // This is required because sometimes, due to priority, prefab spawn position is outside world bounds
                .merged(&prefab.bounding_box.transform_by(&prefab_spawn_position_isometry))
                .toi_with_ray(
                    &Isometry3::new(nalgebra::zero(), nalgebra::zero()),
                    &Ray::new(Point3::origin() + prefab_spawn_position, prefab.velocity),
                    false,
                ).unwrap()
                +
                prefab.bounding_box
                    .toi_with_ray(
                        &Isometry3::from_parts(Translation3::identity(), prefab.rotation),
                        &Ray::new(Point3::origin(), -prefab.velocity),
                        false,
                    ).unwrap();

            let obstacle_spawn_position_isometry = Isometry3::from_parts(Translation3::from(obstacle_spawn_position), obstacle.rotation);
            // TODO In reality this is a fix for a bug in query::time_of_impact
            let distance_at_spawn_point: f32 = query::distance_support_map_support_map(
                &prefab_spawn_position_isometry,
                &Cuboid::new(prefab.bounding_box.half_extents()),
                &obstacle_spawn_position_isometry,
                &Cuboid::new(obstacle.bounding_box.half_extents()),
            );
            if distance_at_spawn_point.approx_eq(0., F32Margin { epsilon: 0.01, ulps: 2 }) {
                return false;
            }

            let time_of_impact = query::time_of_impact(
                &prefab_spawn_position_isometry,
                &prefab.velocity,
                &Cuboid::new(prefab.bounding_box.half_extents()),
                &obstacle_spawn_position_isometry,
                &obstacle.velocity,
                &Cuboid::new(obstacle.bounding_box.half_extents()),
                prefab_world_bounds_toi,
                0.0,
            );
            match time_of_impact {
                Some(toi) => {
                    return false;
                }
                _ => {}
            }
        }
    }
    return true;
}

#[cfg(test)]
mod tests {
    use crate::generator::types::{Prefab, Feature, VisibleWorld, CollideableEntity};
    use ncollide3d::bounding_volume::AABB;
    use nalgebra::{Vector3, Point3, Vector2, Isometry3, Translation3};
    use crate::generator::can_spawn_feature::can_spawn_feature;
    use std::collections::VecDeque;
    use std::iter::FromIterator;

    mod zero_travel_time {
        use super::*;
        use nalgebra::UnitQuaternion;

        #[test]
        fn test_can_spawn_feature_collision() {
            let prefab = Prefab {
                prefab_id: 0,
                position: Vector3::new(0., 0., 0.),
                rotation: UnitQuaternion::identity(),
                bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
                velocity: Vector3::new(0., 0., -1.),
            };
            let feature = Feature {
                translate_x: false,
                translate_x_using_bounds: false,
                translate_x_bounds: Vector2::new(0., 0.),
                translate_y: false,
                translate_y_using_bounds: false,
                translate_y_bounds: Vector2::new(0., 0.),
                prefabs: vec![prefab],
                spawn_count: 1,
                spawn_period: 1.,
                trigger_time: 10.,
                priority: 0,
                missed_spawns: 0,
                is_spawn_period_strict: false,
                last_spawn_attempt: 0.0,
            };
            let obstacle = CollideableEntity {
                spawn_position: Vector3::new(0., 5., 10.),
                prefab_id: 0,
                bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
                rotation: UnitQuaternion::identity(),
                velocity: Vector3::new(0., -1., -1.),
                spawn_time: 0.0,
                priority: 0,
            };
            let world = VisibleWorld {
                world_bounds: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(10., 10., 10.)),
                travel_speed: 4.0,
            };
            let can_spawn = can_spawn_feature(
                &feature,
                &VecDeque::from_iter([obstacle].iter().cloned()),
                &world,
                0.,
                &Vector3::new(0., 0., 0.),
            );
            assert_eq!(can_spawn, false);
        }

        #[test]
        fn test_can_spawn_feature_collision_catch_up() {
            let prefab = Prefab {
                prefab_id: 0,
                position: Vector3::new(0., 0., 0.),
                rotation: UnitQuaternion::identity(),
                bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
                velocity: Vector3::new(0., 0., -1.),
            };
            let feature = Feature {
                translate_x: false,
                translate_x_using_bounds: false,
                translate_x_bounds: Vector2::new(0., 0.),
                translate_y: false,
                translate_y_using_bounds: false,
                translate_y_bounds: Vector2::new(0., 0.),
                prefabs: vec![prefab],
                spawn_count: 1,
                spawn_period: 1.,
                trigger_time: 10.,
                priority: 0,
                missed_spawns: 0,
                is_spawn_period_strict: false,
                last_spawn_attempt: 0.0,
            };
            let obstacle = CollideableEntity {
                spawn_position: Vector3::new(0., 0., -1.25),
                prefab_id: 0,
                rotation: UnitQuaternion::identity(),
                bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
                velocity: Vector3::new(0., -0., -0.5),
                spawn_time: 0.0,
                priority: 0,
            };
            let world = VisibleWorld {
                world_bounds: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(10., 10., 10.)),
                travel_speed: 4.0,
            };
            let can_spawn = can_spawn_feature(
                &feature,
                &VecDeque::from_iter([obstacle].iter().cloned()),
                &world,
                0.,
                &Vector3::new(0., 0., 0.),
            );
            assert_eq!(can_spawn, false);
        }

        #[test]
        fn test_can_spawn_feature_no_collision_catch_up() {
            let prefab = Prefab {
                prefab_id: 0,
                position: Vector3::new(0., 0., 0.),
                rotation: UnitQuaternion::identity(),
                bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
                velocity: Vector3::new(0., 0., -1.),
            };
            let feature = Feature {
                translate_x: false,
                translate_x_using_bounds: false,
                translate_x_bounds: Vector2::new(0., 0.),
                translate_y: false,
                translate_y_using_bounds: false,
                translate_y_bounds: Vector2::new(0., 0.),
                prefabs: vec![prefab],
                spawn_count: 1,
                spawn_period: 1.,
                trigger_time: 10.,
                priority: 0,
                missed_spawns: 0,
                is_spawn_period_strict: false,
                last_spawn_attempt: 0.0,
            };
            let obstacle = CollideableEntity {
                spawn_position: Vector3::new(0., 0., -2.5),
                prefab_id: 0,
                rotation: UnitQuaternion::identity(),
                bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
                velocity: Vector3::new(0., -0., -0.5),
                spawn_time: 0.0,
                priority: 0,
            };
            let world = VisibleWorld {
                world_bounds: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(10., 10., 10.)),
                travel_speed: 4.0,
            };
            let can_spawn = can_spawn_feature(
                &feature,
                &VecDeque::from_iter([obstacle].iter().cloned()),
                &world,
                0.,
                &Vector3::new(0., 0., 0.),
            );
            assert_eq!(can_spawn, true);
        }

        #[test]
        fn test_can_spawn_feature_collision_intersecting_on_spawn() {
            let prefab = Prefab {
                prefab_id: 0,
                position: Vector3::new(0., 0., 0.),
                rotation: UnitQuaternion::identity(),
                bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
                velocity: Vector3::new(0., 0., -1.),
            };
            let feature = Feature {
                translate_x: false,
                translate_x_using_bounds: false,
                translate_x_bounds: Vector2::new(0., 0.),
                translate_y: false,
                translate_y_using_bounds: false,
                translate_y_bounds: Vector2::new(0., 0.),
                prefabs: vec![prefab],
                spawn_count: 1,
                spawn_period: 1.,
                trigger_time: 10.,
                priority: 0,
                missed_spawns: 0,
                is_spawn_period_strict: false,
                last_spawn_attempt: 0.0,
            };
            let obstacle = CollideableEntity {
                spawn_position: Vector3::new(0., 0., 10.0),
                prefab_id: 0,
                rotation: UnitQuaternion::identity(),
                bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
                velocity: Vector3::new(0., -0., -1.0),
                spawn_time: 0.0,
                priority: 0,
            };
            let world = VisibleWorld {
                world_bounds: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(10., 10., 10.)),
                travel_speed: 4.0,
            };
            let can_spawn = can_spawn_feature(
                &feature,
                &VecDeque::from_iter([obstacle].iter().cloned()),
                &world,
                0.,
                &Vector3::new(0., 0., 0.),
            );
            assert_eq!(can_spawn, false);
        }

        #[test]
        fn test_can_spawn_rotated_collides_in_spawn_position() {
            let prefab0 = Prefab {
                prefab_id: 0,
                position: Vector3::new(0., 0., 0.),
                rotation: UnitQuaternion::from_euler_angles(std::f32::consts::FRAC_PI_2, 0., 0.),
                bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 4., 0.5)),
                velocity: Vector3::new(0., 0., -1.),
            };
            let feature0 = Feature {
                translate_x: false,
                translate_x_using_bounds: false,
                translate_x_bounds: Vector2::new(0., 0.),
                translate_y: false,
                translate_y_using_bounds: false,
                translate_y_bounds: Vector2::new(0., 0.),
                prefabs: vec![prefab0],
                spawn_count: 10,
                spawn_period: 1.0,
                trigger_time: 0.0,
                priority: 0,
                missed_spawns: 0,
                is_spawn_period_strict: false,
                last_spawn_attempt: 0.0,
            };

            let world = VisibleWorld {
                world_bounds: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(10., 10., 10.)),
                travel_speed: 4.0,
            };
            let obstacle = CollideableEntity {
                spawn_position: Vector3::new(0., 0., 2.1),
                prefab_id: 0,
                rotation: UnitQuaternion::from_euler_angles(std::f32::consts::FRAC_PI_2, 0., 0.),
                bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 4., 0.5)),
                velocity: Vector3::new(0., -0., -1.0),
                spawn_time: 0.0,
                priority: 0,
            };
            let can_spawn = can_spawn_feature(
                &feature0,
                &VecDeque::from_iter([obstacle].iter().cloned()),
                &world,
                0.,
                &Vector3::new(0., 0., 0.),
            );
            assert_eq!(can_spawn, false);
        }

        #[test]
        fn test_can_spawn_rotated_does_not_collide_in_spawn_position() {
            let prefab0 = Prefab {
                prefab_id: 0,
                position: Vector3::new(0., 0., 0.),
                rotation: UnitQuaternion::from_euler_angles(std::f32::consts::FRAC_PI_2, 0., 0.),
                bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 4., 0.5)),
                velocity: Vector3::new(0., 0., -1.),
            };
            let feature0 = Feature {
                translate_x: false,
                translate_x_using_bounds: false,
                translate_x_bounds: Vector2::new(0., 0.),
                translate_y: false,
                translate_y_using_bounds: false,
                translate_y_bounds: Vector2::new(0., 0.),
                prefabs: vec![prefab0],
                spawn_count: 10,
                spawn_period: 1.0,
                trigger_time: 0.0,
                priority: 0,
                missed_spawns: 0,
                is_spawn_period_strict: false,
                last_spawn_attempt: 0.0,
            };

            let world = VisibleWorld {
                world_bounds: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(10., 10., 10.)),
                travel_speed: 4.0,
            };
            let obstacle = CollideableEntity {
                spawn_position: Vector3::new(0., 0., 1.9),
                prefab_id: 0,
                rotation: UnitQuaternion::from_euler_angles(std::f32::consts::FRAC_PI_2, 0., 0.),
                bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 4., 0.5)),
                velocity: Vector3::new(0., -0., -1.0),
                spawn_time: 0.0,
                priority: 0,
            };
            let can_spawn = can_spawn_feature(
                &feature0,
                &VecDeque::from_iter([obstacle].iter().cloned()),
                &world,
                0.,
                &Vector3::new(0., 0., 0.),
            );
            assert_eq!(can_spawn, true);
        }

        #[test]
        fn test_can_spawn_rotated_does_collide_after_pursuit() {
            let prefab0 = Prefab {
                prefab_id: 0,
                position: Vector3::new(0., 0., 0.),
                rotation: UnitQuaternion::from_euler_angles(std::f32::consts::FRAC_PI_2, 0., 0.),
                bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 4., 0.5)),
                velocity: Vector3::new(0., 0., -2.),
            };
            let feature0 = Feature {
                translate_x: false,
                translate_x_using_bounds: false,
                translate_x_bounds: Vector2::new(0., 0.),
                translate_y: false,
                translate_y_using_bounds: false,
                translate_y_bounds: Vector2::new(0., 0.),
                prefabs: vec![prefab0],
                spawn_count: 10,
                spawn_period: 1.0,
                trigger_time: 0.0,
                priority: 0,
                missed_spawns: 0,
                is_spawn_period_strict: false,
                last_spawn_attempt: 0.0,
            };

            let world = VisibleWorld {
                world_bounds: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(10., 10., 10.)),
                travel_speed: 4.0,
            };
            let obstacle = CollideableEntity {
                spawn_position: Vector3::new(0., 0., -5.),
                prefab_id: 0,
                rotation: UnitQuaternion::from_euler_angles(std::f32::consts::FRAC_PI_2, 0., 0.),
                bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 4., 0.5)),
                velocity: Vector3::new(0., -0., -1.0),
                spawn_time: 0.0,
                priority: 0,
            };
            let can_spawn = can_spawn_feature(
                &feature0,
                &VecDeque::from_iter([obstacle].iter().cloned()),
                &world,
                0.,
                &Vector3::new(0., 0., 0.),
            );
            assert_eq!(can_spawn, false);
        }
    }

    mod non_zero_travel_time_with_shift {
        use super::*;
        use nalgebra::UnitQuaternion;

        #[test]
        fn test_can_spawn_feature_collision() {
            let prefab = Prefab {
                prefab_id: 0,
                position: Vector3::new(0., 0., 0.),
                rotation: UnitQuaternion::identity(),
                bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
                velocity: Vector3::new(0., 0., -1.),
            };
            let feature = Feature {
                translate_x: false,
                translate_x_using_bounds: false,
                translate_x_bounds: Vector2::new(0., 0.),
                translate_y: false,
                translate_y_using_bounds: false,
                translate_y_bounds: Vector2::new(0., 0.),
                prefabs: vec![prefab],
                spawn_count: 1,
                spawn_period: 1.,
                trigger_time: 10.,
                priority: 5,
                missed_spawns: 0,
                is_spawn_period_strict: false,
                last_spawn_attempt: 0.0,
            };
            let obstacle = CollideableEntity {
                spawn_position: Vector3::new(0., 5., 8.),
                prefab_id: 0,
                rotation: UnitQuaternion::identity(),
                bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
                velocity: Vector3::new(0., -0.5, -0.5),
                spawn_time: 5.0,
                priority: 0,
            };
            let world = VisibleWorld {
                world_bounds: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(10., 10., 10.)),
                travel_speed: 4.0,
            };
            let can_spawn = can_spawn_feature(
                &feature,
                &VecDeque::from_iter([obstacle].iter().cloned()),
                &world,
                5.,
                &Vector3::new(0., 0., 0.),
            );
            assert_eq!(can_spawn, false);
        }
    }
}