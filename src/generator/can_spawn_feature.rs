use super::types::{CollideableEntity, VisibleWorld};
use std::collections::VecDeque;
use ncollide3d::shape::Cuboid;
use ncollide3d::query;
use ncollide3d::query::{RayCast, Ray};
use nalgebra::{Isometry3, Vector3, Point3, Translation3, Isometry, UnitQuaternion, U3, Translation, Unit};
use rayon::prelude::*;
use ncollide3d::bounding_volume::{BoundingVolume, AABB};
use ncollide3d::interpolation::{RigidMotion};
use crate::generator::types::Feature;
use crate::generator::tilt_motion::ConstantVelocityZTiltMotion;
use crate::Movement;

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
    let max_time_to_travel = feature.max_time_to_travel(&world, feature_shift.z);
    let any_prefab_in_feature_collides_with_any_obstacle = feature.prefabs
        .par_iter()
        .any(|prefab| {
            let any_obstacle_collides_with_prefab = obstacles
                .into_par_iter()
                .any(|obstacle| {
                    let prefab_motion = ConstantVelocityZTiltMotion::new(
                        max_time_to_travel + feature.priority as f32,
                        Isometry3::from_parts(Translation3::from(prefab.position + Vector3::new(feature_shift.x, feature_shift.y, 0.)), prefab.rotation),
                        prefab.movement.linear_velocity.clone(),
                        prefab.movement.z_axis_tilt_xy_direction.clone(),
                        prefab.movement.z_axis_tilt_angle,
                        prefab.movement.z_axis_tilt_distance,
                        prefab.movement.z_axis_tilt_easing_range,
                        prefab.movement.z_axis_tilt_rotation_strength,
                    );
                    let prefab_spawn_position_isometry: Isometry<f32, U3, UnitQuaternion<f32>> = prefab_motion.position_at_time(0.);

                    let obstacle_motion = ConstantVelocityZTiltMotion::new_from_tilted_start(
                        obstacle.spawn_time - time_travelled,
                        Isometry3::from_parts(Translation3::from(obstacle.spawn_position), obstacle.spawn_rotation_without_tilt),
                        obstacle.movement.linear_velocity.clone(),
                        obstacle.movement.z_axis_tilt_xy_direction.clone(),
                        obstacle.movement.z_axis_tilt_angle,
                        obstacle.movement.z_axis_tilt_distance,
                        obstacle.movement.z_axis_tilt_easing_range,
                        obstacle.movement.z_axis_tilt_rotation_strength,
                    );

                    let prefab_world_bounds_toi = world.world_bounds
                        // This is required because sometimes, due to priority, prefab spawn position is outside world bounds
                        .merged(&prefab.bounding_box.transform_by(&prefab_spawn_position_isometry))
                        .toi_with_ray(
                            &Isometry3::new(nalgebra::zero(), nalgebra::zero()),
                            &Ray::new(Point3::origin() + prefab_spawn_position_isometry.translation.vector, prefab.movement.linear_velocity),
                            f32::MAX,
                            false,
                        ).unwrap()
                        +
                        prefab.bounding_box
                            .toi_with_ray(
                                &Isometry3::from_parts(Translation3::identity(), prefab.rotation),
                                &Ray::new(Point3::origin(), -prefab.movement.linear_velocity),
                                f32::MAX,
                                false,
                            ).unwrap();

                    let prefab_bounding_box = Cuboid::new(find_max_rotated_aabb(&prefab.bounding_box, &prefab.movement).half_extents());
                    let obstacle_bounding_box = Cuboid::new(find_max_rotated_aabb(&obstacle.bounding_box, &obstacle.movement).half_extents());
                    let time_of_impact = query::nonlinear_time_of_impact(
                        &query::DefaultTOIDispatcher,
                        &prefab_motion,
                        &prefab_bounding_box,
                        &obstacle_motion,
                        &obstacle_bounding_box,
                        prefab_world_bounds_toi,
                        0.0,
                    );
                    match time_of_impact {
                        Ok(time_of_impact_option) => {
                            return match time_of_impact_option {
                                None => {
                                    false
                                }
                                Some(_toi) => {
                                    // dbg!(_toi);
                                    true
                                }
                            };
                        }
                        Err(error) => {
                            panic!(error)
                        }
                    }
                });
            any_obstacle_collides_with_prefab
        });
    !any_prefab_in_feature_collides_with_any_obstacle
}

// TODO This won't be needed with toi rotation prediction algo in ncollide
fn find_max_rotated_aabb(original_aabb: &AABB<f32>, movement: &Movement) -> AABB<f32> {
    if movement.z_axis_tilt_angle.is_normal() &&
        movement.z_axis_tilt_rotation_strength.is_normal() &&
        movement.z_axis_tilt_xy_direction.len() > f32::EPSILON as usize {
        let rotation_axis = Unit::new_normalize(Vector3::new(movement.z_axis_tilt_xy_direction.x, movement.z_axis_tilt_xy_direction.y, 0.).cross(&Vector3::z_axis()));
        let angle = movement.z_axis_tilt_angle.to_radians() * movement.z_axis_tilt_rotation_strength;
        let rotated_aabb = original_aabb
            .transform_by(
                &Isometry::from_parts(
                    Translation::identity(),
                    UnitQuaternion::from_axis_angle(&rotation_axis, angle),
                )
            );
        let result = original_aabb.merged(&rotated_aabb);
        return result;
    }
    return original_aabb.clone();
}

#[cfg(test)]
mod tests {
    use crate::generator::types::{Prefab, Feature, VisibleWorld, CollideableEntity};
    use ncollide3d::bounding_volume::AABB;
    use nalgebra::{Vector3, Point3, Vector2};
    use crate::generator::can_spawn_feature::can_spawn_feature;
    use std::collections::VecDeque;
    use std::iter::FromIterator;

    mod zero_travel_time {
        use super::*;
        use nalgebra::UnitQuaternion;
        use crate::generator::Movement;

        #[test]
        fn test_can_spawn_feature_collision() {
            let prefab = Prefab {
                prefab_id: 0,
                position: Vector3::new(0., 0., 0.),
                rotation: UnitQuaternion::identity(),
                bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
                movement: Movement {
                    linear_velocity: Vector3::new(0., 0., -1.),
                    z_axis_tilt_xy_direction: nalgebra::zero(),
                    z_axis_tilt_angle: 0.0,
                    z_axis_tilt_distance: 0.0,
                    z_axis_tilt_easing_range: 0.0,
                    z_axis_tilt_rotation_strength: 0.,
                },
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
                translate_z: 0.0,
            };
            let obstacle = CollideableEntity {
                spawn_position: Vector3::new(0., 5., 10.),
                spawn_rotation: UnitQuaternion::identity(),
                spawn_rotation_without_tilt: UnitQuaternion::identity(),
                prefab_id: 0,
                bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
                movement: Movement {
                    linear_velocity: Vector3::new(0., -1., -1.),
                    z_axis_tilt_xy_direction: nalgebra::zero(),
                    z_axis_tilt_angle: 0.0,
                    z_axis_tilt_distance: 0.0,
                    z_axis_tilt_easing_range: 0.0,
                    z_axis_tilt_rotation_strength: 0.,
                },
                spawn_time: 0.0,
                priority: 0,
            };
            let world = VisibleWorld {
                world_bounds: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(10., 10., 10.)),
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
                movement: Movement {
                    linear_velocity: Vector3::new(0., 0., -1.),
                    z_axis_tilt_xy_direction: nalgebra::zero(),
                    z_axis_tilt_angle: 0.0,
                    z_axis_tilt_distance: 0.0,
                    z_axis_tilt_easing_range: 0.0,
                    z_axis_tilt_rotation_strength: 0.,
                },
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
                translate_z: 0.0,
            };
            let obstacle = CollideableEntity {
                prefab_id: 0,
                spawn_position: Vector3::new(0., 0., -1.25),
                spawn_rotation: UnitQuaternion::identity(),
                spawn_rotation_without_tilt: UnitQuaternion::identity(),
                bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
                movement: Movement {
                    linear_velocity: Vector3::new(0., 0., -0.5),
                    z_axis_tilt_xy_direction: nalgebra::zero(),
                    z_axis_tilt_angle: 0.0,
                    z_axis_tilt_distance: 0.0,
                    z_axis_tilt_easing_range: 0.0,
                    z_axis_tilt_rotation_strength: 0.,
                },
                spawn_time: 0.0,
                priority: 0,
            };
            let world = VisibleWorld {
                world_bounds: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(10., 10., 10.)),
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
                movement: Movement {
                    linear_velocity: Vector3::new(0., 0., -1.),
                    z_axis_tilt_xy_direction: nalgebra::zero(),
                    z_axis_tilt_angle: 0.0,
                    z_axis_tilt_distance: 0.0,
                    z_axis_tilt_easing_range: 0.0,
                    z_axis_tilt_rotation_strength: 0.,
                },
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
                translate_z: 0.0,
            };
            let obstacle = CollideableEntity {
                spawn_position: Vector3::new(0., 0., -2.5),
                spawn_rotation: UnitQuaternion::identity(),
                spawn_rotation_without_tilt: UnitQuaternion::identity(),
                bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
                movement: Movement {
                    linear_velocity: Vector3::new(0., 0., -0.5),
                    z_axis_tilt_xy_direction: nalgebra::zero(),
                    z_axis_tilt_angle: 0.0,
                    z_axis_tilt_distance: 0.0,
                    z_axis_tilt_easing_range: 0.0,
                    z_axis_tilt_rotation_strength: 0.,
                },
                spawn_time: 0.0,
                prefab_id: 0,
                priority: 0,
            };
            let world = VisibleWorld {
                world_bounds: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(10., 10., 10.)),
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
                movement: Movement {
                    linear_velocity: Vector3::new(0., 0., -1.),
                    z_axis_tilt_xy_direction: nalgebra::zero(),
                    z_axis_tilt_angle: 0.0,
                    z_axis_tilt_distance: 0.0,
                    z_axis_tilt_easing_range: 0.0,
                    z_axis_tilt_rotation_strength: 0.,
                },
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
                translate_z: 0.0,
            };
            let obstacle = CollideableEntity {
                spawn_position: Vector3::new(0., 0., 10.0),
                spawn_rotation: UnitQuaternion::identity(),
                spawn_rotation_without_tilt: UnitQuaternion::identity(),
                bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
                movement: Movement {
                    linear_velocity: Vector3::new(0., 0., -1.),
                    z_axis_tilt_xy_direction: nalgebra::zero(),
                    z_axis_tilt_angle: 0.0,
                    z_axis_tilt_distance: 0.0,
                    z_axis_tilt_easing_range: 0.0,
                    z_axis_tilt_rotation_strength: 0.,
                },
                spawn_time: 0.0,
                prefab_id: 0,
                priority: 0,
            };
            let world = VisibleWorld {
                world_bounds: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(10., 10., 10.)),
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
                movement: Movement {
                    linear_velocity: Vector3::new(0., 0., -1.),
                    z_axis_tilt_xy_direction: nalgebra::zero(),
                    z_axis_tilt_angle: 0.0,
                    z_axis_tilt_distance: 0.0,
                    z_axis_tilt_easing_range: 0.0,
                    z_axis_tilt_rotation_strength: 0.,
                },
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
                translate_z: 0.0,
            };

            let world = VisibleWorld {
                world_bounds: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(10., 10., 10.)),
            };
            let obstacle = CollideableEntity {
                spawn_position: Vector3::new(0., 0., 2.1),
                spawn_rotation: UnitQuaternion::identity(),
                spawn_rotation_without_tilt: UnitQuaternion::from_euler_angles(std::f32::consts::FRAC_PI_2, 0., 0.),
                bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 4., 0.5)),
                movement: Movement {
                    linear_velocity: Vector3::new(0., 0., -1.),
                    z_axis_tilt_xy_direction: nalgebra::zero(),
                    z_axis_tilt_angle: 0.0,
                    z_axis_tilt_distance: 0.0,
                    z_axis_tilt_easing_range: 0.0,
                    z_axis_tilt_rotation_strength: 0.,
                },
                spawn_time: 0.0,
                prefab_id: 0,
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
                movement: Movement {
                    linear_velocity: Vector3::new(0., 0., -1.),
                    z_axis_tilt_xy_direction: nalgebra::zero(),
                    z_axis_tilt_angle: 0.0,
                    z_axis_tilt_distance: 0.0,
                    z_axis_tilt_easing_range: 0.0,
                    z_axis_tilt_rotation_strength: 0.,
                },
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
                translate_z: 0.0,
            };

            let world = VisibleWorld {
                world_bounds: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(10., 10., 10.)),
            };
            let obstacle = CollideableEntity {
                spawn_position: Vector3::new(0., 0., 1.9),
                spawn_rotation: UnitQuaternion::identity(),
                spawn_rotation_without_tilt: UnitQuaternion::from_euler_angles(std::f32::consts::FRAC_PI_2, 0., 0.),
                bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 4., 0.5)),
                movement: Movement {
                    linear_velocity: Vector3::new(0., 0., -1.),
                    z_axis_tilt_xy_direction: nalgebra::zero(),
                    z_axis_tilt_angle: 0.0,
                    z_axis_tilt_distance: 0.0,
                    z_axis_tilt_easing_range: 0.0,
                    z_axis_tilt_rotation_strength: 0.,
                },
                spawn_time: 0.0,
                prefab_id: 0,
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
                movement: Movement {
                    linear_velocity: Vector3::new(0., 0., -2.),
                    z_axis_tilt_xy_direction: nalgebra::zero(),
                    z_axis_tilt_angle: 0.0,
                    z_axis_tilt_distance: 0.0,
                    z_axis_tilt_easing_range: 0.0,
                    z_axis_tilt_rotation_strength: 0.,
                },
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
                translate_z: 0.0,
            };

            let world = VisibleWorld {
                world_bounds: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(10., 10., 10.)),
            };
            let obstacle = CollideableEntity {
                spawn_position: Vector3::new(0., 0., -5.),
                spawn_rotation: UnitQuaternion::identity(),
                spawn_rotation_without_tilt: UnitQuaternion::from_euler_angles(std::f32::consts::FRAC_PI_2, 0., 0.),
                bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 4., 0.5)),
                movement: Movement {
                    linear_velocity: Vector3::new(0., 0., -1.),
                    z_axis_tilt_xy_direction: nalgebra::zero(),
                    z_axis_tilt_angle: 0.0,
                    z_axis_tilt_distance: 0.0,
                    z_axis_tilt_easing_range: 0.0,
                    z_axis_tilt_rotation_strength: 0.,
                },
                spawn_time: 0.0,
                prefab_id: 0,
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
        use crate::Movement;

        #[test]
        fn test_can_spawn_feature_collision() {
            let prefab = Prefab {
                prefab_id: 0,
                position: Vector3::new(0., 0., 0.),
                rotation: UnitQuaternion::identity(),
                bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
                movement: Movement {
                    linear_velocity: Vector3::new(0., 0., -1.),
                    z_axis_tilt_xy_direction: nalgebra::zero(),
                    z_axis_tilt_angle: 0.0,
                    z_axis_tilt_distance: 0.0,
                    z_axis_tilt_easing_range: 0.0,
                    z_axis_tilt_rotation_strength: 0.,
                },
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
                translate_z: 0.0,
            };
            let obstacle = CollideableEntity {
                spawn_position: Vector3::new(0., 5., 8.),
                spawn_rotation: UnitQuaternion::identity(),
                spawn_rotation_without_tilt: UnitQuaternion::identity(),
                bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
                movement: Movement {
                    linear_velocity: Vector3::new(0., -0.5, -0.5),
                    z_axis_tilt_xy_direction: nalgebra::zero(),
                    z_axis_tilt_angle: 0.0,
                    z_axis_tilt_distance: 0.0,
                    z_axis_tilt_easing_range: 0.0,
                    z_axis_tilt_rotation_strength: 0.,
                },
                spawn_time: 5.0,
                prefab_id: 0,
                priority: 0,
            };
            let world = VisibleWorld {
                world_bounds: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(10., 10., 10.)),
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

    mod not_collinear_not_tilted {
        use super::*;
        use nalgebra::UnitQuaternion;
        use crate::Movement;

        #[test]
        fn test_cannot_spawn_if_collide_not_collinear() {
            let prefab = Prefab {
                prefab_id: 0,
                position: Vector3::new(0., 0., 0.),
                rotation: UnitQuaternion::identity(),
                bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
                movement: Movement {
                    linear_velocity: Vector3::new(0., 0., -1.),
                    z_axis_tilt_xy_direction: nalgebra::zero(),
                    z_axis_tilt_angle: 0.0,
                    z_axis_tilt_distance: 0.0,
                    z_axis_tilt_easing_range: 0.0,
                    z_axis_tilt_rotation_strength: 0.,
                },
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
                translate_z: 0.0,
            };
            let obstacle = CollideableEntity {
                spawn_position: Vector3::new(0., 100., 100.),
                spawn_rotation: UnitQuaternion::identity(),
                spawn_rotation_without_tilt: UnitQuaternion::identity(),
                bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
                movement: Movement {
                    linear_velocity: Vector3::new(0., -1., -1.),
                    z_axis_tilt_xy_direction: nalgebra::zero(),
                    z_axis_tilt_angle: 0.0,
                    z_axis_tilt_distance: 0.0,
                    z_axis_tilt_easing_range: 0.0,
                    z_axis_tilt_rotation_strength: 0.,
                },
                spawn_time: 5.0,
                prefab_id: 0,
                priority: 0,
            };
            let world = VisibleWorld {
                world_bounds: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(100., 100., 100.)),
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

    mod tilted {
        use super::*;
        use nalgebra::{UnitQuaternion};
        use crate::Movement;
        use crate::generator::can_spawn_feature::find_max_rotated_aabb;

        #[test]
        fn test_cannot_spawn_if_collide_obstacle_tilted() {
            let prefab = Prefab {
                prefab_id: 0,
                position: Vector3::new(0., 0., 0.),
                rotation: UnitQuaternion::identity(),
                bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
                movement: Movement {
                    linear_velocity: Vector3::new(0., 0., -1.),
                    z_axis_tilt_xy_direction: nalgebra::zero(),
                    z_axis_tilt_angle: 0.0,
                    z_axis_tilt_distance: 0.0,
                    z_axis_tilt_easing_range: 0.0,
                    z_axis_tilt_rotation_strength: 0.,
                },
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
                translate_z: 0.0,
            };
            let obstacle = CollideableEntity {
                spawn_position: Vector3::new(0., 100., 100.),
                spawn_rotation: UnitQuaternion::identity(),
                spawn_rotation_without_tilt: UnitQuaternion::identity(),
                bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
                movement: Movement {
                    linear_velocity: Vector3::new(0., 0., -1.),
                    z_axis_tilt_xy_direction: Vector2::new(0., 1.),
                    z_axis_tilt_angle: 45.0,
                    z_axis_tilt_distance: 0.0,
                    z_axis_tilt_easing_range: 0.0,
                    z_axis_tilt_rotation_strength: 0.,
                },
                spawn_time: 5.0,
                prefab_id: 0,
                priority: 0,
            };
            let world = VisibleWorld {
                world_bounds: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(100., 100., 100.)),
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

        #[test]
        fn test_cannot_spawn_if_collide_prefab_tilted() {
            let prefab = Prefab {
                prefab_id: 0,
                position: Vector3::new(0., 0., 0.),
                rotation: UnitQuaternion::identity(),
                bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
                movement: Movement {
                    linear_velocity: Vector3::new(0., 0., -1.),
                    z_axis_tilt_xy_direction: Vector2::new(0., 1.),
                    z_axis_tilt_angle: 45.0,
                    z_axis_tilt_distance: 0.0,
                    z_axis_tilt_easing_range: 10.0,
                    z_axis_tilt_rotation_strength: 0.,
                },
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
                translate_z: 0.0,
            };
            let obstacle = CollideableEntity {
                spawn_position: Vector3::new(0., 0., 100.),
                spawn_rotation: UnitQuaternion::identity(),
                spawn_rotation_without_tilt: UnitQuaternion::identity(),
                bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
                movement: Movement {
                    linear_velocity: Vector3::new(0., 0., -1.),
                    z_axis_tilt_xy_direction: Vector2::new(0., 1.),
                    z_axis_tilt_angle: 0.0,
                    z_axis_tilt_distance: 0.0,
                    z_axis_tilt_easing_range: 0.0,
                    z_axis_tilt_rotation_strength: 0.,
                },
                spawn_time: 5.0,
                prefab_id: 0,
                priority: 0,
            };
            let world = VisibleWorld {
                world_bounds: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(100., 100., 100.)),
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

        #[test]
        fn test_cannot_spawn_if_collide_prefab_tilted_obstacle_tilted() {
            let prefab = Prefab {
                prefab_id: 0,
                position: Vector3::new(0., 0., 0.),
                rotation: UnitQuaternion::identity(),
                bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
                movement: Movement {
                    linear_velocity: Vector3::new(0., 0., -1.),
                    z_axis_tilt_xy_direction: Vector2::new(0., 1.),
                    z_axis_tilt_angle: 45.0,
                    z_axis_tilt_distance: 0.0,
                    z_axis_tilt_easing_range: 0.0,
                    z_axis_tilt_rotation_strength: 0.,
                },
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
                translate_z: 0.0,
            };
            let obstacle = CollideableEntity {
                spawn_position: Vector3::new(0., 100., 100.),
                spawn_rotation: UnitQuaternion::identity(),
                spawn_rotation_without_tilt: UnitQuaternion::identity(),
                bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
                movement: Movement {
                    linear_velocity: Vector3::new(0., 0., -1.),
                    z_axis_tilt_xy_direction: Vector2::new(0., 1.),
                    z_axis_tilt_angle: 45.0,
                    z_axis_tilt_distance: 0.0,
                    z_axis_tilt_easing_range: 0.0,
                    z_axis_tilt_rotation_strength: 0.,
                },
                spawn_time: 5.0,
                prefab_id: 0,
                priority: 0,
            };
            let world = VisibleWorld {
                world_bounds: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(100., 100., 100.)),
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

        #[test]
        fn test_find_max_rotated_aabb() {
            let aabb = AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 5., 0.5));
            let movement = Movement {
                linear_velocity: Vector3::new(0., 0., -1.),
                z_axis_tilt_xy_direction: Vector2::new(0., 1.),
                z_axis_tilt_angle: 45.0,
                z_axis_tilt_distance: 0.0,
                z_axis_tilt_easing_range: 0.0,
                z_axis_tilt_rotation_strength: 1.0,
            };
            let max_aabb = find_max_rotated_aabb(&aabb, &movement);
            assert_eq!(max_aabb, AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 5., 3.8890874)))
        }

        #[test]
        fn test_can_spawn_if_dont_collide_with_zero_rotation_strength() {
            // TODO Current toi algo does not work with rotating motions in ncollide
            // let prefab = Prefab {
            //     prefab_id: 0,
            //     position: Vector3::new(0., 0., 0.),
            //     rotation: UnitQuaternion::identity(),
            //     bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
            //     movement: Movement {
            //         linear_velocity: Vector3::new(0., 0., -1.),
            //         z_axis_tilt_xy_direction: Vector2::new(0., 1.),
            //         z_axis_tilt_angle: 45.0,
            //         z_axis_tilt_distance: 0.0,
            //         z_axis_tilt_easing_range: 0.0,
            //         z_axis_tilt_rotation_strength: 0.,
            //     },
            // };
            // let feature = Feature {
            //     translate_x: false,
            //     translate_x_using_bounds: false,
            //     translate_x_bounds: Vector2::new(0., 0.),
            //     translate_y: false,
            //     translate_y_using_bounds: false,
            //     translate_y_bounds: Vector2::new(0., 0.),
            //     prefabs: vec![prefab],
            //     spawn_count: 1,
            //     spawn_period: 1.,
            //     trigger_time: 10.,
            //     priority: 0,
            //     missed_spawns: 0,
            //     is_spawn_period_strict: false,
            //     last_spawn_attempt: 0.0,
            //     translate_z: 0.0,
            // };
            // let obstacle = CollideableEntity {
            //     spawn_position: Vector3::new(0., 0., 100.),
            //     prefab_id: 0,
            //     rotation: UnitQuaternion::identity(),
            //     bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
            //     movement: Movement {
            //         linear_velocity: Vector3::new(0., 0., -1.),
            //         z_axis_tilt_xy_direction: Vector2::new(0., 1.),
            //         z_axis_tilt_angle: 0.0,
            //         z_axis_tilt_distance: 0.0,
            //         z_axis_tilt_easing_range: 0.0,
            //         z_axis_tilt_rotation_strength: 0.,
            //     },
            //     spawn_time: 5.0,
            //     priority: 0,
            // };
            // let world = VisibleWorld {
            //     world_bounds: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(100., 100., 100.)),
            // };
            // let can_spawn = can_spawn_feature(
            //     &feature,
            //     &VecDeque::from_iter([obstacle].iter().cloned()),
            //     &world,
            //     5.,
            //     &Vector3::new(0., 0., 0.),
            // );
            // assert_eq!(can_spawn, true);
        }
    }
}