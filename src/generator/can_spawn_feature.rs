use super::types::{CollidableEntity, VisibleWorld};
use std::collections::VecDeque;
use ncollide3d::shape::Cuboid;
use ncollide3d::query;
use nalgebra::{Isometry3, Vector3, Translation3};
use rayon::prelude::*;
use crate::generator::types::Feature;
use crate::generator::bi_arc_motion::{BiArcCurveMotion};

/// Checks if a feature can be safely spawn so that it won't collide with any existing entities in
/// a visible world
///
pub fn can_spawn_feature(
    feature: &Feature,
    obstacles: &VecDeque<CollidableEntity>,
    world: &VisibleWorld,
    time_travelled: f32,
    feature_shift: &Vector3<f32>,
) -> bool {
    let max_time_to_travel = feature.max_time_to_travel(&world, &feature_shift);
    let any_prefab_in_feature_collides_with_any_obstacle = feature.prefabs
        .par_iter()
        .any(|prefab| {
            let any_obstacle_collides_with_prefab = obstacles
                .into_par_iter()
                .any(|obstacle| {
                    let prefab_motion = BiArcCurveMotion::new(
                        max_time_to_travel + feature.priority as f32,
                        Isometry3::from_parts(Translation3::from(prefab.position + Vector3::new(feature_shift.x, feature_shift.y, 0.)), prefab.rotation),
                        prefab.movement.baseline_velocity.clone(),
                        prefab.movement.arcs_plane_normal.clone(),
                        prefab.movement.approach_arc_angle,
                        prefab.movement.approach_arc_center_distance,
                        prefab.movement.approach_arc_radius,
                        prefab.movement.approach_rotation_strength,
                        prefab.movement.approach_arc_angle,
                        prefab.movement.approach_arc_center_distance,
                        prefab.movement.approach_arc_radius,
                        prefab.movement.approach_rotation_strength,
                    );

                    let obstacle_motion = BiArcCurveMotion::new(
                        obstacle.spawn_time - time_travelled - obstacle.movement_start_parameter,
                        Isometry3::from_parts(Translation3::from(obstacle.prefab.position + obstacle.spawn_feature_shift), obstacle.prefab.rotation),
                        obstacle.prefab.movement.baseline_velocity.clone(),
                        obstacle.prefab.movement.arcs_plane_normal.clone(),
                        obstacle.prefab.movement.approach_arc_angle,
                        obstacle.prefab.movement.approach_arc_center_distance,
                        obstacle.prefab.movement.approach_arc_radius,
                        obstacle.prefab.movement.approach_rotation_strength,
                        obstacle.prefab.movement.approach_arc_angle,
                        obstacle.prefab.movement.approach_arc_center_distance,
                        obstacle.prefab.movement.approach_arc_radius,
                        obstacle.prefab.movement.approach_rotation_strength,
                    );

                    let prefab_bounding_box = Cuboid::new(prefab.bounding_box.half_extents());
                    let obstacle_bounding_box = Cuboid::new(obstacle.prefab.bounding_box.half_extents());
                    let time_of_impact = query::nonlinear_time_of_impact(
                        &query::DefaultTOIDispatcher,
                        &prefab_motion,
                        &prefab_bounding_box,
                        &obstacle_motion,
                        &obstacle_bounding_box,
                        max_time_to_travel + feature.priority as f32 + prefab.find_departure_time_in_world(&world,  &feature_shift),
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

#[cfg(test)]
mod tests {
    use crate::generator::types::{Prefab, Feature, VisibleWorld, CollidableEntity};
    use ncollide3d::bounding_volume::AABB;
    use nalgebra::{Vector3, Point3, Vector2};
    use crate::generator::can_spawn_feature::can_spawn_feature;
    use std::collections::VecDeque;
    use std::iter::FromIterator;

    mod zero_travel_time {
        use super::*;
        use nalgebra::{UnitQuaternion, Unit};
        use crate::generator::Movement;

        #[test]
        fn test_can_spawn_feature_collision() {
            let prefab = Prefab {
                prefab_id: 0,
                position: Vector3::new(0., 0., 0.),
                rotation: UnitQuaternion::identity(),
                bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
                movement: Movement {
                    baseline_velocity: Vector3::new(0., 0., -1.),
                    arcs_plane_normal: Unit::new_normalize(Vector3::new(1., 0., 0.)),
                    approach_arc_angle: 0.0,
                    approach_arc_center_distance: 0.0,
                    approach_arc_radius: 0.0,
                    approach_rotation_strength: 0.,
                    departure_arc_angle: 0.0,
                    departure_arc_center_distance: 0.0,
                    departure_arc_radius: 0.0,
                    departure_rotation_strength: 0.0,
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
            let obstacle = CollidableEntity {
                movement_start_parameter: -10.5,
                movement_end_parameter: 0.0,
                spawn_position: Vector3::new(0., 5., 10.),
                spawn_rotation: UnitQuaternion::identity(),
                spawn_feature_shift: nalgebra::zero(),
                spawn_time: 0.0,
                prefab: Prefab {
                    prefab_id: 0,
                    position: nalgebra::zero(),
                    rotation: UnitQuaternion::identity(),
                    bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
                    movement: Movement {
                        baseline_velocity: Vector3::new(0., -1., -1.),
                        arcs_plane_normal: Unit::new_normalize(Vector3::new(1., 0., 0.)),
                        approach_arc_angle: 0.0,
                        approach_arc_center_distance: 0.0,
                        approach_arc_radius: 0.0,
                        approach_rotation_strength: 0.,
                        departure_arc_angle: 0.0,
                        departure_arc_center_distance: 0.0,
                        departure_arc_radius: 0.0,
                        departure_rotation_strength: 0.0,
                    },
                },
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
                    baseline_velocity: Vector3::new(0., 0., -1.),
                    arcs_plane_normal: Unit::new_normalize(Vector3::new(1., 0., 0.)),
                    approach_arc_angle: 0.0,
                    approach_arc_center_distance: 0.0,
                    approach_arc_radius: 0.0,
                    approach_rotation_strength: 0.,
                    departure_arc_angle: 0.0,
                    departure_arc_center_distance: 0.0,
                    departure_arc_radius: 0.0,
                    departure_rotation_strength: 0.0,
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
            let obstacle = CollidableEntity {
                movement_start_parameter: -10.5,
                spawn_position: Vector3::new(0., 0., -1.25),
                spawn_rotation: UnitQuaternion::identity(),
                spawn_feature_shift: nalgebra::zero(),
                prefab: Prefab {
                    prefab_id: 0,
                    position: Default::default(),
                    rotation: Default::default(),
                    bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
                    movement: Movement {
                        baseline_velocity: Vector3::new(0., 0., -0.5),
                        arcs_plane_normal: Unit::new_normalize(Vector3::new(1., 0., 0.)),
                        approach_arc_angle: 0.0,
                        approach_arc_center_distance: 0.0,
                        approach_arc_radius: 0.0,
                        approach_rotation_strength: 0.,
                        departure_arc_angle: 0.0,
                        departure_arc_center_distance: 0.0,
                        departure_arc_radius: 0.0,
                        departure_rotation_strength: 0.0,
                    },
                },
                spawn_time: 0.0,
                priority: 0,
                movement_end_parameter: 0.0,
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
                    baseline_velocity: Vector3::new(0., 0., -1.),
                    arcs_plane_normal: Unit::new_normalize(Vector3::new(1., 0., 0.)),
                    approach_arc_angle: 0.0,
                    approach_arc_center_distance: 0.0,
                    approach_arc_radius: 0.0,
                    approach_rotation_strength: 0.,
                    departure_arc_angle: 0.0,
                    departure_arc_center_distance: 0.0,
                    departure_arc_radius: 0.0,
                    departure_rotation_strength: 0.0,
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
            let obstacle = CollidableEntity {
                movement_start_parameter: -24.,
                movement_end_parameter: 0.0,
                spawn_position: Vector3::new(0., 0., -2.5),
                spawn_rotation: UnitQuaternion::identity(),
                spawn_feature_shift: nalgebra::zero(),
                spawn_time: 0.0,
                priority: 0,
                prefab: Prefab {
                    prefab_id: 0,
                    position: nalgebra::zero(),
                    rotation: Default::default(),
                    bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
                    movement: Movement {
                        baseline_velocity: Vector3::new(0., 0., -0.5),
                        arcs_plane_normal: Unit::new_normalize(Vector3::new(1., 0., 0.)),
                        approach_arc_angle: 0.0,
                        approach_arc_center_distance: 0.0,
                        approach_arc_radius: 0.0,
                        approach_rotation_strength: 0.,
                        departure_arc_angle: 0.0,
                        departure_arc_center_distance: 0.0,
                        departure_arc_radius: 0.0,
                        departure_rotation_strength: 0.0,
                    },
                },
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
                    baseline_velocity: Vector3::new(0., 0., -1.),
                    arcs_plane_normal: Unit::new_normalize(Vector3::new(1., 0., 0.)),
                    approach_arc_angle: 0.0,
                    approach_arc_center_distance: 0.0,
                    approach_arc_radius: 0.0,
                    approach_rotation_strength: 0.,
                    departure_arc_angle: 0.0,
                    departure_arc_center_distance: 0.0,
                    departure_arc_radius: 0.0,
                    departure_rotation_strength: 0.0,
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
            let obstacle = CollidableEntity {
                movement_start_parameter: -10.5,
                movement_end_parameter: 0.0,
                spawn_position: Vector3::new(0., 0., 10.0),
                spawn_rotation: UnitQuaternion::identity(),
                spawn_feature_shift: nalgebra::zero(),
                spawn_time: 0.0,
                priority: 0,
                prefab: Prefab {
                    prefab_id: 0,
                    position: nalgebra::zero(),
                    rotation: Default::default(),
                    bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
                    movement: Movement {
                        baseline_velocity: Vector3::new(0., 0., -1.),
                        arcs_plane_normal: Unit::new_normalize(Vector3::new(1., 0., 0.)),
                        approach_arc_angle: 0.0,
                        approach_arc_center_distance: 0.0,
                        approach_arc_radius: 0.0,
                        approach_rotation_strength: 0.,
                        departure_arc_angle: 0.0,
                        departure_arc_center_distance: 0.0,
                        departure_arc_radius: 0.0,
                        departure_rotation_strength: 0.0,
                    },

                },
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

        #[ignore]
        #[test]
        fn test_can_spawn_rotated_collides_in_spawn_position() {
            let prefab0 = Prefab {
                prefab_id: 0,
                position: Vector3::new(0., 0., 0.),
                rotation: UnitQuaternion::from_euler_angles(std::f32::consts::FRAC_PI_2, 0., 0.),
                bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 4., 0.5)),
                movement: Movement {
                    baseline_velocity: Vector3::new(0., 0., -1.),
                    arcs_plane_normal: Unit::new_normalize(Vector3::new(1., 0., 0.)),
                    approach_arc_angle: 0.0,
                    approach_arc_center_distance: 0.0,
                    approach_arc_radius: 0.0,
                    approach_rotation_strength: 0.,
                    departure_arc_angle: 0.0,
                    departure_arc_center_distance: 0.0,
                    departure_arc_radius: 0.0,
                    departure_rotation_strength: 0.0,
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
            let obstacle = CollidableEntity {
                movement_start_parameter: 0.0,
                movement_end_parameter: 0.0,
                spawn_position: Vector3::new(0., 0., 2.1),
                spawn_rotation: UnitQuaternion::identity(),
                spawn_feature_shift: nalgebra::zero(),
                spawn_time: 0.0,
                priority: 0,
                prefab: Prefab {
                    position: nalgebra::zero(),
                    rotation: UnitQuaternion::from_euler_angles(std::f32::consts::FRAC_PI_2, 0., 0.),
                    prefab_id: 0,
                    bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 4., 0.5)),
                    movement: Movement {
                        baseline_velocity: Vector3::new(0., 0., -1.),
                        arcs_plane_normal: Unit::new_normalize(Vector3::new(1., 0., 0.)),
                        approach_arc_angle: 0.0,
                        approach_arc_center_distance: 0.0,
                        approach_arc_radius: 0.0,
                        approach_rotation_strength: 0.,
                        departure_arc_angle: 0.0,
                        departure_arc_center_distance: 0.0,
                        departure_arc_radius: 0.0,
                        departure_rotation_strength: 0.0,
                    },
                },
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
                    baseline_velocity: Vector3::new(0., 0., -1.),
                    arcs_plane_normal: Unit::new_normalize(Vector3::new(1., 0., 0.)),
                    approach_arc_angle: 0.0,
                    approach_arc_center_distance: 0.0,
                    approach_arc_radius: 0.0,
                    approach_rotation_strength: 0.,
                    departure_arc_angle: 0.0,
                    departure_arc_center_distance: 0.0,
                    departure_arc_radius: 0.0,
                    departure_rotation_strength: 0.0,
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
            let obstacle = CollidableEntity {
                movement_start_parameter: 0.0,
                movement_end_parameter: 0.0,
                spawn_position: Vector3::new(0., 0., 1.9),
                spawn_rotation: UnitQuaternion::identity(),
                spawn_feature_shift: nalgebra::zero(),
                spawn_time: 0.0,
                priority: 0,
                prefab: Prefab {
                    prefab_id: 0,
                    position: nalgebra::zero(),
                    rotation: UnitQuaternion::from_euler_angles(std::f32::consts::FRAC_PI_2, 0., 0.),
                    bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 4., 0.5)),
                    movement: Movement {
                        baseline_velocity: Vector3::new(0., 0., -1.),
                        arcs_plane_normal: Unit::new_normalize(Vector3::new(1., 0., 0.)),
                        approach_arc_angle: 0.0,
                        approach_arc_center_distance: 0.0,
                        approach_arc_radius: 0.0,
                        approach_rotation_strength: 0.,
                        departure_arc_angle: 0.0,
                        departure_arc_center_distance: 0.0,
                        departure_arc_radius: 0.0,
                        departure_rotation_strength: 0.0,
                    },

                },
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
                    baseline_velocity: Vector3::new(0., 0., -2.),
                    arcs_plane_normal: Unit::new_normalize(Vector3::new(1., 0., 0.)),
                    approach_arc_angle: 0.0,
                    approach_arc_center_distance: 0.0,
                    approach_arc_radius: 0.0,
                    approach_rotation_strength: 0.,
                    departure_arc_angle: 0.0,
                    departure_arc_center_distance: 0.0,
                    departure_arc_radius: 0.0,
                    departure_rotation_strength: 0.0,
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
            let obstacle = CollidableEntity {
                movement_start_parameter: 0.0,
                movement_end_parameter: 0.0,
                spawn_position: Vector3::new(0., 0., -5.),
                spawn_rotation: UnitQuaternion::identity(),
                spawn_feature_shift: nalgebra::zero(),
                spawn_time: 0.0,
                priority: 0,
                prefab: Prefab {
                    prefab_id: 0,
                    position: nalgebra::zero(),
                    rotation: UnitQuaternion::from_euler_angles(std::f32::consts::FRAC_PI_2, 0., 0.),
                    bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 4., 0.5)),
                    movement: Movement {
                        baseline_velocity: Vector3::new(0., 0., -1.),
                        arcs_plane_normal: Unit::new_normalize(Vector3::new(1., 0., 0.)),
                        approach_arc_angle: 0.0,
                        approach_arc_center_distance: 0.0,
                        approach_arc_radius: 0.0,
                        approach_rotation_strength: 0.,
                        departure_arc_angle: 0.0,
                        departure_arc_center_distance: 0.0,
                        departure_arc_radius: 0.0,
                        departure_rotation_strength: 0.0,
                    },

                },
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
        use nalgebra::{UnitQuaternion, Unit};
        use crate::Movement;

        #[test]
        fn test_can_spawn_feature_collision() {
            let prefab = Prefab {
                prefab_id: 0,
                position: Vector3::new(0., 0., 0.),
                rotation: UnitQuaternion::identity(),
                bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
                movement: Movement {
                    baseline_velocity: Vector3::new(0., 0., -1.),
                    arcs_plane_normal: Unit::new_normalize(Vector3::new(1., 0., 0.)),
                    approach_arc_angle: 0.0,
                    approach_arc_center_distance: 0.0,
                    approach_arc_radius: 0.0,
                    approach_rotation_strength: 0.,
                    departure_arc_angle: 0.0,
                    departure_arc_center_distance: 0.0,
                    departure_arc_radius: 0.0,
                    departure_rotation_strength: 0.0,
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
            let obstacle = CollidableEntity {
                movement_start_parameter: -15.5,
                movement_end_parameter: 0.0,
                spawn_position: nalgebra::zero(),
                spawn_rotation: UnitQuaternion::identity(),
                spawn_feature_shift: nalgebra::zero(),
                spawn_time: 5.0,
                priority: 0,
                prefab: Prefab {
                    prefab_id: 0,
                    position: nalgebra::zero(),
                    rotation: UnitQuaternion::identity(),
                    bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
                    movement: Movement {
                        baseline_velocity: Vector3::new(0., -0.5, -0.5),
                        arcs_plane_normal: Unit::new_normalize(Vector3::new(1., 0., 0.)),
                        approach_arc_angle: 0.0,
                        approach_arc_center_distance: 0.0,
                        approach_arc_radius: 0.0,
                        approach_rotation_strength: 0.,
                        departure_arc_angle: 0.0,
                        departure_arc_center_distance: 0.0,
                        departure_arc_radius: 0.0,
                        departure_rotation_strength: 0.0
                    },

                },
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
        use nalgebra::{UnitQuaternion, Unit};
        use crate::Movement;

        #[test]
        fn test_cannot_spawn_if_collide_not_collinear() {
            let prefab = Prefab {
                prefab_id: 0,
                position: Vector3::new(0., 0., 0.),
                rotation: UnitQuaternion::identity(),
                bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
                movement: Movement {
                    baseline_velocity: Vector3::new(0., 0., -1.),
                    arcs_plane_normal: Unit::new_normalize(Vector3::new(1., 0., 0.)),
                    approach_arc_angle: 0.0,
                    approach_arc_center_distance: 0.0,
                    approach_arc_radius: 0.0,
                    approach_rotation_strength: 0.,
                    departure_arc_angle: 0.0,
                    departure_arc_center_distance: 0.0,
                    departure_arc_radius: 0.0,
                    departure_rotation_strength: 0.0
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
            let obstacle = CollidableEntity {
                movement_start_parameter: -100.25,
                movement_end_parameter: 0.0,
                spawn_position: Vector3::new(0., 100., 100.),
                spawn_rotation: UnitQuaternion::identity(),
                spawn_feature_shift: nalgebra::zero(),
                spawn_time: 5.0,
                priority: 0,
                prefab: Prefab {
                    prefab_id: 0,
                    position: nalgebra::zero(),
                    rotation: UnitQuaternion::identity(),
                    bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
                    movement: Movement {
                        baseline_velocity: Vector3::new(0., -1., -1.),
                        arcs_plane_normal: Unit::new_normalize(Vector3::new(1., 0., 0.)),
                        approach_arc_angle: 0.0,
                        approach_arc_center_distance: 0.0,
                        approach_arc_radius: 0.0,
                        approach_rotation_strength: 0.,
                        departure_arc_angle: 0.0,
                        departure_arc_center_distance: 0.0,
                        departure_arc_radius: 0.0,
                        departure_rotation_strength: 0.0
                    },

                },
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
        use nalgebra::{UnitQuaternion, Unit};
        use crate::Movement;

        #[test]
        fn test_cannot_spawn_if_collide_obstacle_tilted() {
            let prefab = Prefab {
                prefab_id: 0,
                position: Vector3::new(0., 0., 0.),
                rotation: UnitQuaternion::identity(),
                bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
                movement: Movement {
                    baseline_velocity: Vector3::new(0., 0., -1.),
                    arcs_plane_normal: Unit::new_normalize(Vector3::new(1., 0., 0.)),
                    approach_arc_angle: 0.0,
                    approach_arc_center_distance: 0.0,
                    approach_arc_radius: 0.0,
                    approach_rotation_strength: 0.,
                    departure_arc_angle: 0.0,
                    departure_arc_center_distance: 0.0,
                    departure_arc_radius: 0.0,
                    departure_rotation_strength: 0.0
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
            let obstacle = CollidableEntity {
                movement_start_parameter: -100.5,
                movement_end_parameter: 0.0,
                spawn_position: nalgebra::zero(),
                spawn_rotation: UnitQuaternion::identity(),
                spawn_feature_shift: nalgebra::zero(),
                spawn_time: 5.0,
                priority: 0,
                prefab: Prefab {
                    prefab_id: 0,
                    position: nalgebra::zero(),
                    rotation: UnitQuaternion::identity(),
                    bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
                    movement: Movement {
                        baseline_velocity: Vector3::new(0., 0., -1.),
                        arcs_plane_normal: Unit::new_normalize(Vector3::new(1., 0., 0.)),
                        approach_arc_angle: 45.0_f32.to_radians(),
                        approach_arc_center_distance: 0.0,
                        approach_arc_radius: 0.0,
                        approach_rotation_strength: 0.,
                        departure_arc_angle: 0.0,
                        departure_arc_center_distance: 0.0,
                        departure_arc_radius: 0.0,
                        departure_rotation_strength: 0.0
                    },
                },
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
                    baseline_velocity: Vector3::new(0., 0., -1.),
                    arcs_plane_normal: Unit::new_normalize(Vector3::new(1., 0., 0.)),
                    approach_arc_angle: 45.0_f32.to_radians(),
                    approach_arc_center_distance: 0.0,
                    approach_arc_radius: 10.0,
                    approach_rotation_strength: 0.,
                    departure_arc_angle: 0.0,
                    departure_arc_center_distance: 0.0,
                    departure_arc_radius: 0.0,
                    departure_rotation_strength: 0.0
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
            let obstacle = CollidableEntity {
                movement_start_parameter: -100.5,
                movement_end_parameter: 0.0,
                spawn_position: Vector3::new(0., 0., 100.),
                spawn_rotation: UnitQuaternion::identity(),
                spawn_feature_shift: nalgebra::zero(),
                spawn_time: 5.0,
                priority: 0,
                prefab: Prefab {
                    prefab_id: 0,
                    position: nalgebra::zero(),
                    rotation: UnitQuaternion::identity(),
                    bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
                    movement: Movement {
                        baseline_velocity: Vector3::new(0., 0., -1.),
                        arcs_plane_normal: Unit::new_normalize(Vector3::new(1., 0., 0.)),
                        approach_arc_angle: 0.0,
                        approach_arc_center_distance: 0.0,
                        approach_arc_radius: 0.0,
                        approach_rotation_strength: 0.,
                        departure_arc_angle: 0.0,
                        departure_arc_center_distance: 0.0,
                        departure_arc_radius: 0.0,
                        departure_rotation_strength: 0.0
                    },

                },
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
                    baseline_velocity: Vector3::new(0., 0., -1.),
                    arcs_plane_normal: Unit::new_normalize(Vector3::new(1., 0., 0.)),
                    approach_arc_angle: 45.0_f32.to_radians(),
                    approach_arc_center_distance: 0.0,
                    approach_arc_radius: 0.0,
                    approach_rotation_strength: 0.,
                    departure_arc_angle: 0.0,
                    departure_arc_center_distance: 0.0,
                    departure_arc_radius: 0.0,
                    departure_rotation_strength: 0.0
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
            let obstacle = CollidableEntity {
                movement_start_parameter: -100.25,
                movement_end_parameter: 0.0,
                spawn_position: Vector3::new(0., 100., 100.),
                spawn_rotation: UnitQuaternion::identity(),
                spawn_feature_shift: nalgebra::zero(),
                spawn_time: 5.0,
                priority: 0,
                prefab: Prefab {
                    prefab_id: 0,
                    position: nalgebra::zero(),
                    rotation: UnitQuaternion::identity(),
                    bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
                    movement: Movement {
                        baseline_velocity: Vector3::new(0., 0., -1.),
                        arcs_plane_normal: Unit::new_normalize(Vector3::new(1., 0., 0.)),
                        approach_arc_angle: 45.0_f32.to_radians(),
                        approach_arc_center_distance: 0.0,
                        approach_arc_radius: 0.0,
                        approach_rotation_strength: 0.,
                        departure_arc_angle: 0.0,
                        departure_arc_center_distance: 0.0,
                        departure_arc_radius: 0.0,
                        departure_rotation_strength: 0.0
                    },

                },
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