use super::types::{CollideableEntity, VisibleWorld};
use std::collections::VecDeque;
use ncollide3d::shape::Cuboid;
use ncollide3d::query;
use ncollide3d::query::{RayCast, Ray};
use nalgebra::{Isometry3, Vector3, Point3};
use crate::generator::types::Feature;
use ncollide3d::bounding_volume::BoundingVolume;

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
    'prefabs_loop: for prefab in &feature.prefabs {
        'obstacles_loop: for obstacle in obstacles {
            let time_to_travel_to_origin_plane_from_worlds_start = world.world_bounds.maxs().y / -prefab.velocity.y;
            let prefab_spawn_position = //Vector3::new(0., world.world_bounds.maxs().y, 0.)
                prefab.position
                - prefab.velocity * time_to_travel_to_origin_plane_from_worlds_start
                - prefab.velocity * (feature.priority as f32)
                + feature_shift;
            let obstacle_spawn_position = obstacle.spawn_position
                + obstacle.velocity * (time_travelled - obstacle.spawn_time);

            let prefab_world_bounds_toi = world.world_bounds
                // This is required because sometimes, due to priority, prefab spawn position is outside world bounds
                .merged(&prefab.bounding_box.transform_by(&Isometry3::new(prefab_spawn_position, nalgebra::zero())))
                .toi_with_ray(
                    &Isometry3::new(nalgebra::zero(), nalgebra::zero()),
                    &Ray::new(Point3::origin() + prefab_spawn_position, prefab.velocity),
                    false,
                ).unwrap()
                +
                prefab.bounding_box
                    .toi_with_ray(
                        &Isometry3::new(nalgebra::zero(), nalgebra::zero()),
                        &Ray::new(Point3::origin(), -prefab.velocity),
                        false,
                    ).unwrap();

//            dbg!("CAN SPAWN CHECK");
//            dbg!(&prefab.prefab_id);
//            dbg!(&obstacle.prefab_id);
//            dbg!(&prefab_spawn_position);
//            dbg!(&obstacle_spawn_position);
            // TODO In reality this is a fix for a bug in query::time_of_impact
            if prefab.bounding_box.transform_by(&Isometry3::new(prefab_spawn_position, nalgebra::zero()))
                .intersects(&obstacle.bounding_box.transform_by(&Isometry3::new(obstacle_spawn_position, nalgebra::zero()))) {
//                dbg!("CANNOT INTERSECTS");
                return false;
            }

            let time_of_impact = query::time_of_impact(
                &Isometry3::new(prefab_spawn_position, nalgebra::zero()),
                &prefab.velocity,
                &Cuboid::new(prefab.bounding_box.half_extents()),
                &Isometry3::new(obstacle_spawn_position, nalgebra::zero()),
                &obstacle.velocity,
                &Cuboid::new(obstacle.bounding_box.half_extents()),
                prefab_world_bounds_toi,
                0.0,
            );
//            dbg!(&time_of_impact);
            match time_of_impact {
                Some(_) => {
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
    use nalgebra::{Vector3, Point3};
    use crate::generator::can_spawn_feature::can_spawn_feature;
    use std::collections::VecDeque;
    use std::iter::FromIterator;

    mod zero_travel_time {
        use super::*;

        #[test]
        fn test_can_spawn_feature_collision() {
            let prefab = Prefab {
                prefab_id: 0,
                position: Vector3::new(0., 0., 0.),
                bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
                velocity: Vector3::new(0., -1., 0.),
            };
            let feature = Feature {
                translate_x: false,
                translate_x_out_of_bounds: false,
                translate_z: false,
                translate_z_out_of_bounds: false,
                prefabs: vec![prefab],
                spawn_count: 1,
                spawns_per_second: 1.,
                trigger_position: 10.,
                priority: 0,
                missed_spawns: 0,
            };
            let obstacle = CollideableEntity {
                spawn_position: Vector3::new(0., 10., 5.),
                prefab_id: 0,
                bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
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
                bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
                velocity: Vector3::new(0., -1., 0.),
            };
            let feature = Feature {
                translate_x: false,
                translate_x_out_of_bounds: false,
                translate_z: false,
                translate_z_out_of_bounds: false,
                prefabs: vec![prefab],
                spawn_count: 1,
                spawns_per_second: 1.,
                trigger_position: 10.,
                priority: 0,
                missed_spawns: 0,
            };
            let obstacle = CollideableEntity {
                spawn_position: Vector3::new(0., -1.25, 0.),
                prefab_id: 0,
                bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
                velocity: Vector3::new(0., -0.5, -0.),
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
                bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
                velocity: Vector3::new(0., -1., 0.),
            };
            let feature = Feature {
                translate_x: false,
                translate_x_out_of_bounds: false,
                translate_z: false,
                translate_z_out_of_bounds: false,
                prefabs: vec![prefab],
                spawn_count: 1,
                spawns_per_second: 1.,
                trigger_position: 10.,
                priority: 0,
                missed_spawns: 0,
            };
            let obstacle = CollideableEntity {
                spawn_position: Vector3::new(0., -2.5, 0.),
                prefab_id: 0,
                bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
                velocity: Vector3::new(0., -0.5, -0.),
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
                bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
                velocity: Vector3::new(0., -1., 0.),
            };
            let feature = Feature {
                translate_x: false,
                translate_x_out_of_bounds: false,
                translate_z: false,
                translate_z_out_of_bounds: false,
                prefabs: vec![prefab],
                spawn_count: 1,
                spawns_per_second: 1.,
                trigger_position: 10.,
                priority: 0,
                missed_spawns: 0,
            };
            let obstacle = CollideableEntity {
                spawn_position: Vector3::new(0., 10.0, 0.),
                prefab_id: 0,
                bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
                velocity: Vector3::new(0., -1.0, -0.),
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
    }

    mod non_zero_travel_time_with_shift {
        use super::*;

        #[test]
        fn test_can_spawn_feature_collision() {
            let prefab = Prefab {
                prefab_id: 0,
                position: Vector3::new(0., 0., 0.),
                bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
                velocity: Vector3::new(0., -1., 0.),
            };
            let feature = Feature {
                translate_x: false,
                translate_x_out_of_bounds: false,
                translate_z: false,
                translate_z_out_of_bounds: false,
                prefabs: vec![prefab],
                spawn_count: 1,
                spawns_per_second: 1.,
                trigger_position: 10.,
                priority: 5,
                missed_spawns: 0,
            };
            let obstacle = CollideableEntity {
                spawn_position: Vector3::new(0., 8., 5.),
                prefab_id: 0,
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