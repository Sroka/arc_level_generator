use super::types::{CollideableEntity, Prefab, VisibleWorld};
use std::collections::VecDeque;
use ncollide3d::shape::Cuboid;
use ncollide3d::query;
use ncollide3d::query::{RayCast, Ray};
use nalgebra::{Isometry3, Vector3, Point3};
use crate::generate::types::Feature;
use ncollide3d::bounding_volume::BoundingVolume;

pub fn can_spawn_feature(
    feature: &Feature,
    obstacles: &VecDeque<CollideableEntity>,
    world: &VisibleWorld,
    time_travelled: f32,
    feature_shift: &Vector3<f32>,
) -> bool {
    'prefabs_loop: for prefab in feature.prefabs {
        'obstacles_loop: for obstacle in obstacles {
            let prefab_spawn_position = Vector3::new(0., world.spawn_barrier, 0.)
                + prefab.position
                + prefab.velocity * (-feature.priority as f32)
                + feature_shift;
            let obstacle_spawn_position = obstacle.spawn_position
                + obstacle.velocity * (time_travelled - obstacle.spawn_time);

            let prefab_world_bounds_toi = world.world_bounds
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
            println!("Prefab spawn position: {:?}", prefab_spawn_position);
            println!("Obstacle spawn position: {:?}", obstacle_spawn_position);
            println!("Prefab toi: {}", prefab_world_bounds_toi);
            // TODO In reality this is a fix for a bug in query::time_of_impact
            if prefab.bounding_box.transform_by(&Isometry3::new(prefab_spawn_position, nalgebra::zero()))
                .intersects(&obstacle.bounding_box.transform_by(&Isometry3::new(obstacle_spawn_position, nalgebra::zero()))) {
                return false
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
    use crate::generate::types::{Prefab, Feature, VisibleWorld, CollideableEntity};
    use ncollide3d::bounding_volume::AABB;
    use nalgebra::{Isometry3, Vector3, Point3};
    use crate::generate::can_spawn_feature::can_spawn_feature;
    use std::collections::VecDeque;
    use std::iter::FromIterator;

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
            translate_z: false,
            prefabs: &[prefab],
            spawn_count: 1,
            spawns_per_second: 1.,
            trigger_position: 10.,
            priority: 0,
        };
        let obstacle = CollideableEntity {
            spawn_position: Vector3::new(0., 8., 5.),
            prefab_id: 0,
            bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
            velocity: Vector3::new(0., -1., -1.),
            spawn_time: 0.0,
            priority: 0,
        };
        let world = VisibleWorld {
            world_bounds: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(10., 10., 10.)),
            travel_speed: 4.0,
            spawn_barrier: 8.0,
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
            translate_z: false,
            prefabs: &[prefab],
            spawn_count: 1,
            spawns_per_second: 1.,
            trigger_position: 10.,
            priority: 0,
        };
        let obstacle = CollideableEntity {
            spawn_position: Vector3::new(0., -2.25, 0.),
            prefab_id: 0,
            bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
            velocity: Vector3::new(0., -0.5, -0.),
            spawn_time: 0.0,
            priority: 0,
        };
        let world = VisibleWorld {
            world_bounds: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(10., 10., 10.)),
            travel_speed: 4.0,
            spawn_barrier: 8.0,
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
            translate_z: false,
            prefabs: &[prefab],
            spawn_count: 1,
            spawns_per_second: 1.,
            trigger_position: 10.,
            priority: 0,
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
            spawn_barrier: 8.0,
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
            translate_z: false,
            prefabs: &[prefab],
            spawn_count: 1,
            spawns_per_second: 1.,
            trigger_position: 10.,
            priority: 0,
        };
        let obstacle = CollideableEntity {
            spawn_position: Vector3::new(0., 8.0, 0.),
            prefab_id: 0,
            bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
            velocity: Vector3::new(0., -1.0, -0.),
            spawn_time: 0.0,
            priority: 0,
        };
        let world = VisibleWorld {
            world_bounds: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(10., 10., 10.)),
            travel_speed: 4.0,
            spawn_barrier: 8.0,
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