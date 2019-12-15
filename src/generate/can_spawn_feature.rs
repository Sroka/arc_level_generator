use super::types::{CollideableEntity, Prefab, VisibleWorld};
use std::collections::VecDeque;
use ncollide3d::shape::Cuboid;
use ncollide3d::query;
use ncollide3d::query::{RayCast, Ray};
use nalgebra::{Isometry3, Vector3, Point3};
use crate::generate::types::Feature;

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
                + prefab.velocity * feature.priority as f32
                + feature_shift;
            let obstacle_spawn_position = obstacle.spawn_position
                + obstacle.velocity * obstacle.priority as f32;

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
    #[test]
    fn test_can_spawn_feature() {
        assert!(false)
    }
}