use super::types::{CollideableEntity, Prefab, VisibleWorld};
use std::collections::VecDeque;
use ncollide3d::shape::Cuboid;
use ncollide3d::query;
use nalgebra::{Isometry3};

pub fn can_spawn_feature(
    feature_prefabs: &[Prefab],
    obstacles: &VecDeque<CollideableEntity>,
    world: &VisibleWorld,
    time_travelled: f32,
) -> bool {
    'prefabs_loop: for prefab in feature_prefabs {
        'obstacles_loop: for existing_entity in obstacles {
            let time_of_impact = query::time_of_impact(
                &Isometry3::new(prefab.position, nalgebra::zero()),
                &prefab.velocity,
                &Cuboid::new(prefab.bounding_box.half_extents()),
                &Isometry3::new(existing_entity.spawn_position, nalgebra::zero()),
                &existing_entity.velocity,
                &Cuboid::new(existing_entity.bounding_box.half_extents()),
                100.0,
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