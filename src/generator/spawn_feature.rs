use std::collections::VecDeque;
use crate::generator::types::{Feature, CollideableEntity, VisibleWorld};
use nalgebra::Vector3;

pub fn spawn_feature(feature: &Feature,
                     obstacles: &mut VecDeque<CollideableEntity>,
                     generated_entities: &mut Vec<CollideableEntity>,
                     time: f32,
                     world: &VisibleWorld,
                     feature_shift: &Vector3<f32>,
) {
    for prefab in feature.prefabs {
        let entity = CollideableEntity {
            spawn_position: prefab.position + feature_shift + Vector3::new(0., world.spawn_barrier_y_coord, 0.),
            spawn_time: time + feature.priority as f32,
            velocity: prefab.velocity,
            bounding_box: prefab.bounding_box.clone(),
            prefab_id: prefab.prefab_id,
            priority: feature.priority,
        };
        obstacles.push_back(entity.clone());
        generated_entities.push(entity.clone());
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn test_spawn_feature() {
        assert!(false)
    }
}
