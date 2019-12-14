use std::collections::VecDeque;
use crate::generate::types::{Feature, CollideableEntity};

pub fn spawn_feature(feature: &Feature,
                     obstacles: &mut VecDeque<CollideableEntity>,
                     generated_entities: &mut Vec<CollideableEntity>,
                     time: f32,
) {
    for prefab in feature.prefabs {
        let entity = CollideableEntity {
            spawn_position: prefab.position,
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
