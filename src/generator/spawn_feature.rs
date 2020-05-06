use std::collections::VecDeque;
use crate::generator::types::{Feature, CollideableEntity, VisibleWorld};
use nalgebra::Vector3;

/// Spawns entities belonging to a feature at a given time of travel in a given world
/// * `feature` - feature to spawn
/// * `obstacles` - a list of entities that all s entities spawned from a feature will be added to.
///                 They will now have to be check against collisions until they get trimmed from
///                 the list after leaving the visible world
/// * `generated_entities` -  a list of all spawned entities. Entities spawned from this feature
///                           will be added to it
/// * `time` - current time travel
/// * `world` - visible world
/// * `feature_shift` - a randomized positional shift of this feature. All feature entities will
///                     be spawned shifted by this value
///
pub fn spawn_feature(feature: &Feature,
                     obstacles: &mut VecDeque<CollideableEntity>,
                     generated_entities: &mut Vec<CollideableEntity>,
                     time: f32,
                     world: &VisibleWorld,
                     feature_shift: &Vector3<f32>,
) {
    for prefab in &feature.prefabs {
        let time_to_travel_to_origin_plane_from_worlds_start = world.world_bounds.maxs().y / -prefab.velocity.y;
        let entity = CollideableEntity {
            spawn_position: prefab.position + feature_shift - prefab.velocity * time_to_travel_to_origin_plane_from_worlds_start,
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
