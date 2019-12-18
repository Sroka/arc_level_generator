use nalgebra::{Vector3, Point3};
use crate::interop::types::{VisibleWorldDescription, FeatureDescription, PrefabDescription, EntitiesArrayDescription, EntityDescription};
use std::slice::from_raw_parts;
use crate::{Feature, Prefab, VisibleWorld};
use crate::generate;
use ncollide3d::bounding_volume::AABB;
use std::iter::FromIterator;
use rand::thread_rng;
use std::mem;

#[no_mangle]
pub unsafe extern fn bind_generate(
    features_ptr: *const FeatureDescription,
    features_count: i32,
    prefabs_ptr: *const PrefabDescription,
    prefabs_count: i32,
    world_description: VisibleWorldDescription,
) -> EntitiesArrayDescription {
    let prefabs: Vec<Prefab> = from_raw_parts(prefabs_ptr, prefabs_count as usize)
        .iter()
        .map(|prefab_description| Prefab {
            prefab_id: prefab_description.prefab_id,
            position: prefab_description.position,
            bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), prefab_description.half_extents),
            velocity: prefab_description.velocity,
        })
        .collect();
    let features: Vec<Feature> = from_raw_parts(features_ptr, features_count as usize)
        .iter()
        .map(|feature_description| {
            let prefab_ids = from_raw_parts(feature_description.prefabs_ids, feature_description.prefabs_ids_count as usize);
            let feature_prefabs: Vec<Prefab> = prefab_ids.iter()
                .map(|prefab_id| {
                    let option = prefabs.iter().find(|prefab| prefab.prefab_id == *prefab_id);
                    match option {
                        None => panic!("No prefab havings a prefab id"),
                        Some(existing_prefab) => existing_prefab.clone(),
                    }
                })
                .collect();
            Feature {
                prefabs: feature_prefabs,
                spawns_per_second: feature_description.spawns_per_second,
                spawn_count: feature_description.spawn_count,
                trigger_position: feature_description.trigger_position,
                priority: feature_description.priority,
                translate_x: feature_description.translate_x,
                translate_z: feature_description.translate_z,
                missed_spawns: 0,
            }
        })
        .collect();


    let world = VisibleWorld {
        world_bounds: AABB::from_half_extents(Point3::new(
            world_description.position.x,
            world_description.position.y,
            world_description.position.z,
        ), world_description.half_extents),
        travel_speed: world_description.travel_speed,
        spawn_barrier_y_coord: world_description.spawn_barrier_y_coord,
    };


    let generated_entities = generate(
        &world,
        features.as_slice(),
        &mut thread_rng(),
    );

    let mut entities_descriptions: Vec<EntityDescription> = generated_entities.iter().map(|entity| EntityDescription {
        spawn_position: entity.spawn_position,
        spawn_time: entity.spawn_time,
        velocity: entity.velocity,
        prefab_id: entity.prefab_id,
    }).collect();
    entities_descriptions.shrink_to_fit();
    assert_eq!(entities_descriptions.capacity(), entities_descriptions.len());
    let pointer = entities_descriptions.as_mut_ptr();
    let length = entities_descriptions.len() as i32;
    mem::forget(entities_descriptions);
    EntitiesArrayDescription {
        pointer,
        length,
    }
}