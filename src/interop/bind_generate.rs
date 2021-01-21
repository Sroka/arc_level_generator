use nalgebra::{Point3};
use crate::interop::types::{VisibleWorldDescription, FeatureDescription, EntitiesArrayDescription, EntityDescription};
use std::slice::from_raw_parts;
use crate::{Feature, Prefab, VisibleWorld};
use crate::generate;
use ncollide3d::bounding_volume::AABB;
use rand::thread_rng;
use std::mem;

/// Unsafe wrapper around #generate() function. It is a callers responsibility to call
/// #bind_deallocate_vec ona returned array. Otherwise this array will never be deallocated and
/// will leak memory
#[no_mangle]
pub unsafe extern fn bind_generate(
    features_ptr: *const FeatureDescription,
    features_count: i32,
    world_description: VisibleWorldDescription,
) -> EntitiesArrayDescription {
    let features: Vec<Feature> = from_raw_parts(features_ptr, features_count as usize)
        .iter()
        .map(|feature_description| {
            let feature_prefabs: Vec<Prefab> = from_raw_parts(feature_description.prefabs, feature_description.prefabs_count as usize)
                .iter()
                .map(|prefab_description| prefab_description.clone().into())
                .collect();

            Feature {
                prefabs: feature_prefabs,
                spawn_period: feature_description.spawn_period,
                is_spawn_period_strict: feature_description.is_spawn_period_strict,
                spawn_count: feature_description.spawn_count,
                trigger_time: feature_description.trigger_time,
                priority: feature_description.priority,
                translate_x: feature_description.translate_x,
                translate_x_using_bounds: feature_description.translate_x_using_bounds,
                translate_x_bounds: feature_description.translate_x_bounds.clone(),
                translate_y: feature_description.translate_y,
                translate_y_using_bounds: feature_description.translate_y_using_bounds,
                translate_y_bounds: feature_description.translate_y_bounds.clone(),
                translate_z: feature_description.translate_z,
                missed_spawns: 0,
                last_spawn_attempt: 0.0,
            }
        })
        .collect();


    let world = VisibleWorld {
        world_bounds: AABB::from_half_extents(Point3::new(
            world_description.position.x,
            world_description.position.y,
            world_description.position.z,
        ), world_description.half_extents),
    };

    dbg!(&world);
    dbg!(&features);

    let generated_entities = generate(
        &world,
        features.as_slice(),
        &mut thread_rng(),
    );

    let mut entities_descriptions: Vec<EntityDescription> = generated_entities.iter().map(|entity| entity.clone().into()).collect();
    entities_descriptions.shrink_to_fit();
    dbg!(&entities_descriptions);
    assert_eq!(entities_descriptions.capacity(), entities_descriptions.len());
    let pointer = entities_descriptions.as_mut_ptr();
    let length = entities_descriptions.len() as i32;
    mem::forget(entities_descriptions);
    EntitiesArrayDescription {
        pointer,
        length,
    }
}