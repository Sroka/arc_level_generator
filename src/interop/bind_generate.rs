use nalgebra::{Point3, UnitQuaternion, Quaternion};
use crate::interop::types::{VisibleWorldDescription, FeatureDescription, EntitiesArrayDescription, EntityDescription, MovementDescription};
use std::slice::from_raw_parts;
use crate::{Feature, Prefab, VisibleWorld};
use crate::generate;
use ncollide3d::bounding_volume::AABB;
use rand::thread_rng;
use std::mem;
use crate::generator::Movement;

/// Unsafe wrapper around #generate() function. It is a callers responsibility to call
/// #bind_deallocate_vec ona returned array. Otherwise this array will never be deallocated and
/// will leak memory
#[no_mangle]
pub unsafe extern fn bind_generate(
    features_ptr: *const FeatureDescription,
    features_count: i32,
    world_description: VisibleWorldDescription,
) -> EntitiesArrayDescription {
    // let features: Vec<Feature> = from_raw_parts(features_ptr, features_count as usize)
    //     .iter()
    //     .map(|feature_description| {
    //         let feature_prefabs: Vec<Prefab> = from_raw_parts(feature_description.prefabs, feature_description.prefabs_count as usize)
    //             .iter()
    //             .map(|prefab_description| {
    //                 Prefab {
    //                     prefab_id: prefab_description.prefab_id,
    //                     position: prefab_description.position,
    //                     rotation: UnitQuaternion::from_quaternion(Quaternion::from(prefab_description.rotation)),
    //                     bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), prefab_description.half_extents),
    //                     movement: Movement {
    //                         linear_velocity: prefab_description.movement.linear_velocity,
    //                         arcs_plane_normal: prefab_description.movement.z_axis_tilt_xy_direction,
    //                         approach_arc_angle: prefab_description.movement.z_axis_tilt_angle,
    //                         approach_arc_center_distance: prefab_description.movement.z_axis_tilt_distance,
    //                         approach_arc_radius: prefab_description.movement.z_axis_tilt_easing_range,
    //                         approach_rotation_strength: prefab_description.movement.z_axis_tilt_rotation_strength,
    //                     },
    //                 }
    //             })
    //             .collect();
    //
    //         Feature {
    //             prefabs: feature_prefabs,
    //             spawn_period: feature_description.spawn_period,
    //             is_spawn_period_strict: feature_description.is_spawn_period_strict,
    //             spawn_count: feature_description.spawn_count,
    //             trigger_time: feature_description.trigger_time,
    //             priority: feature_description.priority,
    //             translate_x: feature_description.translate_x,
    //             translate_x_using_bounds: feature_description.translate_x_using_bounds,
    //             translate_x_bounds: feature_description.translate_x_bounds.clone(),
    //             translate_y: feature_description.translate_y,
    //             translate_y_using_bounds: feature_description.translate_y_using_bounds,
    //             translate_y_bounds: feature_description.translate_y_bounds.clone(),
    //             translate_z: feature_description.translate_z,
    //             missed_spawns: 0,
    //             last_spawn_attempt: 0.0,
    //         }
    //     })
    //     .collect();


    let features: Vec<Feature> = vec![];
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

    let mut entities_descriptions: Vec<EntityDescription> = vec![];
    // let mut entities_descriptions: Vec<EntityDescription> = generated_entities.iter().map(|entity| {
    //     EntityDescription {
    //         spawn_position: entity.spawn_position,
    //         spawn_rotation: entity.spawn_rotation.coords,
    //         spawn_time: entity.spawn_time,
    //         movement: MovementDescription {
    //             linear_velocity: entity.movement.linear_velocity,
    //             z_axis_tilt_xy_direction: entity.movement.arcs_plane_normal,
    //             z_axis_tilt_angle: entity.movement.approach_arc_angle,
    //             z_axis_tilt_distance: entity.movement.approach_arc_center_distance,
    //             z_axis_tilt_easing_range: entity.movement.approach_arc_radius,
    //             z_axis_tilt_rotation_strength: entity.movement.approach_rotation_strength,
    //         },
    //         prefab_id: entity.prefab_id,
    //     }
    // }).collect();
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