use std::collections::VecDeque;
use crate::generator::types::{Feature, CollideableEntity, VisibleWorld};
use nalgebra::{Vector3, Isometry3, Translation3};
use std::cmp::Ordering::Equal;
use itertools::Itertools;
use crate::generator::tilt_motion::ConstantVelocityZTiltMotion;
use ncollide3d::interpolation::RigidMotion;

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
    let min_z_velocity_in_a_feature = feature.prefabs
        .iter()
        .map(|prefab| prefab.movement.linear_velocity.z)
        .sorted_by(|a, b| { a.partial_cmp(b).unwrap_or(Equal) })
        .last()
        .clone()
        .unwrap();
    let time_to_travel_to_origin_plane_from_worlds_start = (world.world_bounds.maxs().z + feature_shift.z) / -min_z_velocity_in_a_feature;
    for prefab in &feature.prefabs {
        let prefab_motion = ConstantVelocityZTiltMotion::new(
            time_to_travel_to_origin_plane_from_worlds_start,
            Isometry3::from_parts(Translation3::from(prefab.position + Vector3::new(feature_shift.x, feature_shift.y, 0.)), prefab.rotation),
            prefab.movement.linear_velocity.clone(),
            prefab.movement.z_axis_tilt_xy_direction.clone(),
            prefab.movement.z_axis_tilt_angle,
            prefab.movement.z_axis_tilt_distance,
            prefab.movement.z_axis_tilt_easing_range,
        );
        let entity = CollideableEntity {
            spawn_position: prefab_motion.position_at_time(0.).translation.vector,
            spawn_time: time + feature.priority as f32,
            movement: prefab.movement.clone(),
            rotation: prefab.rotation,
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
    use crate::{Feature, Prefab, Movement, VisibleWorld};
    use nalgebra::{Vector2, Vector3, Point3, UnitQuaternion};
    use crate::generator::spawn_feature::spawn_feature;
    use std::collections::VecDeque;
    use ncollide3d::bounding_volume::AABB;

    #[test]
    fn test_spawn_feature() {
        let prefab = Prefab {
            prefab_id: 1,
            position: nalgebra::zero(),
            rotation: UnitQuaternion::identity(),
            bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
            movement: Movement {
                linear_velocity: Vector3::new(0., 0., -8.),
                z_axis_tilt_xy_direction: Vector2::new(0., 1.),
                z_axis_tilt_angle: 0.0,
                z_axis_tilt_distance: 0.0,
                z_axis_tilt_easing_range: 50.0,
            },
        };
        let feature = Feature {
            prefabs: vec![prefab],
            spawn_period: 0.0,
            is_spawn_period_strict: false,
            spawn_count: 5,
            trigger_time: 0.0,
            priority: 0,
            translate_x: false,
            translate_x_using_bounds: false,
            translate_x_bounds: nalgebra::zero(),
            translate_y: false,
            translate_y_using_bounds: false,
            translate_y_bounds: nalgebra::zero(),
            translate_z: 0.0,
            missed_spawns: 0,
            last_spawn_attempt: 0.0,
        };
        let world = VisibleWorld {
            world_bounds: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(100., 100., 100.)),
        };
        let generated_entities = &mut vec![];
        spawn_feature(
            &feature,
            &mut VecDeque::new(),
            generated_entities,
            0.,
            &world,
            &nalgebra::zero(),
        );

        for (index, entity) in generated_entities.iter().enumerate() {
            println!("Generated entitity {}: {:?}", index, entity)
        }
    }

    #[test]
    fn test_spawn_feature_tiled() {
        let prefab = Prefab {
            prefab_id: 1,
            position: nalgebra::zero(),
            rotation: UnitQuaternion::identity(),
            bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
            movement: Movement {
                linear_velocity: Vector3::new(0., 0., -8.),
                z_axis_tilt_xy_direction: Vector2::new(0., 1.),
                z_axis_tilt_angle: 45.0,
                z_axis_tilt_distance: 0.0,
                z_axis_tilt_easing_range: 50.0,
            },
        };
        let feature = Feature {
            prefabs: vec![prefab],
            spawn_period: 0.0,
            is_spawn_period_strict: false,
            spawn_count: 5,
            trigger_time: 0.0,
            priority: 0,
            translate_x: false,
            translate_x_using_bounds: false,
            translate_x_bounds: nalgebra::zero(),
            translate_y: false,
            translate_y_using_bounds: false,
            translate_y_bounds: nalgebra::zero(),
            translate_z: 0.0,
            missed_spawns: 0,
            last_spawn_attempt: 0.0,
        };
        let world = VisibleWorld {
            world_bounds: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(100., 100., 100.)),
        };
        let generated_entities = &mut vec![];
        spawn_feature(
            &feature,
            &mut VecDeque::new(),
            generated_entities,
            0.,
            &world,
            &nalgebra::zero(),
        );
        for (index, entity) in generated_entities.iter().enumerate() {
            println!("Generated entitity {}: {:?}", index, entity)
        }
    }
}
