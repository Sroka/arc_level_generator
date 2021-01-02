use std::collections::VecDeque;
use crate::generator::types::{Feature, CollideableEntity, VisibleWorld};
use nalgebra::{Vector3, Isometry3, Translation3, UnitQuaternion, Unit};
use std::cmp::Ordering::Equal;
use itertools::Itertools;
use crate::generator::tilt_motion::ConstantVelocityZTiltMotion;
use ncollide3d::interpolation::RigidMotion;
use crate::Prefab;

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
    let time_to_travel_to_origin_plane_from_worlds_start = (world.world_bounds.maxs.z + feature_shift.z) / -min_z_velocity_in_a_feature;
    for prefab in &feature.prefabs {
        let prefab_motion = ConstantVelocityZTiltMotion::new(
            time_to_travel_to_origin_plane_from_worlds_start,
            Isometry3::from_parts(Translation3::from(prefab.position + Vector3::new(feature_shift.x, feature_shift.y, 0.)), prefab.rotation),
            prefab.movement.linear_velocity.clone(),
            prefab.movement.z_axis_tilt_xy_direction.clone(),
            prefab.movement.z_axis_tilt_angle,
            prefab.movement.z_axis_tilt_distance,
            prefab.movement.z_axis_tilt_easing_range,
            prefab.movement.z_axis_tilt_rotation_strength,
        );
        let entity = CollideableEntity {
            prefab_id: prefab.prefab_id,
            spawn_position: prefab_motion.position_at_time(0.).translation.vector,
            spawn_rotation: rotation_with_tilt(&prefab),
            spawn_rotation_without_tilt: prefab.rotation,
            spawn_time: time + feature.priority as f32,
            bounding_box: prefab.bounding_box.clone(),
            movement: prefab.movement.clone(),
            priority: feature.priority,
        };
        obstacles.push_back(entity.clone());
        generated_entities.push(entity.clone());
    }
}

// TODO This won't be needed with toi rotation prediction algo in ncollide
fn rotation_with_tilt(prefab: &Prefab) -> UnitQuaternion<f32> {
    if prefab.movement.z_axis_tilt_angle.is_normal() &&
        prefab.movement.z_axis_tilt_rotation_strength.is_normal() &&
        prefab.movement.z_axis_tilt_xy_direction.len() > f32::EPSILON as usize {
        let rotation_axis = Unit::new_normalize(Vector3::new(prefab.movement.z_axis_tilt_xy_direction.x, prefab.movement.z_axis_tilt_xy_direction.y, 0.).cross(&Vector3::z_axis()));
        let angle = -prefab.movement.z_axis_tilt_angle.to_radians() * prefab.movement.z_axis_tilt_rotation_strength;
        return UnitQuaternion::from_axis_angle(&rotation_axis, angle) * prefab.rotation;
    }
    return prefab.rotation;
}

#[cfg(test)]
mod tests {
    use crate::{Feature, Prefab, Movement, VisibleWorld};
    use nalgebra::{Vector2, Vector3, Point3, UnitQuaternion};
    use crate::generator::spawn_feature::{spawn_feature, rotation_with_tilt};
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
                z_axis_tilt_rotation_strength: 0.,
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
                z_axis_tilt_rotation_strength: 0.,
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
    fn test_rotation_with_tilt() {
        let prefab = Prefab {
            prefab_id: 1,
            position: nalgebra::zero(),
            rotation: UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f32::consts::FRAC_PI_4),
            bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
            movement: Movement {
                linear_velocity: Vector3::new(0., 0., -8.),
                z_axis_tilt_xy_direction: Vector2::new(0., 1.),
                z_axis_tilt_angle: 45.0,
                z_axis_tilt_distance: 0.0,
                z_axis_tilt_easing_range: 50.0,
                z_axis_tilt_rotation_strength: 1.,
            },
        };
        let angle = rotation_with_tilt(&prefab);
        let euler_angles = angle.euler_angles();
        assert!((euler_angles.0.to_degrees() - 90.) < 0.01);
        assert!((euler_angles.1.to_degrees() - 0.) < 0.01);
        assert!((euler_angles.2.to_degrees() - 0.) < 0.01);
    }

    #[test]
    fn test_rotation_with_tilt_orthogonal() {
        let prefab = Prefab {
            prefab_id: 1,
            position: nalgebra::zero(),
            rotation: UnitQuaternion::from_axis_angle(&Vector3::y_axis(), std::f32::consts::FRAC_PI_4),
            bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
            movement: Movement {
                linear_velocity: Vector3::new(0., 0., -8.),
                z_axis_tilt_xy_direction: Vector2::new(0., 1.),
                z_axis_tilt_angle: 45.0,
                z_axis_tilt_distance: 0.0,
                z_axis_tilt_easing_range: 50.0,
                z_axis_tilt_rotation_strength: 1.,
            },
        };
        let angle = rotation_with_tilt(&prefab);
        let euler_angles = angle.euler_angles();
        dbg!(euler_angles.0.to_degrees());
        dbg!(euler_angles.1.to_degrees());
        dbg!(euler_angles.2.to_degrees());
        assert!((euler_angles.0.to_degrees() - 45.) < 0.01);
        assert!((euler_angles.1.to_degrees() - 45.) < 0.01);
        assert!((euler_angles.2.to_degrees() - 0.) < 0.01);
    }
}
