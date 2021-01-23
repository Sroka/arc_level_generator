use std::collections::VecDeque;
use crate::generator::types::{Feature, CollidableEntity, VisibleWorld};
use nalgebra::{Vector3, Isometry3, Translation3, };
use crate::generator::tilt_motion::{BiArcCurveMotion};
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
                     obstacles: &mut VecDeque<CollidableEntity>,
                     generated_entities: &mut Vec<CollidableEntity>,
                     time: f32,
                     world: &VisibleWorld,
                     feature_shift: &Vector3<f32>,
) {
    let max_time_to_travel = feature.max_time_to_travel(&world, &feature_shift);
    for prefab in &feature.prefabs {
        let prefab_motion = BiArcCurveMotion::new(
            max_time_to_travel,
            Isometry3::from_parts(Translation3::from(prefab.position + Vector3::new(feature_shift.x, feature_shift.y, 0.)), prefab.rotation),
            prefab.movement.baseline_velocity.clone(),
            prefab.movement.arcs_plane_normal.clone(),
            prefab.movement.approach_arc_angle,
            prefab.movement.approach_arc_center_distance,
            prefab.movement.approach_arc_radius,
            prefab.movement.approach_rotation_strength,
            prefab.movement.approach_arc_angle,
            prefab.movement.approach_arc_center_distance,
            prefab.movement.approach_arc_radius,
            prefab.movement.approach_rotation_strength,
        );
        let entity = CollidableEntity {
            movement_start_parameter: -max_time_to_travel,
            movement_end_parameter: prefab.find_departure_time_in_world(&world, feature_shift),
            spawn_position: prefab_motion.position_at_time(0.).translation.vector,
            spawn_rotation: prefab_motion.rotation_at_time(0.),
            spawn_feature_shift: feature_shift.clone(),
            spawn_time: time + feature.priority as f32,
            prefab: prefab.clone(),
            priority: feature.priority,
        };
        obstacles.push_back(entity.clone());
        generated_entities.push(entity.clone());
    }
}

#[cfg(test)]
mod tests {
    use crate::{Feature, Prefab, Movement, VisibleWorld};
    use nalgebra::{Vector3, Point3, UnitQuaternion, Unit};
    use crate::generator::spawn_feature::{spawn_feature};
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
                baseline_velocity: Vector3::new(0., 0., -8.0),
                arcs_plane_normal: Unit::new_normalize(Vector3::new(1., 0., 0.)),
                approach_arc_angle: 0.0,
                approach_arc_center_distance: 0.0,
                approach_arc_radius: 50.0,
                approach_rotation_strength: 0.,
                departure_arc_angle: 0.0,
                departure_arc_center_distance: 0.0,
                departure_arc_radius: 0.0,
                departure_rotation_strength: 0.0,
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
                baseline_velocity: Vector3::new(0., 0., -8.0),
                arcs_plane_normal: Unit::new_normalize(Vector3::new(1., 0., 0.)),
                approach_arc_angle: 45.0_f32.to_radians(),
                approach_arc_center_distance: 0.0,
                approach_arc_radius: 50.0,
                approach_rotation_strength: 0.,
                departure_arc_angle: 0.0,
                departure_arc_center_distance: 0.0,
                departure_arc_radius: 0.0,
                departure_rotation_strength: 0.0,
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
