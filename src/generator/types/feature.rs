use crate::generator::types::prefab::Prefab;
use nalgebra::{Vector2, Vector3};
use crate::VisibleWorld;
use itertools::Itertools;
use std::cmp::Ordering::Equal;

/// Represents a description of a feature that can be spawned in a generated world. Single feature
/// can consist of a multiple prefabs. The feature can only be spawned if all of its prefabs can be
/// spawned so that they won't collide with any of other already spawned entities
#[derive(Clone, PartialEq, Debug)]
pub struct Feature {
    pub prefabs: Vec<Prefab>,
    pub spawn_period: f32,
    pub is_spawn_period_strict: bool,
    pub spawn_count: i32,
    pub trigger_time: f32,
    pub priority: i32,
    pub translate_x: bool,
    pub translate_x_using_bounds: bool,
    pub translate_x_bounds: Vector2<f32>,
    pub translate_y: bool,
    pub translate_y_using_bounds: bool,
    pub translate_y_bounds: Vector2<f32>,
    pub missed_spawns: i32,
    pub last_spawn_attempt: f32,
}

impl Feature {
    pub fn max_approach_time(&self, world: &VisibleWorld, shift: &Vector3<f32>) -> f32 {
        self.prefabs
            .iter()
            .map(|prefab| prefab.find_approach_time_in_world(&world, &shift))
            .sorted_by(|a, b| { a.partial_cmp(b).unwrap_or(Equal) })
            .last()
            .unwrap()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{UnitQuaternion, Point3, Vector3, Unit};
    use ncollide3d::bounding_volume::AABB;
    use crate::Movement;

    #[test]
    pub fn test_max_time_to_travel() {
        let world = VisibleWorld {
            world_bounds: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(20., 20., 20.)),
        };
        let prefab1 = Prefab {
            prefab_id: 1,
            position: nalgebra::zero(),
            rotation: UnitQuaternion::identity(),
            bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
            movement: Movement {
                baseline_velocity: Vector3::new(0., 0., -8.),
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
        let prefab2 = Prefab {
            prefab_id: 1,
            position: nalgebra::zero(),
            rotation: UnitQuaternion::identity(),
            bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
            movement: Movement {
                baseline_velocity: Vector3::new(0., 0., -4.),
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
        let prefab3 = Prefab {
            prefab_id: 1,
            position: nalgebra::zero(),
            rotation: UnitQuaternion::identity(),
            bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
            movement: Movement {
                baseline_velocity: Vector3::new(0., 0., -2.),
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
            prefabs: vec![prefab1, prefab2, prefab3],
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
            missed_spawns: 0,
            last_spawn_attempt: 0.0,
        };
        let max_time_to_travel = feature.max_approach_time(&world, &Vector3::new(0., 0., 0.));
        assert_eq!(max_time_to_travel, 10.25);
    }
}
