use nalgebra::{Vector3, UnitQuaternion, Isometry3, Translation3};
use ncollide3d::bounding_volume::{AABB};
use crate::generator::types::movement::Movement;
use crate::VisibleWorld;
use ncollide3d::query::{RayCast, Ray};
use serde::{Serialize, Deserialize};
use crate::generator::types::{serialize_aabb, deserialize_aabb};

/// Represents single smallest piece of a generated level
#[derive(Clone, PartialEq, Debug, Serialize, Deserialize)]
pub struct Prefab {
    pub prefab_id: i32,
    pub position: Vector3<f32>,
    pub rotation: UnitQuaternion<f32>,
    #[serde(serialize_with = "serialize_aabb", deserialize_with = "deserialize_aabb")]
    pub bounding_box: AABB<f32>,
    pub movement: Movement,
}

impl Prefab {
    pub fn find_approach_time_in_world(&self, world: &VisibleWorld, shift: &Vector3<f32>) -> f32 {
        let extended_world_bounds = loosened_unequal(&world.world_bounds, &self.bounding_box.half_extents());
        let baseline_velocity_magnitude = self.movement.baseline_velocity.magnitude();
        let baseline_movement_line_length = self.movement.approach_arc_center_distance + self.movement.approach_arc_angle.sin().abs() * self.movement.approach_arc_radius;
        let baseline_movement_toi = extended_world_bounds
            .toi_with_ray(&Isometry3::from_parts(Translation3::identity(), UnitQuaternion::identity()),
                          &Ray::new((&self.position + shift).into(), -self.movement.baseline_velocity.normalize()),
                          f32::MAX,
                          false)
            .unwrap();
        if baseline_movement_toi <= baseline_movement_line_length {
            let baseline_movement_velocity_toi = baseline_movement_toi / baseline_velocity_magnitude;
            return baseline_movement_velocity_toi;
        }

        let arc_direction = self.movement.arcs_plane_normal.cross(&self.movement.baseline_velocity.normalize());
        let approach_movement_line_direction = (-self.movement.baseline_velocity +
            arc_direction * self.movement.approach_arc_angle.tan() * baseline_velocity_magnitude).normalize();

        let approach_movement_line_start = &self.position + shift - self.movement.baseline_velocity.normalize() * baseline_movement_line_length;
        let approach_movement_toi = extended_world_bounds
            .toi_with_ray(
                &Isometry3::new(nalgebra::zero(), nalgebra::zero()),
                &Ray::new(approach_movement_line_start.into(), approach_movement_line_direction),
                f32::MAX,
                false,
            )
            .unwrap();
        let baseline_movement_velocity_toi = baseline_movement_line_length / baseline_velocity_magnitude;
        let approach_movement_velocity_toi = approach_movement_toi * self.movement.approach_arc_angle.cos() / baseline_velocity_magnitude;
        baseline_movement_velocity_toi + approach_movement_velocity_toi
    }

    pub fn find_departure_time_in_world(&self, world: &VisibleWorld, shift: &Vector3<f32>) -> f32 {
        let extended_world_bounds = loosened_unequal(&world.world_bounds, &self.bounding_box.half_extents());
        let baseline_velocity_magnitude = self.movement.baseline_velocity.magnitude();
        let baseline_movement_line_length = self.movement.departure_arc_center_distance + self.movement.departure_arc_angle.sin().abs() * self.movement.departure_arc_radius;
        let baseline_movement_toi = extended_world_bounds
            .toi_with_ray(&Isometry3::from_parts(Translation3::identity(), UnitQuaternion::identity()),
                          &Ray::new((&self.position + shift).into(), self.movement.baseline_velocity.normalize()),
                          f32::MAX,
                          false)
            .unwrap();
        if baseline_movement_toi <= baseline_movement_line_length {
            let baseline_movement_velocity_toi = baseline_movement_toi / baseline_velocity_magnitude;
            return baseline_movement_velocity_toi;
        }

        let arc_direction = self.movement.arcs_plane_normal.cross(&self.movement.baseline_velocity.normalize());
        let departure_movement_line_direction = (self.movement.baseline_velocity +
            arc_direction * self.movement.departure_arc_angle.tan() * baseline_velocity_magnitude).normalize();

        let departure_movement_line_start = &self.position + shift + self.movement.baseline_velocity.normalize() * baseline_movement_line_length;
        let departure_movement_toi = extended_world_bounds
            .toi_with_ray(
                &Isometry3::new(nalgebra::zero(), nalgebra::zero()),
                &Ray::new(departure_movement_line_start.into(), departure_movement_line_direction),
                f32::MAX,
                false,
            )
            .unwrap();
        let baseline_movement_velocity_toi = baseline_movement_line_length / baseline_velocity_magnitude;
        let departure_movement_velocity_toi = departure_movement_toi * self.movement.departure_arc_angle.cos() / baseline_velocity_magnitude;
        baseline_movement_velocity_toi + departure_movement_velocity_toi
    }
}

fn loosened_unequal(aabb: &AABB<f32>, amount: &Vector3<f32>) -> AABB<f32> {
    AABB {
        mins: &aabb.mins - amount,
        maxs: &aabb.maxs + amount,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Unit, Point3};

    #[test]
    fn test_find_approach_time_in_world_baseline_line() {
        let prefab = Prefab {
            prefab_id: 0,
            position: Vector3::new(10., 10., 10.),
            rotation: Default::default(),
            bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(2., 2., 2.)),
            movement: Movement {
                baseline_velocity: Vector3::new(0., 0., -2.),
                arcs_plane_normal: Unit::new_normalize(Vector3::new(1., 0., 0.)),
                approach_arc_angle: 45.0_f32.to_radians(),
                approach_arc_center_distance: 150.0,
                approach_arc_radius: 10.0,
                approach_rotation_strength: 0.0,
                departure_arc_angle: 0.0,
                departure_arc_center_distance: 0.0,
                departure_arc_radius: 0.0,
                departure_rotation_strength: 0.0,
            },
        };
        let world = VisibleWorld {
            world_bounds: AABB::from_half_extents(Point3::new(-10., -10., -10.), Vector3::new(50., 50., 50.))
        };

        let approach_time = prefab.find_approach_time_in_world(&world, &Vector3::new(10., 10., 10.));
        assert_eq!(approach_time, 11.);
    }

    #[test]
    fn test_find_approach_time_in_world_approach_line() {
        let prefab = Prefab {
            prefab_id: 0,
            position: Vector3::new(10., 10., 10.),
            rotation: Default::default(),
            bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(2., 2., 2.)),
            movement: Movement {
                baseline_velocity: Vector3::new(0., 0., -1.),
                arcs_plane_normal: Unit::new_normalize(Vector3::new(1., 0., 0.)),
                approach_arc_angle: 45.0_f32.to_radians(),
                approach_arc_center_distance: 10.0,
                approach_arc_radius: 10.0,
                approach_rotation_strength: 0.0,
                departure_arc_angle: 0.0,
                departure_arc_center_distance: 0.0,
                departure_arc_radius: 0.0,
                departure_rotation_strength: 0.0,
            },
        };
        let world = VisibleWorld {
            world_bounds: AABB::from_half_extents(Point3::new(-10., -10., -10.), Vector3::new(50., 50., 50.))
        };

        let approach_time = prefab.find_approach_time_in_world(&world, &Vector3::new(10., 10., 10.));
        assert_eq!(approach_time, 22.);
    }

    #[test]
    fn test_find_departure_time_in_world_baseline_line() {
        let prefab = Prefab {
            prefab_id: 0,
            position: Vector3::new(10., 10., 10.),
            rotation: Default::default(),
            bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(2., 2., 2.)),
            movement: Movement {
                baseline_velocity: Vector3::new(0., 0., -2.),
                arcs_plane_normal: Unit::new_normalize(Vector3::new(1., 0., 0.)),
                approach_arc_angle: 45.0_f32.to_radians(),
                approach_arc_center_distance: 150.0,
                approach_arc_radius: 10.0,
                approach_rotation_strength: 0.0,
                departure_arc_angle: 45.0_f32.to_radians(),
                departure_arc_center_distance: 150.0,
                departure_arc_radius: 10.0,
                departure_rotation_strength: 0.0,
            },
        };
        let world = VisibleWorld {
            world_bounds: AABB::from_half_extents(Point3::new(-10., -10., -10.), Vector3::new(50., 50., 50.))
        };

        let approach_time = prefab.find_departure_time_in_world(&world, &Vector3::new(10., 10., 10.));
        assert_eq!(approach_time, 41.);
    }

    #[test]
    fn test_find_departure_time_in_world_departure_line() {
        let prefab = Prefab {
            prefab_id: 0,
            position: Vector3::new(10., 10., 10.),
            rotation: Default::default(),
            bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(2., 2., 2.)),
            movement: Movement {
                baseline_velocity: Vector3::new(0., 0., -1.),
                arcs_plane_normal: Unit::new_normalize(Vector3::new(1., 0., 0.)),
                approach_arc_angle: 45.0_f32.to_radians(),
                approach_arc_center_distance: 10.0,
                approach_arc_radius: 10.0,
                approach_rotation_strength: 0.0,
                departure_arc_angle: 45.0_f32.to_radians(),
                departure_arc_center_distance: 10.0,
                departure_arc_radius: 10.0,
                departure_rotation_strength: 0.0,
            },
        };
        let world = VisibleWorld {
            world_bounds: AABB::from_half_extents(Point3::new(-10., -10., -10.), Vector3::new(50., 50., 50.))
        };

        let approach_time = prefab.find_departure_time_in_world(&world, &Vector3::new(10., 10., 10.));
        assert_eq!(approach_time, 39.071068);
    }

    #[test]
    fn test_find_approach_time_in_world_approach_line_with_negative_angle() {
        let prefab = Prefab {
            prefab_id: 0,
            position: Vector3::new(10., 10., 10.),
            rotation: Default::default(),
            bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(2., 2., 2.)),
            movement: Movement {
                baseline_velocity: Vector3::new(0., 0., -1.),
                arcs_plane_normal: Unit::new_normalize(Vector3::new(1., 0., 0.)),
                approach_arc_angle: -45.0_f32.to_radians(),
                approach_arc_center_distance: 10.0,
                approach_arc_radius: 10.0,
                approach_rotation_strength: 0.0,
                departure_arc_angle: 0.0,
                departure_arc_center_distance: 0.0,
                departure_arc_radius: 0.0,
                departure_rotation_strength: 0.0,
            },
        };
        let world = VisibleWorld {
            world_bounds: AABB::from_half_extents(Point3::new(-10., -10., -10.), Vector3::new(50., 50., 50.))
        };

        let approach_time = prefab.find_approach_time_in_world(&world, &Vector3::new(10., 10., 10.));
        assert_eq!(approach_time, 22.);
    }
}

