use nalgebra::{Vector3, Vector2, Vector4, Unit, Quaternion, UnitQuaternion, Point3};
use crate::{Movement, Prefab, CollidableEntity};
use ncollide3d::bounding_volume::AABB;

#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct PrefabDescription {
    pub prefab_id: i32,
    pub position: Vector3<f32>,
    pub rotation: Vector4<f32>,
    pub half_extents: Vector3<f32>,
    pub movement: MovementDescription,
}

#[derive(Debug, Clone)]
#[repr(C)]
pub struct FeatureDescription {
    pub prefabs: *const PrefabDescription,
    pub prefabs_count: i32,
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
}

#[derive(Debug, Clone)]
#[repr(C)]
pub struct VisibleWorldDescription {
    pub position: Vector3<f32>,
    pub half_extents: Vector3<f32>,
}

#[derive(Debug, Clone)]
#[repr(C)]
pub struct EntityDescription {
    pub prefab_id: i32,
    pub spawn_position: Vector3<f32>,
    pub spawn_rotation: Vector4<f32>,
    pub spawn_time: f32,
    pub movement_start_parameter: f32,
    pub movement_end_parameter: f32,
    pub movement: MovementDescription,
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct MovementDescription {
    pub baseline_velocity: Vector3<f32>,
    pub arcs_plane_normal: Vector3<f32>,

    pub approach_arc_angle: f32,
    pub approach_arc_center_distance: f32,
    pub approach_arc_radius: f32,
    pub approach_rotation_strength: f32,

    pub departure_arc_angle: f32,
    pub departure_arc_center_distance: f32,
    pub departure_arc_radius: f32,
    pub departure_rotation_strength: f32,
}

#[repr(C)]
pub struct EntitiesArrayDescription {
    pub pointer: *mut EntityDescription,
    pub length: i32,
}

impl Into<Movement> for MovementDescription {
    fn into(self) -> Movement {
        Movement {
            baseline_velocity: self.baseline_velocity,
            arcs_plane_normal: Unit::new_normalize(self.arcs_plane_normal),
            approach_arc_angle: self.approach_arc_angle,
            approach_arc_center_distance: self.approach_arc_center_distance,
            approach_arc_radius: self.approach_arc_radius,
            approach_rotation_strength: self.approach_rotation_strength,
            departure_arc_angle: self.departure_arc_angle,
            departure_arc_center_distance: self.departure_arc_center_distance,
            departure_arc_radius: self.departure_arc_radius,
            departure_rotation_strength: self.departure_rotation_strength,
        }
    }
}

impl From<Movement> for MovementDescription {
    fn from(movement: Movement) -> Self {
        MovementDescription {
            baseline_velocity: movement.baseline_velocity,
            arcs_plane_normal: *movement.arcs_plane_normal,
            approach_arc_angle: movement.approach_arc_angle,
            approach_arc_center_distance: movement.approach_arc_center_distance,
            approach_arc_radius: movement.approach_arc_radius,
            approach_rotation_strength: movement.approach_rotation_strength,
            departure_arc_angle: movement.departure_arc_angle,
            departure_arc_center_distance: movement.departure_arc_center_distance,
            departure_arc_radius: movement.departure_arc_radius,
            departure_rotation_strength: movement.departure_rotation_strength,
        }
    }
}

impl Into<Prefab> for PrefabDescription {
    fn into(self) -> Prefab {
        Prefab {
            prefab_id: self.prefab_id,
            position: self.position,
            rotation: UnitQuaternion::from_quaternion(Quaternion::from(self.rotation)),
            bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), self.half_extents),
            movement: self.movement.into(),
        }
    }
}

impl From<CollidableEntity> for EntityDescription {
    fn from(entity: CollidableEntity) -> Self {
        EntityDescription {
            prefab_id: entity.prefab.prefab_id,
            spawn_position: entity.spawn_position,
            spawn_rotation: entity.spawn_rotation.coords.clone(),
            spawn_time: entity.spawn_time,
            movement_start_parameter: entity.movement_start_parameter,
            movement_end_parameter: entity.movement_end_parameter,
            movement: entity.prefab.movement.into()
        }
    }
}