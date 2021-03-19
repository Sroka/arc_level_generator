#[cfg(test)]
mod tests {
    extern crate arc_level_generator;

    use self::arc_level_generator::{PrefabDescription, FeatureDescription, VisibleWorldDescription, MovementDescription};
    use nalgebra::{Vector3, Vector2, UnitQuaternion, Unit};
    use std::slice::from_raw_parts;

    #[test]
    fn test_bind_generate() {
        let description = PrefabDescription {
            prefab_id: 1,
            position: Vector3::new(0., 0., 0.),
            rotation: nalgebra::zero(),
            movement: MovementDescription {
                baseline_velocity: Vector3::new(0., 0., -1.),
                arcs_plane_normal: Vector3::new(1., 0., 0.),
                approach_arc_angle: 0.0,
                approach_arc_center_distance: 0.0,
                approach_arc_radius: 0.0,
                approach_rotation_strength: 0.0,
                departure_arc_angle: 0.0,
                departure_arc_center_distance: 0.0,
                departure_arc_radius: 0.0,
                departure_rotation_strength: 0.0,
            },
            half_extents: Vector3::new(0.5, 0.5, 0.5),
        };
        let description1 = PrefabDescription {
            prefab_id: 2,
            position: Vector3::new(1., 0., 0.),
            rotation: nalgebra::zero(),
            movement: MovementDescription {
                baseline_velocity: Vector3::new(0., 0., -1.),
                arcs_plane_normal: Vector3::new(1., 0., 0.),
                approach_arc_angle: 0.0,
                approach_arc_center_distance: 0.0,
                approach_arc_radius: 0.0,
                approach_rotation_strength: 0.0,
                departure_arc_angle: 0.0,
                departure_arc_center_distance: 0.0,
                departure_arc_radius: 0.0,
                departure_rotation_strength: 0.0,
            },
            half_extents: Vector3::new(0.5, 0.5, 0.5),
        };
        let description2 = PrefabDescription {
            prefab_id: 3,
            position: Vector3::new(-1., 0., 0.),
            rotation: nalgebra::zero(),
            movement: MovementDescription {
                baseline_velocity: Vector3::new(0., 0., -1.),
                arcs_plane_normal: Vector3::new(1., 0., 0.),
                approach_arc_angle: 0.0,
                approach_arc_center_distance: 0.0,
                approach_arc_radius: 0.0,
                approach_rotation_strength: 0.0,
                departure_arc_angle: 0.0,
                departure_arc_center_distance: 0.0,
                departure_arc_radius: 0.0,
                departure_rotation_strength: 0.0,
            },
            half_extents: Vector3::new(0.5, 0.5, 0.5),
        };
        let description3 = PrefabDescription {
            prefab_id: 4,
            position: Vector3::new(2., 0., 2.),
            rotation: nalgebra::zero(),
            movement: MovementDescription {
                baseline_velocity: Vector3::new(-1., -1., -1.),
                arcs_plane_normal: Vector3::new(1., 0., 0.),
                approach_arc_angle: 0.0,
                approach_arc_center_distance: 0.0,
                approach_arc_radius: 0.0,
                approach_rotation_strength: 0.0,
                departure_arc_angle: 0.0,
                departure_arc_center_distance: 0.0,
                departure_arc_radius: 0.0,
                departure_rotation_strength: 0.0,
            },
            half_extents: Vector3::new(0.5, 0.5, 0.5),
        };
        let description4 = PrefabDescription {
            prefab_id: 5,
            position: Vector3::new(-2., 0., 2.),
            rotation: nalgebra::zero(),
            movement: MovementDescription {
                baseline_velocity: Vector3::new(2., 0., -1.),
                arcs_plane_normal: Vector3::new(1., 0., 0.),
                approach_arc_angle: 0.0,
                approach_arc_center_distance: 0.0,
                approach_arc_radius: 0.0,
                approach_rotation_strength: 0.0,
                departure_arc_angle: 0.0,
                departure_arc_center_distance: 0.0,
                departure_arc_radius: 0.0,
                departure_rotation_strength: 0.0,
            },
            half_extents: Vector3::new(0.5, 0.5, 0.5),
        };
        let features = [
            FeatureDescription {
                translate_x: true,
                translate_x_using_bounds: false,
                translate_x_bounds: Vector2::new(1., 1.),
                translate_y: true,
                translate_y_using_bounds: false,
                translate_y_bounds: Vector2::new(1., 1.),
                prefabs: [description, description1, description2].as_ptr(),
                prefabs_count: 3,
                spawn_period: 1.0,
                is_spawn_period_strict: false,
                spawn_count: 10,
                trigger_time: 10.0,
                priority: 0,
            },
            FeatureDescription {
                translate_x: true,
                translate_x_using_bounds: false,
                translate_x_bounds: Vector2::new(1., 1.),
                translate_y: true,
                translate_y_using_bounds: false,
                translate_y_bounds: Vector2::new(1., 1.),
                prefabs: [description3].as_ptr(),
                prefabs_count: 1,
                spawn_period: 1.0,
                is_spawn_period_strict: false,
                spawn_count: 10,
                trigger_time: 20.0,
                priority: 0,
            },
            FeatureDescription {
                translate_x: true,
                translate_x_using_bounds: false,
                translate_x_bounds: Vector2::new(1., 1.),
                translate_y: true,
                translate_y_using_bounds: false,
                translate_y_bounds: Vector2::new(1., 1.),
                prefabs: [description4].as_ptr(),
                prefabs_count: 1,
                spawn_period: 1.0,
                is_spawn_period_strict: false,
                spawn_count: 10,
                trigger_time: 30.0,
                priority: 0,
            },
        ];
        let world = VisibleWorldDescription {
            position: Vector3::new(0., 0., 0.),
            half_extents: Vector3::new(10., 10., 10.),
        };
        unsafe {
            let generated_entities_description = arc_level_generator::bind_generate(
                features.as_ptr(),
                features.len() as i32,
                world,
            );
            let entities = from_raw_parts(generated_entities_description.pointer, generated_entities_description.length as usize);

            for entity in entities {
                println!("Entity description: {:?}", entity)
            }
            arc_level_generator::bind_deallocate_vec(generated_entities_description)
        }
    }

    #[test]
    fn test_bind_generate_varying_speeds() {
        let description = PrefabDescription {
            prefab_id: 1,
            position: Vector3::new(0., 0., 0.),
            rotation: nalgebra::zero(),
            movement: MovementDescription {
                baseline_velocity: Vector3::new(0., 0., -4.),
                arcs_plane_normal: Vector3::new(1., 0., 0.),
                approach_arc_angle: 0.0,
                approach_arc_center_distance: 0.0,
                approach_arc_radius: 0.0,
                approach_rotation_strength: 0.0,
                departure_arc_angle: 0.0,
                departure_arc_center_distance: 0.0,
                departure_arc_radius: 0.0,
                departure_rotation_strength: 0.0,
            },
            half_extents: Vector3::new(0.5, 0.5, 0.5),
        };
        let description1 = PrefabDescription {
            prefab_id: 2,
            position: Vector3::new(0., 0., 0.),
            rotation: nalgebra::zero(),
            movement: MovementDescription {
                baseline_velocity: Vector3::new(0., 0., -6.),
                arcs_plane_normal: Vector3::new(1., 0., 0.),
                approach_arc_angle: 0.0,
                approach_arc_center_distance: 0.0,
                approach_arc_radius: 0.0,
                approach_rotation_strength: 0.0,
                departure_arc_angle: 0.0,
                departure_arc_center_distance: 0.0,
                departure_arc_radius: 0.0,
                departure_rotation_strength: 0.0,
            },
            half_extents: Vector3::new(0.5, 0.5, 0.5),
        };
        let features = [
            FeatureDescription {
                translate_x: false,
                translate_x_using_bounds: false,
                translate_x_bounds: Vector2::new(1., 1.),
                translate_y: false,
                translate_y_using_bounds: false,
                translate_y_bounds: Vector2::new(1., 1.),
                prefabs: [description].as_ptr(),
                prefabs_count: 1,
                spawn_period: 0.1,
                is_spawn_period_strict: false,
                spawn_count: 30,
                trigger_time: 10.0,
                priority: 0,
            },
            FeatureDescription {
                translate_x: false,
                translate_x_using_bounds: false,
                translate_x_bounds: Vector2::new(1., 1.),
                translate_y: false,
                translate_y_using_bounds: false,
                translate_y_bounds: Vector2::new(1., 1.),
                prefabs: [description1].as_ptr(),
                prefabs_count: 1,
                spawn_period: 0.1,
                is_spawn_period_strict: false,
                spawn_count: 30,
                trigger_time: 10.0,
                priority: 0,
            },
        ];
        let world = VisibleWorldDescription {
            position: Vector3::new(0., 0., 0.),
            half_extents: Vector3::new(9., 9., 30.),
        };
        unsafe {
            let generated_entities_description = arc_level_generator::bind_generate(
                features.as_ptr(),
                features.len() as i32,
                world,
            );
            let entities = from_raw_parts(generated_entities_description.pointer, generated_entities_description.length as usize);

            for entity in entities {
                println!("Entity description: {:?}", entity)
            }
            arc_level_generator::bind_deallocate_vec(generated_entities_description)
        }
    }

    #[test]
    fn test_bind_generate_large() {
        let description = PrefabDescription {
            prefab_id: 1,
            position: Vector3::new(0., 0., 0.),
            rotation: nalgebra::zero(),
            movement: MovementDescription {
                baseline_velocity: Vector3::new(0., 0., -1.),
                arcs_plane_normal: Vector3::new(1., 0., 0.),
                approach_arc_angle: 0.0,
                approach_arc_center_distance: 0.0,
                approach_arc_radius: 0.0,
                approach_rotation_strength: 0.0,
                departure_arc_angle: 0.0,
                departure_arc_center_distance: 0.0,
                departure_arc_radius: 0.0,
                departure_rotation_strength: 0.0,
            },
            half_extents: Vector3::new(4.0, 4.0, 4.0),
        };
        let description1 = PrefabDescription {
            prefab_id: 2,
            position: Vector3::new(0., 0., 0.),
            rotation: nalgebra::zero(),
            movement: MovementDescription {
                baseline_velocity: Vector3::new(0., 0., -1.),
                arcs_plane_normal: Vector3::new(1., 0., 0.),
                approach_arc_angle: 0.0,
                approach_arc_center_distance: 0.0,
                approach_arc_radius: 0.0,
                approach_rotation_strength: 0.0,
                departure_arc_angle: 0.0,
                departure_arc_center_distance: 0.0,
                departure_arc_radius: 0.0,
                departure_rotation_strength: 0.0,
            },
            half_extents: Vector3::new(4.0, 4.0, 4.0),
        };
        let features = [
            FeatureDescription {
                translate_x: false,
                translate_x_using_bounds: false,
                translate_x_bounds: Vector2::new(1., 1.),
                translate_y: false,
                translate_y_using_bounds: false,
                translate_y_bounds: Vector2::new(1., 1.),
                prefabs: [description].as_ptr(),
                prefabs_count: 1,
                spawn_period: 0.1,
                is_spawn_period_strict: false,
                spawn_count: 1,
                trigger_time: 10.0,
                priority: 0,
            },
            FeatureDescription {
                translate_x: false,
                translate_x_using_bounds: false,
                translate_x_bounds: Vector2::new(1., 1.),
                translate_y: false,
                translate_y_using_bounds: false,
                translate_y_bounds: Vector2::new(1., 1.),
                prefabs: [description1].as_ptr(),
                prefabs_count: 1,
                spawn_period: 0.1,
                is_spawn_period_strict: false,
                spawn_count: 1,
                trigger_time: 10.0,
                priority: 0,
            },
        ];
        let world = VisibleWorldDescription {
            position: Vector3::new(0., 0., 0.),
            half_extents: Vector3::new(9., 9., 30.),
        };
        unsafe {
            let generated_entities_description = arc_level_generator::bind_generate(
                features.as_ptr(),
                features.len() as i32,
                world,
            );
            let entities = from_raw_parts(generated_entities_description.pointer, generated_entities_description.length as usize);

            for entity in entities {
                println!("Entity description: {:?}", entity)
            }
            arc_level_generator::bind_deallocate_vec(generated_entities_description)
        }
    }


    #[test]
    fn test_simple_tilt() {
        let description = PrefabDescription {
            prefab_id: 0,
            position: Vector3::new(0., 0., 0.),
            rotation: UnitQuaternion::from_axis_angle(&Unit::new_normalize(Vector3::new(0., 1., 0.)), -45.).coords,
            movement: MovementDescription {
                baseline_velocity: Vector3::new(8., 0., -8.),
                arcs_plane_normal: Vector3::new(1., 0., 0.),
                approach_arc_angle: 0.0,
                approach_arc_center_distance: 0.0,
                approach_arc_radius: 0.0,
                approach_rotation_strength: 0.0,
                departure_arc_angle: 0.0,
                departure_arc_center_distance: 0.0,
                departure_arc_radius: 0.0,
                departure_rotation_strength: 0.0,
            },
            half_extents: Vector3::new(0.5, 0.5, 0.5),
        };
        let features = [
            FeatureDescription {
                translate_x: false,
                translate_x_using_bounds: false,
                translate_x_bounds: Vector2::new(-4.5, 4.5),
                translate_y: false,
                translate_y_using_bounds: false,
                translate_y_bounds: Vector2::new(-4.5, 4.5),
                prefabs: [description].as_ptr(),
                prefabs_count: 1,
                spawn_period: 0.0001,
                is_spawn_period_strict: false,
                spawn_count: 1,
                trigger_time: 0.0,
                priority: 8,
            },
        ];
        let world = VisibleWorldDescription {
            position: Vector3::new(0., 0., 175.),
            half_extents: Vector3::new(100., 100., 400.),
        };
        unsafe {
            let generated_entities_description = arc_level_generator::bind_generate(
                features.as_ptr(),
                features.len() as i32,
                world,
            );
            let entities = from_raw_parts(generated_entities_description.pointer, generated_entities_description.length as usize);

            for entity in entities {
                println!("Entity description: {:?}", entity)
            }
            arc_level_generator::bind_deallocate_vec(generated_entities_description)
        }
    }

    #[test]
    fn test_rotated_45_hang() {
        let description = PrefabDescription {
            prefab_id: 0,
            position: Vector3::new(0., -7., 0.),
            rotation: UnitQuaternion::from_axis_angle(&Unit::new_normalize(Vector3::new(0., 1., 0.)), -45.).coords,
            movement: MovementDescription {
                baseline_velocity: Vector3::new(8., 0., -8.),
                arcs_plane_normal: Vector3::new(1., 0., 0.),
                approach_arc_angle: 0.0,
                approach_arc_center_distance: 0.0,
                approach_arc_radius: 0.0,
                approach_rotation_strength: 0.0,
                departure_arc_angle: 0.0,
                departure_arc_center_distance: 0.0,
                departure_arc_radius: 0.0,
                departure_rotation_strength: 0.0,
            },
            half_extents: Vector3::new(12.43841, 8.28537, 30.5627),
        };
        let features = [
            FeatureDescription {
                translate_x: false,
                translate_x_using_bounds: false,
                translate_x_bounds: Vector2::new(-4.5, 4.5),
                translate_y: false,
                translate_y_using_bounds: false,
                translate_y_bounds: Vector2::new(-4.5, 4.5),
                prefabs: [description].as_ptr(),
                prefabs_count: 1,
                spawn_period: 0.1,
                is_spawn_period_strict: false,
                spawn_count: 10,
                trigger_time: 0.0,
                priority: 50,
            },
        ];
        let world = VisibleWorldDescription {
            position: Vector3::new(0., 0., 175.),
            half_extents: Vector3::new(100., 100., 400.),
        };
        unsafe {
            let generated_entities_description = arc_level_generator::bind_generate(
                features.as_ptr(),
                features.len() as i32,
                world,
            );
            let entities = from_raw_parts(generated_entities_description.pointer, generated_entities_description.length as usize);

            for entity in entities {
                println!("Entity description: {:?}", entity)
            }
            arc_level_generator::bind_deallocate_vec(generated_entities_description)
        }
    }

    #[test]
    fn test_rotated_135() {
        let description = PrefabDescription {
            prefab_id: 0,
            position: Vector3::new(0., -7., 0.),
            rotation: UnitQuaternion::from_axis_angle(&Unit::new_normalize(Vector3::new(0., 1., 0.)), 135.).coords,
            movement: MovementDescription {
                baseline_velocity: Vector3::new(8., 0., -8.),
                arcs_plane_normal: Vector3::new(1., 0., 0.),
                approach_arc_angle: 0.0,
                approach_arc_center_distance: 0.0,
                approach_arc_radius: 0.0,
                approach_rotation_strength: 0.0,
                departure_arc_angle: 0.0,
                departure_arc_center_distance: 0.0,
                departure_arc_radius: 0.0,
                departure_rotation_strength: 0.0,
            },
            half_extents: Vector3::new(12.43841, 8.28537, 30.5627),
        };
        let features = [
            FeatureDescription {
                translate_x: false,
                translate_x_using_bounds: false,
                translate_x_bounds: Vector2::new(-4.5, 4.5),
                translate_y: false,
                translate_y_using_bounds: false,
                translate_y_bounds: Vector2::new(-4.5, 4.5),
                prefabs: [description].as_ptr(),
                prefabs_count: 1,
                spawn_period: 0.1,
                is_spawn_period_strict: false,
                spawn_count: 10,
                trigger_time: 0.0,
                priority: 50,
            },
        ];
        let world = VisibleWorldDescription {
            position: Vector3::new(0., 0., 175.),
            half_extents: Vector3::new(100., 100., 400.),
        };
        unsafe {
            let generated_entities_description = arc_level_generator::bind_generate(
                features.as_ptr(),
                features.len() as i32,
                world,
            );
            let entities = from_raw_parts(generated_entities_description.pointer, generated_entities_description.length as usize);

            for entity in entities {
                println!("Entity description: {:?}", entity)
            }
            arc_level_generator::bind_deallocate_vec(generated_entities_description)
        }
    }
}
