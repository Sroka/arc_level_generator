#[cfg(test)]
mod tests {
    extern crate arc_level_generator;

    use self::arc_level_generator::{PrefabDescription, FeatureDescription, VisibleWorldDescription};
    use nalgebra::{Vector3, Vector2};
    use std::slice::from_raw_parts;

    #[test]
    fn test_bind_generate() {
        let description = PrefabDescription {
            prefab_id: 1,
            position: Vector3::new(0., 0., 0.),
            euler_angles: nalgebra::zero(),
            velocity: Vector3::new(0., 0., -1.),
            half_extents: Vector3::new(0.5, 0.5, 0.5),
        };
        let description1 = PrefabDescription {
            prefab_id: 2,
            position: Vector3::new(1., 0., 0.),
            euler_angles: nalgebra::zero(),
            velocity: Vector3::new(0., 0., -1.),
            half_extents: Vector3::new(0.5, 0.5, 0.5),
        };
        let description2 = PrefabDescription {
            prefab_id: 3,
            position: Vector3::new(-1., 0., 0.),
            euler_angles: nalgebra::zero(),
            velocity: Vector3::new(0., 0., -1.),
            half_extents: Vector3::new(0.5, 0.5, 0.5),
        };
        let description3 = PrefabDescription {
            prefab_id: 4,
            position: Vector3::new(2., 0., 2.),
            euler_angles: nalgebra::zero(),
            velocity: Vector3::new(-1., -1., -1.),
            half_extents: Vector3::new(0.5, 0.5, 0.5),
        };
        let description4 = PrefabDescription {
            prefab_id: 5,
            position: Vector3::new(-2., 0., 2.),
            euler_angles: nalgebra::zero(),
            velocity: Vector3::new(2., 0., -1.),
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
            travel_speed: 4.0,
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
            euler_angles: nalgebra::zero(),
            velocity: Vector3::new(0., 0., -4.),
            half_extents: Vector3::new(0.5, 0.5, 0.5),
        };
        let description1 = PrefabDescription {
            prefab_id: 2,
            position: Vector3::new(0., 0., 0.),
            euler_angles: nalgebra::zero(),
            velocity: Vector3::new(0., 0., -6.),
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
            travel_speed: 4.0,
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
            euler_angles: nalgebra::zero(),
            velocity: Vector3::new(0., 0., -1.),
            half_extents: Vector3::new(4.0, 4.0, 4.0),
        };
        let description1 = PrefabDescription {
            prefab_id: 2,
            position: Vector3::new(0., 0., 0.),
            euler_angles: nalgebra::zero(),
            velocity: Vector3::new(0., 0., -1.),
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
            travel_speed: 4.0,
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
