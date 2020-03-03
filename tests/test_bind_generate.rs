#[cfg(test)]
mod tests {
    extern crate arc_level_generator;

    use self::arc_level_generator::{PrefabDescription, FeatureDescription, VisibleWorldDescription};
    use nalgebra::Vector3;
    use std::slice::from_raw_parts;

    #[test]
    fn test_bind_generate() {
        let description = PrefabDescription {
            prefab_id: 1,
            position: Vector3::new(0., 0., 0.),
            velocity: Vector3::new(0., -1., 0.),
            half_extents: Vector3::new(0.5, 0.5, 0.5),
        };
        let description1 = PrefabDescription {
            prefab_id: 2,
            position: Vector3::new(1., 0., 0.),
            velocity: Vector3::new(0., -1., 0.),
            half_extents: Vector3::new(0.5, 0.5, 0.5),
        };
        let description2 = PrefabDescription {
            prefab_id: 3,
            position: Vector3::new(-1., 0., 0.),
            velocity: Vector3::new(0., -1., 0.),
            half_extents: Vector3::new(0.5, 0.5, 0.5),
        };
        let description3 = PrefabDescription {
            prefab_id: 4,
            position: Vector3::new(2., 0., 2.),
            velocity: Vector3::new(-1., -1., -1.),
            half_extents: Vector3::new(0.5, 0.5, 0.5),
        };
        let description4 = PrefabDescription {
            prefab_id: 5,
            position: Vector3::new(-2., 0., 2.),
            velocity: Vector3::new(2., -1., 0.),
            half_extents: Vector3::new(0.5, 0.5, 0.5),
        };
        let features = [
            FeatureDescription {
                translate_x: true,
                translate_x_out_of_bounds: false,
                translate_z: true,
                translate_z_out_of_bounds: false,
                prefabs: [description, description1, description2].as_ptr(),
                prefabs_count: 3,
                spawns_per_second: 1.0,
                spawn_count: 10,
                trigger_position: 10.0,
                priority: 0,
            },
            FeatureDescription {
                translate_x: true,
                translate_x_out_of_bounds: false,
                translate_z: true,
                translate_z_out_of_bounds: false,
                prefabs: [description3].as_ptr(),
                prefabs_count: 1,
                spawns_per_second: 1.0,
                spawn_count: 10,
                trigger_position: 20.0,
                priority: 0,
            },
            FeatureDescription {
                translate_x: true,
                translate_x_out_of_bounds: false,
                translate_z: true,
                translate_z_out_of_bounds: false,
                prefabs: [description4].as_ptr(),
                prefabs_count: 1,
                spawns_per_second: 1.0,
                spawn_count: 10,
                trigger_position: 30.0,
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
            velocity: Vector3::new(0., -4., 0.),
            half_extents: Vector3::new(0.5, 0.5, 0.5),
        };
        let description1 = PrefabDescription {
            prefab_id: 2,
            position: Vector3::new(0., 0., 0.),
            velocity: Vector3::new(0., -6., 0.),
            half_extents: Vector3::new(0.5, 0.5, 0.5),
        };
        let features = [
            FeatureDescription {
                translate_x: false,
                translate_x_out_of_bounds: false,
                translate_z: false,
                translate_z_out_of_bounds: false,
                prefabs: [description].as_ptr(),
                prefabs_count: 1,
                spawns_per_second: 0.1,
                spawn_count: 30,
                trigger_position: 10.0,
                priority: 0,
            },
            FeatureDescription {
                translate_x: false,
                translate_x_out_of_bounds: false,
                translate_z: false,
                translate_z_out_of_bounds: false,
                prefabs: [description1].as_ptr(),
                prefabs_count: 1,
                spawns_per_second: 0.1,
                spawn_count: 30,
                trigger_position: 10.0,
                priority: 0,
            },
        ];
        let world = VisibleWorldDescription {
            position: Vector3::new(0., 0., 0.),
            half_extents: Vector3::new(9., 30., 9.),
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
            velocity: Vector3::new(0., -1., 0.),
            half_extents: Vector3::new(4.0, 4.0, 4.0),
        };
        let description1 = PrefabDescription {
            prefab_id: 2,
            position: Vector3::new(0., 0., 0.),
            velocity: Vector3::new(0., -1., 0.),
            half_extents: Vector3::new(4.0, 4.0, 4.0),
        };
        let features = [
            FeatureDescription {
                translate_x: false,
                translate_x_out_of_bounds: false,
                translate_z: false,
                translate_z_out_of_bounds: false,
                prefabs: [description].as_ptr(),
                prefabs_count: 1,
                spawns_per_second: 0.1,
                spawn_count: 1,
                trigger_position: 10.0,
                priority: 0,
            },
            FeatureDescription {
                translate_x: false,
                translate_x_out_of_bounds: false,
                translate_z: false,
                translate_z_out_of_bounds: false,
                prefabs: [description1].as_ptr(),
                prefabs_count: 1,
                spawns_per_second: 0.1,
                spawn_count: 1,
                trigger_position: 10.0,
                priority: 0,
            },
        ];
        let world = VisibleWorldDescription {
            position: Vector3::new(0., 0., 0.),
            half_extents: Vector3::new(9., 30., 9.),
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
