#[cfg(test)]
mod tests {
    extern crate arc_level_generator;

    use self::arc_level_generator::{PrefabDescription, FeatureDescription, VisibleWorldDescription};
    use nalgebra::Vector3;
    use std::slice::from_raw_parts;

    #[test]
    fn test_bind_generate() {
        let prefabs = [
            PrefabDescription {
                prefab_id: 1,
                position: Vector3::new(0., 0., 0.),
                velocity: Vector3::new(0., -1., 0.),
                half_extents: Vector3::new(0.5, 0.5, 0.5),
            },
            PrefabDescription {
                prefab_id: 2,
                position: Vector3::new(1., 0., 0.),
                velocity: Vector3::new(0., -1., 0.),
                half_extents: Vector3::new(0.5, 0.5, 0.5),
            },
            PrefabDescription {
                prefab_id: 3,
                position: Vector3::new(-1., 0., 0.),
                velocity: Vector3::new(0., -1., 0.),
                half_extents: Vector3::new(0.5, 0.5, 0.5),
            },
            PrefabDescription {
                prefab_id: 4,
                position: Vector3::new(2., 0., 2.),
                velocity: Vector3::new(-1., -1., -1.),
                half_extents: Vector3::new(0.5, 0.5, 0.5),
            },
            PrefabDescription {
                prefab_id: 5,
                position: Vector3::new(-2., 0., 2.),
                velocity: Vector3::new(2., -1., 0.),
                half_extents: Vector3::new(0.5, 0.5, 0.5),
            },
        ];
        let features = [
            FeatureDescription {
                translate_x: true,
                translate_z: true,
                prefabs_ids: [1, 2, 3].as_ptr(),
                prefabs_ids_count: 3,
                spawns_per_second: 1.0,
                spawn_count: 10,
                trigger_position: 10.0,
                priority: 0,
            },
            FeatureDescription {
                translate_x: true,
                translate_z: true,
                prefabs_ids: [4].as_ptr(),
                prefabs_ids_count: 1,
                spawns_per_second: 1.0,
                spawn_count: 10,
                trigger_position: 20.0,
                priority: 0,
            },
            FeatureDescription {
                translate_x: true,
                translate_z: true,
                prefabs_ids: [5].as_ptr(),
                prefabs_ids_count: 1,
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
            spawn_barrier_y_coord: 8.0,
        };
        unsafe {
            let generated_entities_description = arc_level_generator::bind_generate(
                features.as_ptr(),
                features.len() as i32,
                prefabs.as_ptr(),
                prefabs.len() as i32,
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
