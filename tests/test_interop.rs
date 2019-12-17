#[cfg(test)]
mod tests {
    extern crate arc_level_generator;

    use self::arc_level_generator::{PrefabDescription, FeatureDescription, VisibleWorldDescription};
    use nalgebra::Vector3;

    #[test]
    fn test_interop() {
        let prefabs = [PrefabDescription {
            prefab_id: 1,
            position: Vector3::new(0., 0., 0.),
            velocity: Vector3::new(0., -1., 0.),
            half_extents: Vector3::new(0.5, 0.5, 0.5),
        }];
        let prefab_ids = [1];
        let features = [FeatureDescription {
            translate_x: false,
            translate_z: false,
            prefabs_ids: prefab_ids.as_ptr(),
            prefabs_ids_count: prefab_ids.len() as i32,
            spawns_per_second: 1.0,
            spawn_count: 1,
            trigger_position: 10.0,
            priority: 0
        }];
        let world = VisibleWorldDescription {
            position: Vector3::new(0., 0., 0.),
            half_extents: Vector3::new(10., 10., 10.),
            travel_speed: 4.0,
            spawn_barrier_y_coord: 8.0,
        };
        unsafe {
            let generated_entities = arc_level_generator::generate_entities(
                features.as_ptr(),
                features.len() as i32,
                prefabs.as_ptr(),
                prefabs.len() as i32,
                world,
            );

        }
    }
}