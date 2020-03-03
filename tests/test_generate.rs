#[cfg(test)]
mod tests {
    use ncollide3d::bounding_volume::AABB;
    use nalgebra::{Vector3, Point3};
    use self::arc_level_generator::{VisibleWorld, Prefab, Feature};

    extern crate arc_level_generator;

    #[test]
    fn test_generate() {
        let prefab0 = Prefab {
            prefab_id: 0,
            position: Vector3::new(0., 0., 0.),
            bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
            velocity: Vector3::new(0., -1., 0.),
        };
        let feature0 = Feature {
            translate_x: true,
            translate_z: true,
            prefabs: vec![prefab0],
            spawn_count: 1000,
            spawns_per_second: 1.0,
            trigger_position: 10.0,
            priority: 0,
            missed_spawns: 0,
        };

        let world = VisibleWorld {
            world_bounds: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(10., 10., 10.)),
            travel_speed: 4.0,
        };
        let generated_entities = arc_level_generator::generate(
            &world,
            &[feature0],
            &mut rand::thread_rng(),
        );
        for (index, entity) in generated_entities.iter().enumerate() {
            println!("Generated entitity {}: {:?}", index, entity)
        }
    }

    #[test]
    fn test_generate_large() {
        let prefab0 = Prefab {
            prefab_id: 0,
            position: Vector3::new(0., 0., 0.),
            bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(4.0, 4.0, 4.0)),
            velocity: Vector3::new(0., -1., 0.),
        };
        let prefab1 = Prefab {
            prefab_id: 1,
            position: Vector3::new(0., 0., 0.),
            bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(4.0, 4.0, 4.0)),
            velocity: Vector3::new(0., -1., 0.),
        };
        let feature0 = Feature {
            translate_x: true,
            translate_z: true,
            prefabs: vec![prefab0],
            spawn_count: 1,
            spawns_per_second: 1.0,
            trigger_position: 0.0,
            priority: 0,
            missed_spawns: 0,
        };
        let feature1 = Feature {
            translate_x: true,
            translate_z: true,
            prefabs: vec![prefab1],
            spawn_count: 1,
            spawns_per_second: 1.0,
            trigger_position: 0.0,
            priority: 0,
            missed_spawns: 0,
        };

        let world = VisibleWorld {
            world_bounds: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(4.5, 30., 4.5)),
            travel_speed: 4.0,
        };
        let generated_entities = arc_level_generator::generate(
            &world,
            &[feature0, feature1],
            &mut rand::thread_rng(),
        );
        for (index, entity) in generated_entities.iter().enumerate() {
            println!("Generated entity {}: {:?}", index, entity)
        }
    }

    #[test]
    fn test_generate_too_large_shifted() {
        let prefab0 = Prefab {
            prefab_id: 0,
            position: Vector3::new(0., 0., 0.),
            bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(5.0, 5.0, 5.0)),
            velocity: Vector3::new(0., -1., 0.),
        };
        let feature0 = Feature {
            translate_x: true,
            translate_z: true,
            prefabs: vec![prefab0],
            spawn_count: 10,
            spawns_per_second: 1.0,
            trigger_position: 10.0,
            priority: 0,
            missed_spawns: 0,
        };

        let world = VisibleWorld {
            world_bounds: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(3., 3., 3.)),
            travel_speed: 4.0,
        };
        let generated_entities = arc_level_generator::generate(
            &world,
            &[feature0],
            &mut rand::thread_rng(),
        );
        for (index, entity) in generated_entities.iter().enumerate() {
            println!("Generated entitity {}: {:?}", index, entity)
        }
    }
}