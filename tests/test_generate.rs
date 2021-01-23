#[cfg(test)]
mod tests {
    use ncollide3d::bounding_volume::AABB;
    use nalgebra::{Vector3, Point3, Vector2, UnitQuaternion, Unit};
    use self::arc_level_generator::{VisibleWorld, Prefab, Feature, Movement};

    extern crate arc_level_generator;

    #[test]
    fn test_generate_large_quantities() {
        let prefab0 = Prefab {
            prefab_id: 0,
            position: Vector3::new(0., 0., 0.),
            rotation: UnitQuaternion::identity(),
            bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
            movement: Movement {
                baseline_velocity: Vector3::new(0., 0., -1.),
                arcs_plane_normal: Unit::new_normalize(Vector3::new(1.,0.,0.)),
                approach_arc_angle: 0.0,
                approach_arc_center_distance: 0.0,
                approach_arc_radius: 0.0,
                approach_rotation_strength: 0.,
                departure_arc_angle: 0.0,
                departure_arc_center_distance: 0.0,
                departure_arc_radius: 0.0,
                departure_rotation_strength: 0.0
            },
        };
        let prefab1 = Prefab {
            prefab_id: 0,
            position: Vector3::new(4., 2., 0.),
            rotation: UnitQuaternion::identity(),
            bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
            movement: Movement {
                baseline_velocity: Vector3::new(0., 0., -1.),
                arcs_plane_normal: Unit::new_normalize(Vector3::new(1.,0.,0.)),
                approach_arc_angle: 0.0,
                approach_arc_center_distance: 0.0,
                approach_arc_radius: 0.0,
                approach_rotation_strength: 0.,
                departure_arc_angle: 0.0,
                departure_arc_center_distance: 0.0,
                departure_arc_radius: 0.0,
                departure_rotation_strength: 0.0
            },
        };
        let prefab2 = Prefab {
            prefab_id: 0,
            position: Vector3::new(0., 3., 0.),
            rotation: UnitQuaternion::identity(),
            bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
            movement: Movement {
                baseline_velocity: Vector3::new(0., 0., -1.),
                arcs_plane_normal: Unit::new_normalize(Vector3::new(1.,0.,0.)),
                approach_arc_angle: 0.0,
                approach_arc_center_distance: 0.0,
                approach_arc_radius: 0.0,
                approach_rotation_strength: 0.,
                departure_arc_angle: 0.0,
                departure_arc_center_distance: 0.0,
                departure_arc_radius: 0.0,
                departure_rotation_strength: 0.0
            },
        };
        let feature0 = Feature {
            translate_x: true,
            translate_x_using_bounds: false,
            translate_x_bounds: Vector2::new(0., 0.),
            translate_y: true,
            translate_y_using_bounds: false,
            translate_y_bounds: Vector2::new(0., 0.),
            prefabs: vec![prefab0, prefab1, prefab2],
            spawn_count: 100,
            spawn_period: 1.0,
            is_spawn_period_strict: false,
            trigger_time: 10.0,
            priority: 0,
            missed_spawns: 0,
            last_spawn_attempt: 0.0,
            translate_z: 0.0
        };

        let world = VisibleWorld {
            world_bounds: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(10., 10., 10.)),
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
    fn test_generate() {
        let prefab0 = Prefab {
            prefab_id: 0,
            position: Vector3::new(0., 0., 0.),
            rotation: UnitQuaternion::identity(),
            bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
            movement: Movement {
                baseline_velocity: Vector3::new(0., 0., -1.),
                arcs_plane_normal: Unit::new_normalize(Vector3::new(1.,0.,0.)),
                approach_arc_angle: 0.0,
                approach_arc_center_distance: 0.0,
                approach_arc_radius: 0.0,
                approach_rotation_strength: 0.,
                departure_arc_angle: 0.0,
                departure_arc_center_distance: 0.0,
                departure_arc_radius: 0.0,
                departure_rotation_strength: 0.0
            },
        };
        let feature0 = Feature {
            translate_x: true,
            translate_x_using_bounds: false,
            translate_x_bounds: Vector2::new(0., 0.),
            translate_y: true,
            translate_y_using_bounds: false,
            translate_y_bounds: Vector2::new(0., 0.),
            prefabs: vec![prefab0],
            spawn_count: 10,
            spawn_period: 1.0,
            is_spawn_period_strict: false,
            trigger_time: 10.0,
            priority: 0,
            missed_spawns: 0,
            last_spawn_attempt: 0.0,
            translate_z: 0.0
        };

        let world = VisibleWorld {
            world_bounds: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(10., 10., 10.)),
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
            rotation: UnitQuaternion::identity(),
            bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(4.0, 4.0, 4.0)),
            movement: Movement {
                baseline_velocity: Vector3::new(0., -1., 0.),
                arcs_plane_normal: Unit::new_normalize(Vector3::new(1.,0.,0.)),
                approach_arc_angle: 0.0,
                approach_arc_center_distance: 0.0,
                approach_arc_radius: 0.0,
                approach_rotation_strength: 0.,
                departure_arc_angle: 0.0,
                departure_arc_center_distance: 0.0,
                departure_arc_radius: 0.0,
                departure_rotation_strength: 0.0
            },
        };
        let prefab1 = Prefab {
            prefab_id: 1,
            position: Vector3::new(0., 0., 0.),
            rotation: UnitQuaternion::identity(),
            bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(4.0, 4.0, 4.0)),
            movement: Movement {
                baseline_velocity: Vector3::new(0., -1., 0.),
                arcs_plane_normal: Unit::new_normalize(Vector3::new(1.,0.,0.)),
                approach_arc_angle: 0.0,
                approach_arc_center_distance: 0.0,
                approach_arc_radius: 0.0,
                approach_rotation_strength: 0.,
                departure_arc_angle: 0.0,
                departure_arc_center_distance: 0.0,
                departure_arc_radius: 0.0,
                departure_rotation_strength: 0.0
            },
        };
        let feature0 = Feature {
            translate_x: true,
            translate_x_using_bounds: false,
            translate_x_bounds: Vector2::new(0., 0.),
            translate_y: true,
            translate_y_using_bounds: false,
            translate_y_bounds: Vector2::new(0., 0.),
            prefabs: vec![prefab0],
            spawn_count: 1,
            spawn_period: 1.0,
            trigger_time: 0.0,
            priority: 0,
            missed_spawns: 0,
            is_spawn_period_strict: false,
            last_spawn_attempt: 0.0,
            translate_z: 0.0
        };
        let feature1 = Feature {
            translate_x: true,
            translate_x_using_bounds: false,
            translate_x_bounds: Vector2::new(0., 0.),
            translate_y: true,
            translate_y_using_bounds: false,
            translate_y_bounds: Vector2::new(0., 0.),
            prefabs: vec![prefab1],
            spawn_count: 1,
            spawn_period: 1.0,
            trigger_time: 0.0,
            priority: 0,
            missed_spawns: 0,
            is_spawn_period_strict: false,
            last_spawn_attempt: 0.0,
            translate_z: 0.0
        };

        let world = VisibleWorld {
            world_bounds: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(4.5, 30., 4.5)),
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
            rotation: UnitQuaternion::identity(),
            bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(5.0, 5.0, 5.0)),
            movement: Movement {
                baseline_velocity: Vector3::new(0., -1., 0.),
                arcs_plane_normal: Unit::new_normalize(Vector3::new(1.,0.,0.)),
                approach_arc_angle: 0.0,
                approach_arc_center_distance: 0.0,
                approach_arc_radius: 0.0,
                approach_rotation_strength: 0.,
                departure_arc_angle: 0.0,
                departure_arc_center_distance: 0.0,
                departure_arc_radius: 0.0,
                departure_rotation_strength: 0.0
            },
        };
        let feature0 = Feature {
            translate_x: true,
            translate_x_using_bounds: false,
            translate_x_bounds: Vector2::new(0., 0.),
            translate_y: false,
            translate_y_using_bounds: false,
            translate_y_bounds: Vector2::new(0., 0.),
            prefabs: vec![prefab0],
            spawn_count: 10,
            spawn_period: 0.1,
            trigger_time: 0.0,
            priority: 0,
            missed_spawns: 0,
            is_spawn_period_strict: false,
            last_spawn_attempt: 0.0,
            translate_z: 0.0
        };

        let world = VisibleWorld {
            world_bounds: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(6., 30., 6.)),
        };
        let generated_entities = arc_level_generator::generate(
            &world,
            &[feature0],
            &mut rand::thread_rng(),
        );
        for (index, entity) in generated_entities.iter().enumerate() {
            println!("Generated entitity {}: {:?}", index, entity.spawn_time)
        }
    }

    #[test]
    fn test_generate_rotated() {
        let prefab0 = Prefab {
            prefab_id: 0,
            position: Vector3::new(0., 0., 0.),
            rotation: UnitQuaternion::from_euler_angles( 0., std::f32::consts::FRAC_PI_2, 0.),
            bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(8.0, 0.5, 0.5)),
            movement: Movement {
                baseline_velocity: Vector3::new(0., 0., -1.),
                arcs_plane_normal: Unit::new_normalize(Vector3::new(1.,0.,0.)),
                approach_arc_angle: 0.0,
                approach_arc_center_distance: 0.0,
                approach_arc_radius: 0.0,
                approach_rotation_strength: 0.,
                departure_arc_angle: 0.0,
                departure_arc_center_distance: 0.0,
                departure_arc_radius: 0.0,
                departure_rotation_strength: 0.0
            },
        };
        let feature0 = Feature {
            translate_x: false,
            translate_x_using_bounds: false,
            translate_x_bounds: Vector2::new(0., 0.),
            translate_y: false,
            translate_y_using_bounds: false,
            translate_y_bounds: Vector2::new(0., 0.),
            prefabs: vec![prefab0],
            spawn_count: 10,
            spawn_period: 0.1,
            trigger_time: 0.0,
            priority: 0,
            missed_spawns: 0,
            is_spawn_period_strict: false,
            last_spawn_attempt: 0.0,
            translate_z: 0.0
        };

        let world = VisibleWorld {
            world_bounds: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(10., 10., 50.)),
        };
        let generated_entities = arc_level_generator::generate(
            &world,
            &[feature0],
            &mut rand::thread_rng(),
        );
        for (index, entity) in generated_entities.iter().enumerate() {
            println!("Generated entity {}: {:?}", index, entity)
        }
    }

    #[test]
    fn test_generate_strict_period() {
        let prefab0 = Prefab {
            prefab_id: 0,
            position: Vector3::new(0., 0., 0.),
            rotation: UnitQuaternion::from_euler_angles( 0., std::f32::consts::FRAC_PI_2, 0.),
            bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(8.0, 0.5, 0.5)),
            movement: Movement {
                baseline_velocity: Vector3::new(0., 0., -1.),
                arcs_plane_normal: Unit::new_normalize(Vector3::new(1.,0.,0.)),
                approach_arc_angle: 0.0,
                approach_arc_center_distance: 0.0,
                approach_arc_radius: 0.0,
                approach_rotation_strength: 0.,
                departure_arc_angle: 0.0,
                departure_arc_center_distance: 0.0,
                departure_arc_radius: 0.0,
                departure_rotation_strength: 0.0
            }
        };
        let feature0 = Feature {
            translate_x: false,
            translate_x_using_bounds: false,
            translate_x_bounds: Vector2::new(0., 0.),
            translate_y: false,
            translate_y_using_bounds: false,
            translate_y_bounds: Vector2::new(0., 0.),
            prefabs: vec![prefab0],
            spawn_count: 10,
            spawn_period: 5.0,
            trigger_time: 0.0,
            priority: 0,
            missed_spawns: 0,
            is_spawn_period_strict: true,
            last_spawn_attempt: 0.0,
            translate_z: 0.0
        };

        let world = VisibleWorld {
            world_bounds: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(10., 10., 50.)),
        };
        let generated_entities = arc_level_generator::generate(
            &world,
            &[feature0],
            &mut rand::thread_rng(),
        );
        for (index, entity) in generated_entities.iter().enumerate() {
            println!("Generated entity {}: {:?}", index, entity)
        }
    }

    #[test]
    fn test_generate_large_rotated() {
        let prefab0 = Prefab {
            prefab_id: 0,
            position: Vector3::new(0., -9.38, 0.),
            rotation: UnitQuaternion::from_euler_angles( -std::f32::consts::FRAC_PI_2, 0., std::f32::consts::PI),
            bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(8.0, 0.5, 0.5)),
            movement: Movement {
                baseline_velocity: Vector3::new(0., 0., -1.),
                arcs_plane_normal: Unit::new_normalize(Vector3::new(1.,0.,0.)),
                approach_arc_angle: 0.0,
                approach_arc_center_distance: 0.0,
                approach_arc_radius: 0.0,
                approach_rotation_strength: 0.,
                departure_arc_angle: 0.0,
                departure_arc_center_distance: 0.0,
                departure_arc_radius: 0.0,
                departure_rotation_strength: 0.0
            },
        };
        let feature0 = Feature {
            translate_x: false,
            translate_x_using_bounds: false,
            translate_x_bounds: Vector2::new(0., 0.),
            translate_y: false,
            translate_y_using_bounds: false,
            translate_y_bounds: Vector2::new(0., 0.),
            prefabs: vec![prefab0],
            spawn_count: 10,
            spawn_period: 5.0,
            trigger_time: 0.0,
            priority: 0,
            missed_spawns: 0,
            is_spawn_period_strict: true,
            last_spawn_attempt: 0.0,
            translate_z: 0.0
        };

        let world = VisibleWorld {
            world_bounds: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(10., 10., 50.)),
        };
        let generated_entities = arc_level_generator::generate(
            &world,
            &[feature0],
            &mut rand::thread_rng(),
        );
        for (index, entity) in generated_entities.iter().enumerate() {
            println!("Generated entity {}: {:?}", index, entity)
        }
    }

    #[test]
    fn test_generate_large_2() {
        let prefab0 = Prefab {
            prefab_id: 0,
            position: Vector3::new(0., -20., 0.),
            rotation: UnitQuaternion::identity(),
            bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(7.5, 7., 11.)),
            movement: Movement {
                baseline_velocity: Vector3::new(0., 0., -1.),
                arcs_plane_normal: Unit::new_normalize(Vector3::new(1.,0.,0.)),
                approach_arc_angle: 0.0,
                approach_arc_center_distance: 0.0,
                approach_arc_radius: 0.0,
                approach_rotation_strength: 0.,
                departure_arc_angle: 0.0,
                departure_arc_center_distance: 0.0,
                departure_arc_radius: 0.0,
                departure_rotation_strength: 0.0
            },
        };
        let feature0 = Feature {
            translate_x: false,
            translate_x_using_bounds: false,
            translate_x_bounds: Vector2::new(0., 0.),
            translate_y: true,
            translate_y_using_bounds: true,
            translate_y_bounds: Vector2::new(-15., 6.2),
            prefabs: vec![prefab0],
            spawn_count: 10,
            spawn_period: 1.0,
            trigger_time: 0.0,
            priority: 0,
            missed_spawns: 0,
            is_spawn_period_strict: true,
            last_spawn_attempt: 0.0,
            translate_z: 0.0
        };

        let world = VisibleWorld {
            world_bounds: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(50., 50., 50.)),
        };
        let generated_entities = arc_level_generator::generate(
            &world,
            &[feature0],
            &mut rand::thread_rng(),
        );
        for (index, entity) in generated_entities.iter().enumerate() {
            println!("Generated entity {}: {:?}", index, entity.spawn_position.xy())
        }
    }


    // TODO: This bug is still happening
    #[test]
    fn test_generate_hang() {
        let prefab0 = Prefab {
            prefab_id: 0,
            position: Vector3::new(0., 0., 0.),
            rotation: UnitQuaternion::identity(),
            bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(22.05775, 13.11225, 9.712485)),
            // bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(22.05775, 13.11225, 9.5)),
            movement: Movement {
                baseline_velocity: Vector3::new(0., 0., -8.),
                arcs_plane_normal: Unit::new_normalize(Vector3::new(1.,0.,0.)),
                approach_arc_angle: 0.0,
                approach_arc_center_distance: 0.0,
                approach_arc_radius: 0.0,
                approach_rotation_strength: 0.,
                departure_arc_angle: 0.0,
                departure_arc_center_distance: 0.0,
                departure_arc_radius: 0.0,
                departure_rotation_strength: 0.0
            },
        };
        let feature0 = Feature {
            translate_x: false,
            translate_x_using_bounds: false,
            translate_x_bounds: Vector2::new(0., 0.),
            translate_y: false,
            translate_y_using_bounds: false,
            translate_y_bounds: Vector2::new(0., 0.),
            prefabs: vec![prefab0],
            spawn_count: 2,
            spawn_period: 1.0,
            is_spawn_period_strict: false,
            trigger_time: 0.0,
            priority: 0,
            missed_spawns: 0,
            last_spawn_attempt: 0.0,
            translate_z: 0.0
        };

        let world = VisibleWorld {
            world_bounds: AABB::from_half_extents(Point3::new(0., 0., 175.), Vector3::new(50., 50., 200.)),
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
    fn test_generate_tilt_simple() {
        let prefab0 = Prefab {
            prefab_id: 0,
            position: Vector3::new(0., 0., 0.),
            rotation: UnitQuaternion::identity(),
            bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
            movement: Movement {
                baseline_velocity: Vector3::new(0., 0., -8.),
                arcs_plane_normal: Unit::new_normalize(Vector3::new(1.,0.,0.)),
                approach_arc_angle: 45.0_f32.to_radians(),
                approach_arc_center_distance: 0.0,
                approach_arc_radius: 50.0,
                approach_rotation_strength: 1.,
                departure_arc_angle: 0.0,
                departure_arc_center_distance: 0.0,
                departure_arc_radius: 0.0,
                departure_rotation_strength: 0.0
            },
        };
        let feature0 = Feature {
            translate_x: false,
            translate_x_using_bounds: false,
            translate_x_bounds: Vector2::new(0., 0.),
            translate_y: false,
            translate_y_using_bounds: false,
            translate_y_bounds: Vector2::new(0., 0.),
            prefabs: vec![prefab0],
            spawn_count: 1,
            spawn_period: 1.0,
            is_spawn_period_strict: false,
            trigger_time: 0.0,
            priority: 0,
            missed_spawns: 0,
            last_spawn_attempt: 0.0,
            translate_z: 0.0
        };

        let world = VisibleWorld {
            world_bounds: AABB::from_half_extents(Point3::new(0., 0., 175.), Vector3::new(50., 50., 200.)),
        };
        let generated_entities = arc_level_generator::generate(
            &world,
            &[feature0],
            &mut rand::thread_rng(),
        );
        for (index, entity) in generated_entities.iter().enumerate() {
            dbg!(entity.spawn_rotation.euler_angles());
            println!("Generated entitity {}: {:?}", index, entity)
        }
    }

    #[test]
    fn test_generate_priority() {
        let prefab0 = Prefab {
            prefab_id: 0,
            position: Vector3::new(0., 0., 0.),
            rotation: UnitQuaternion::identity(),
            bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
            movement: Movement {
                baseline_velocity: Vector3::new(0., 0., -1.),
                arcs_plane_normal: Unit::new_normalize(Vector3::new(1.,0.,0.)),
                approach_arc_angle: 0.0,
                approach_arc_center_distance: 0.0,
                approach_arc_radius: 0.0,
                approach_rotation_strength: 0.,
                departure_arc_angle: 0.0,
                departure_arc_center_distance: 0.0,
                departure_arc_radius: 0.0,
                departure_rotation_strength: 0.0
            },
        };
        let feature0 = Feature {
            translate_x: false,
            translate_x_using_bounds: false,
            translate_x_bounds: Vector2::new(0., 0.),
            translate_y: false,
            translate_y_using_bounds: false,
            translate_y_bounds: Vector2::new(0., 0.),
            prefabs: vec![prefab0],
            spawn_count: 10,
            spawn_period: 1.0,
            is_spawn_period_strict: false,
            trigger_time: 0.0,
            priority: 1000,
            missed_spawns: 0,
            last_spawn_attempt: 0.0,
            translate_z: 0.0
        };
        let prefab1 = Prefab {
            prefab_id: 0,
            position: Vector3::new(0., 20., 0.),
            rotation: UnitQuaternion::identity(),
            bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
            movement: Movement {
                baseline_velocity: Vector3::new(0., 0., -1.),
                arcs_plane_normal: Unit::new_normalize(Vector3::new(1.,0.,0.)),
                approach_arc_angle: 0.0,
                approach_arc_center_distance: 0.0,
                approach_arc_radius: 0.0,
                approach_rotation_strength: 0.,
                departure_arc_angle: 0.0,
                departure_arc_center_distance: 0.0,
                departure_arc_radius: 0.0,
                departure_rotation_strength: 0.0
            },
        };
        let feature1 = Feature {
            translate_x: false,
            translate_x_using_bounds: false,
            translate_x_bounds: Vector2::new(0., 0.),
            translate_y: false,
            translate_y_using_bounds: false,
            translate_y_bounds: Vector2::new(0., 0.),
            prefabs: vec![prefab1],
            spawn_count: 10,
            spawn_period: 1.0,
            is_spawn_period_strict: false,
            trigger_time: 0.0,
            priority: 0,
            missed_spawns: 0,
            last_spawn_attempt: 0.0,
            translate_z: 10.0
        };

        let world = VisibleWorld {
            world_bounds: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(10., 10., 10.)),
        };
        let generated_entities = arc_level_generator::generate(
            &world,
            &[feature0, feature1],
            &mut rand::thread_rng(),
        );
        for (index, entity) in generated_entities.iter().enumerate() {
            println!("Generated entitity {}: {:?}", index, entity)
        }
    }
}