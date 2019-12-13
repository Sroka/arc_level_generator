use ncollide3d::bounding_volume::AABB;
use ncollide3d::query;
use crate::types::{CollideableEntity, Feature, Prefab, VisibleWorld};
use nalgebra::{Vector3, Point, Point3, Isometry3};
use std::collections::VecDeque;
use rand::prelude::RngCore;
use std::iter::FromIterator;
use std::borrow::BorrowMut;
use rand::Rng;
use ncollide3d::shape::Cuboid;

const STEP: f32 = 0.1;

fn generate(rng: &mut impl RngCore,
            features: &[Feature]) {
    let mut upcoming_features: VecDeque<&Feature> = VecDeque::from_iter(features);
    let mut active_features: VecDeque<&Feature> = VecDeque::new();

    let mut generated_entities: Vec<&CollideableEntity> = Vec::new();
    let mut obstacles: VecDeque<&CollideableEntity> = VecDeque::new();
    let world = VisibleWorld {
        half_extents: Vector3::new(9.0, 30.0, 9.0),
        travel_speed: 4.0,
        spawn_barrier: 2.0,
    };

    let mut distance_travelled = 0.0_f32;

    'main_loop: loop {
        if active_features.is_empty() && upcoming_features.is_empty() {
            // That's it. We generated everything
            break 'main_loop;
        }
        distance_travelled += STEP;
        let time_travelled = distance_travelled / world.travel_speed;
        drain_upcoming_features(upcoming_features.borrow_mut(), active_features.borrow_mut(), distance_travelled);
        trim_active_features(active_features.borrow_mut());
        trim_obstacles(obstacles.borrow_mut(), &world, time_travelled);

        'features_loop: for feature in &active_features {
            if !rng.gen_bool((STEP / feature.spawns_per_second) as f64) {
                continue;
            }
            let can_spawn = can_spawn_feature(
                feature.prefabs,
                &obstacles,
                &world,
                time_travelled,
            );
            if can_spawn {
                for prefab in feature.prefabs {}
            }
        }
    }
}

fn drain_upcoming_features<'a>(upcoming_features: &mut VecDeque<&'a Feature<'a>>,
                               active_features: &mut VecDeque<&'a Feature<'a>>,
                               distance_travelled: f32,
) {
    if upcoming_features.is_empty() {
        return;
    }
    upcoming_features.retain(|feature| {
        if feature.trigger_position - feature.priority <= distance_travelled {
            active_features.push_back(*feature);
            false
        } else {
            true
        }
    })
}

fn trim_active_features(
    active_features: &mut VecDeque<&Feature>,
) {
    active_features.retain(|feature| feature.spawn_count > 0);
}

fn trim_obstacles(
    active_entities: &mut VecDeque<&CollideableEntity>,
    world: &VisibleWorld,
    time_travelled: f32,
) {
    active_entities.retain(|entity| {
        let entity_travel_time = time_travelled - entity.spawn_time;
        let current_entity_position = entity.spawn_position + entity.velocity * entity_travel_time;
        AABB::from_half_extents(Point3::new(0.0, 0.0, 0.0), world.half_extents).contains_local_point(&Point::from(current_entity_position))
    });
}

fn can_spawn_feature(
    feature_prefabs: &[Prefab],
    obstacles: &VecDeque<&CollideableEntity>,
    world: &VisibleWorld,
    time_travelled: f32,
) -> bool {
    'prefabs_loop: for prefab in feature_prefabs {
        'obstacles_loop: for existing_entity in obstacles {
            let time_of_impact = query::time_of_impact(
                &Isometry3::new(prefab.position, nalgebra::zero()),
                &prefab.velocity,
                &Cuboid::new(prefab.bounding_box),
                &Isometry3::new(existing_entity.spawn_position, nalgebra::zero()),
                &existing_entity.velocity,
                &Cuboid::new(existing_entity.bounding_box),
                100.0,
                0.0,
            );
            match time_of_impact {
                Some(_) => {
                    return false;
                }
                _ => {}
            }
        }
    }
    return true;
}


impl Feature<'_> {
    fn calculate_feature_spawn_bounds(&self) -> AABB<f32> {
        let min_x_prefab = self.prefabs.iter().min_by(|prefab1, prefab2| (prefab1.position.x - prefab1.bounding_box.x / 2.0).partial_cmp(&(prefab2.position.x - prefab2.bounding_box.x)).unwrap()).unwrap();
        let min_y_prefab = self.prefabs.iter().min_by(|prefab1, prefab2| (prefab1.position.y - prefab1.bounding_box.y / 2.0).partial_cmp(&(prefab2.position.x - prefab2.bounding_box.y)).unwrap()).unwrap();
        let min_z_prefab = self.prefabs.iter().min_by(|prefab1, prefab2| (prefab1.position.z - prefab1.bounding_box.z / 2.0).partial_cmp(&(prefab2.position.x - prefab2.bounding_box.z)).unwrap()).unwrap();
        let max_x_prefab = self.prefabs.iter().min_by(|prefab1, prefab2| (prefab1.position.x + prefab1.bounding_box.x / 2.0).partial_cmp(&(prefab2.position.x + prefab2.bounding_box.x)).unwrap()).unwrap();
        let max_y_prefab = self.prefabs.iter().min_by(|prefab1, prefab2| (prefab1.position.y + prefab1.bounding_box.y / 2.0).partial_cmp(&(prefab2.position.x + prefab2.bounding_box.y)).unwrap()).unwrap();
        let max_z_prefab = self.prefabs.iter().min_by(|prefab1, prefab2| (prefab1.position.z + prefab1.bounding_box.z / 2.0).partial_cmp(&(prefab2.position.x + prefab2.bounding_box.z)).unwrap()).unwrap();
        AABB::new(
            Point3::new(
                min_x_prefab.position.x - min_x_prefab.bounding_box.x,
                min_y_prefab.position.x - min_y_prefab.bounding_box.x,
                min_z_prefab.position.z - min_z_prefab.bounding_box.z,
            ),
            Point3::new(
                max_x_prefab.position.x + max_x_prefab.bounding_box.x,
                max_y_prefab.position.x + max_y_prefab.bounding_box.x,
                max_z_prefab.position.z + max_z_prefab.bounding_box.z,
            ),
        )
    }
}

#[cfg(test)]
mod tests {
    use crate::types::{Prefab, Feature};
    use nalgebra::Vector3;
    use crate::min_loop::{trim_active_features, drain_upcoming_features};
    use std::collections::VecDeque;
    use std::iter::FromIterator;
    use std::borrow::BorrowMut;

    #[test]
    fn test_drain_upcoming_features() {
        let prefab0 = Prefab {
            prefab_id: 0,
            position: Vector3::new(0.0, 0.0, 0.0),
            bounding_box: Vector3::new(0.0, 0.0, 0.0),
            velocity: Vector3::new(1.0, 1.0, 1.0),
        };
        let feature0 = Feature {
            translate_x: false,
            translate_z: false,
            prefabs: &[prefab0],
            spawn_count: 1,
            spawns_per_second: 1.0,
            trigger_position: 10.0,
            priority: 0.0,
        };
        let feature1 = Feature {
            trigger_position: 100.0,
            priority: 0.0,
            ..feature0
        };
        let feature2 = Feature {
            trigger_position: 110.0,
            priority: 15.0,
            ..feature0
        };
        let feature3 = Feature {
            trigger_position: 110.0,
            priority: 5.0,
            ..feature0
        };

        let mut upcoming_features: VecDeque<&Feature> = VecDeque::from_iter([&feature0, &feature1, &feature2, &feature3].iter().cloned());
        let mut active_features: VecDeque<&Feature> = VecDeque::new();
        let mut distance_travelled = 0.0_f32;


        drain_upcoming_features(
            &mut upcoming_features,
            &mut active_features,
            distance_travelled,
        );

        let expected: Vec<&Feature> = Vec::new();
        assert!(active_features.iter().eq(expected.iter()));

       distance_travelled = 11.0;

        drain_upcoming_features(
            &mut upcoming_features,
            &mut active_features,
            distance_travelled,
        );

        let expected = [&feature0];
        assert!(active_features.iter().eq(expected.iter()));

        distance_travelled = 98.0;

        drain_upcoming_features(
            &mut upcoming_features,
            &mut active_features,
            distance_travelled,
        );

        let expected = [&feature0, &feature2];
        assert!(active_features.iter().eq(expected.iter()));

        distance_travelled = 102.0;

        drain_upcoming_features(
            &mut upcoming_features,
            &mut active_features,
            distance_travelled,
        );

        let expected = [&feature0, &feature2, &feature1];
        assert!(active_features.iter().eq(expected.iter()));

        distance_travelled = 107.0;

        drain_upcoming_features(
            &mut upcoming_features,
            &mut active_features,
            distance_travelled,
        );

        let expected = [&feature0, &feature2, &feature1, &feature3];
        assert!(active_features.iter().eq(expected.iter()));
    }

    #[test]
    fn test_trim_active_features() {
        let prefab0 = Prefab {
            prefab_id: 0,
            position: Vector3::new(0.0, 0.0, 0.0),
            bounding_box: Vector3::new(0.0, 0.0, 0.0),
            velocity: Vector3::new(1.0, 1.0, 1.0),
        };
        let feature0 = Feature {
            translate_x: false,
            translate_z: false,
            prefabs: &[prefab0],
            spawn_count: 1,
            spawns_per_second: 1.0,
            trigger_position: 10.0,
            priority: 0.0,
        };
        let feature1 = Feature {
            spawn_count: 0,
            ..feature0
        };
        let feature2 = Feature {
            spawn_count: 10,
            ..feature0
        };
        let feature3 = Feature {
            spawn_count: 100,
            ..feature0
        };
        let mut features = VecDeque::from_iter([&feature0, &feature1, &feature2, &feature3].iter().cloned());

        trim_active_features(&mut features);
        assert!(features.iter().eq([&feature0, &feature2, &feature3].iter()));
    }
}
