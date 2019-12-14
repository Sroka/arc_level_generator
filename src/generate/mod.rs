use ncollide3d::bounding_volume::AABB;
use nalgebra::{Vector3, Point3};
use std::collections::VecDeque;
use rand::prelude::RngCore;
use std::iter::FromIterator;
use rand::Rng;

mod types;
mod drain_upcoming_features;
mod trim_active_features;
mod trim_obstacles;
mod can_spawn_feature;
mod spawn_feature;

use self::types::{CollideableEntity, Feature, VisibleWorld};
use self::drain_upcoming_features::drain_upcoming_features;
use self::trim_active_features::trim_active_features;
use self::trim_obstacles::trim_obstacles;
use self::can_spawn_feature::can_spawn_feature;
use self::spawn_feature::spawn_feature;

const STEP: f32 = 0.1;

fn generate(rng: &mut impl RngCore,
            features: &[Feature]) {
    let mut upcoming_features: VecDeque<Feature> = VecDeque::from_iter(features.iter().cloned());
    let mut active_features: VecDeque<Feature> = VecDeque::new();

    let mut generated_entities: Vec<CollideableEntity> = Vec::new();
    let mut obstacles: VecDeque<CollideableEntity> = VecDeque::new();
    let world = VisibleWorld {
        world_bounds: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(9.0, 30.0, 9.0)),
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
        drain_upcoming_features(&mut upcoming_features, &mut active_features, distance_travelled);
        trim_active_features(&mut active_features);
        trim_obstacles(&mut obstacles, &world, time_travelled);

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
                spawn_feature(
                    &feature,
                    &mut obstacles,
                    &mut generated_entities,
                    time_travelled,
                );
            }
        }
    }
}


impl Feature<'_> {
    fn calculate_feature_spawn_bounds(&self) -> AABB<f32> {
        let min_x_prefab = self.prefabs.iter().min_by(|prefab1, prefab2| (prefab1.position.x - prefab1.bounding_box.half_extents().x).partial_cmp(&(prefab2.position.x - prefab2.bounding_box.half_extents().x)).unwrap()).unwrap();
        let min_y_prefab = self.prefabs.iter().min_by(|prefab1, prefab2| (prefab1.position.y - prefab1.bounding_box.half_extents().y).partial_cmp(&(prefab2.position.x - prefab2.bounding_box.half_extents().y)).unwrap()).unwrap();
        let min_z_prefab = self.prefabs.iter().min_by(|prefab1, prefab2| (prefab1.position.z - prefab1.bounding_box.half_extents().z).partial_cmp(&(prefab2.position.x - prefab2.bounding_box.half_extents().z)).unwrap()).unwrap();
        let max_x_prefab = self.prefabs.iter().min_by(|prefab1, prefab2| (prefab1.position.x + prefab1.bounding_box.half_extents().x).partial_cmp(&(prefab2.position.x + prefab2.bounding_box.half_extents().x)).unwrap()).unwrap();
        let max_y_prefab = self.prefabs.iter().min_by(|prefab1, prefab2| (prefab1.position.y + prefab1.bounding_box.half_extents().y).partial_cmp(&(prefab2.position.x + prefab2.bounding_box.half_extents().y)).unwrap()).unwrap();
        let max_z_prefab = self.prefabs.iter().min_by(|prefab1, prefab2| (prefab1.position.z + prefab1.bounding_box.half_extents().z).partial_cmp(&(prefab2.position.x + prefab2.bounding_box.half_extents().z)).unwrap()).unwrap();
        AABB::new(
            Point3::new(
                min_x_prefab.position.x - min_x_prefab.bounding_box.half_extents().x,
                min_y_prefab.position.x - min_y_prefab.bounding_box.half_extents().y,
                min_z_prefab.position.z - min_z_prefab.bounding_box.half_extents().z,
            ),
            Point3::new(
                max_x_prefab.position.x + max_x_prefab.bounding_box.half_extents().x,
                max_y_prefab.position.x + max_y_prefab.bounding_box.half_extents().y,
                max_z_prefab.position.z + max_z_prefab.bounding_box.half_extents().z,
            ),
        )
    }
}
