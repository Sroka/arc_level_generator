use ncollide3d::bounding_volume::AABB;
use nalgebra::{Vector3, Point3, Vector2};
use std::collections::VecDeque;
use rand::prelude::RngCore;
use std::iter::FromIterator;
use rand::Rng;

mod types;
mod drain_upcoming_features;
mod trim_active_features;
mod trim_obstacles;
mod calculate_feature_shift;
mod can_spawn_feature;
mod spawn_feature;

use self::types::{CollideableEntity, Feature, VisibleWorld};
use self::drain_upcoming_features::drain_upcoming_features;
use self::trim_active_features::trim_active_features;
use self::trim_obstacles::trim_obstacles;
use self::calculate_feature_shift::calculate_feature_shift;
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
            let feature_shift = calculate_feature_shift(rng, &world, feature);
            let can_spawn = can_spawn_feature(
                &feature,
                &obstacles,
                &world,
                time_travelled,
                &feature_shift,
            );
            if can_spawn {
                spawn_feature(
                    &feature,
                    &mut obstacles,
                    &mut generated_entities,
                    time_travelled,
                    &feature_shift,
                );
            }
        }
    }
}
