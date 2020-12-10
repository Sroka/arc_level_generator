use std::collections::VecDeque;
use crate::{Feature, CollideableEntity, VisibleWorld};
use rand::{RngCore, Rng};
use rand::seq::SliceRandom;
use crate::generator::drain_upcoming_features::drain_upcoming_features;
use crate::generator::trim_active_features::trim_active_features;
use crate::generator::trim_obstacles::trim_obstacles;
use crate::generator::calculate_feature_shift::calculate_feature_shift;
use crate::generator::can_spawn_feature::can_spawn_feature;
use crate::generator::spawn_feature::spawn_feature;

const STEP: f32 = 0.025;

/// Randomly generates non-intersecting entities
///
/// * `world` - a bounded volume in which entities paths are non-intersecting. Outside of it there
///             is no guarantee that entities won't intersect
/// * `features` - a list of possible level features that can be spawned. It is guaranteed that
///             all of them will eventually be spawned before algorithm stops
/// * `rng` - random number generator used during generation
pub fn generate(
    world: &VisibleWorld,
    features: &[Feature],
    rng: &mut impl RngCore,
) -> Vec<CollideableEntity> {
    let mut upcoming_features: Vec<Feature> = Vec::from(features);
    let mut active_features: Vec<Feature> = Vec::new();

    let mut generated_entities: Vec<CollideableEntity> = Vec::new();
    let mut obstacles: VecDeque<CollideableEntity> = VecDeque::new();

    let mut time_travelled = 0.0_f32;

    'main_loop: loop {
        if active_features.is_empty() && upcoming_features.is_empty() {
            // That's it. We generated everything
            break 'main_loop;
        }
        time_travelled += STEP;
        drain_upcoming_features(&mut upcoming_features, &mut active_features, time_travelled);
        trim_active_features(&mut active_features);
        trim_obstacles(&mut obstacles, &world, time_travelled);
        active_features.shuffle(rng);

        for feature in &mut active_features {
            let should_try_spawning = if feature.is_spawn_period_strict {
                time_travelled > feature.last_spawn_attempt + feature.spawn_period
            } else {
                let chance_to_spawn = ((STEP * (1 + feature.missed_spawns) as f32 / feature.spawn_period) as f64).min(1.0);
                rng.gen_bool(chance_to_spawn)
            };
            if !should_try_spawning {
                continue;
            }
            let spawn_time = if feature.is_spawn_period_strict { feature.last_spawn_attempt + feature.spawn_period } else { time_travelled };
            let feature_shift = calculate_feature_shift(rng, &world, feature);
            let can_spawn = can_spawn_feature(
                &feature,
                &obstacles,
                &world,
                spawn_time,
                &feature_shift,
            );
            if can_spawn {
                spawn_feature(
                    &feature,
                    &mut obstacles,
                    &mut generated_entities,
                    spawn_time,
                    &world,
                    &feature_shift,
                );
                feature.spawn_count -= 1;
                feature.missed_spawns = 0;
                feature.last_spawn_attempt = spawn_time;
            } else {
                feature.missed_spawns += 1;
                feature.last_spawn_attempt = spawn_time;
            }
        }
    }
    generated_entities
}
