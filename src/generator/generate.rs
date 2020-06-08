use std::collections::VecDeque;
use crate::{Feature, CollideableEntity, VisibleWorld};
use std::iter::FromIterator;
use rand::{RngCore, Rng};
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
    let mut upcoming_features: VecDeque<Feature> = VecDeque::from_iter(features.iter().cloned());
    let mut active_features: VecDeque<Feature> = VecDeque::new();

    let mut generated_entities: Vec<CollideableEntity> = Vec::new();
    let mut obstacles: VecDeque<CollideableEntity> = VecDeque::new();

    let mut time_travelled = 0.0_f32;
    dbg!(features);

    'main_loop: loop {
        if active_features.is_empty() && upcoming_features.is_empty() {
            // That's it. We generated everything
            break 'main_loop;
        }
        time_travelled += STEP;
        drain_upcoming_features(&mut upcoming_features, &mut active_features, time_travelled);
        trim_active_features(&mut active_features);
        trim_obstacles(&mut obstacles, &world, time_travelled);

        'features_loop: for feature in &mut active_features {
            if !rng.gen_bool(((STEP * (1 + feature.missed_spawns) as f32 * feature.spawns_per_second) as f64).min(1.0)) {
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
                    &world,
                    &feature_shift,
                );
                feature.spawn_count = feature.spawn_count - 1;
                feature.missed_spawns = 0;
            } else {
                feature.missed_spawns = feature.missed_spawns + 1;
            }
        }
    }
    dbg!(&generated_entities);
    generated_entities
}
