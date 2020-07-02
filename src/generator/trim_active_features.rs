use std::collections::VecDeque;
use super::types::Feature;

/// Removes features that can no longer spawn any entities
///
/// * `active_fatures` - features that are currently spawning
///
pub fn trim_active_features(
    active_features: &mut Vec<Feature>,
) {
    active_features.retain(|feature| feature.spawn_count > 0);
}

#[cfg(test)]
mod tests {
    use crate::generator::types::{Feature, Prefab};
    use super::trim_active_features;

    use std::collections::VecDeque;
    use std::iter::FromIterator;

    use nalgebra::{Vector3, Point3, Vector2, UnitQuaternion};
    use ncollide3d::bounding_volume::AABB;

    #[test]
    fn test_trim_active_features() {
        let prefab0 = Prefab {
            prefab_id: 0,
            position: Vector3::new(0.0, 0.0, 0.0),
            rotation: UnitQuaternion::identity(),
            bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
            velocity: Vector3::new(1.0, 1.0, 1.0),
        };
        let feature0 = Feature {
            translate_x: false,
            translate_x_using_bounds: false,
            translate_x_bounds: Vector2::new(0., 0.),
            translate_y: false,
            translate_y_using_bounds: false,
            translate_y_bounds: Vector2::new(0., 0.),
            prefabs: vec![prefab0.clone()],
            spawn_count: 1,
            spawn_period: 1.0,
            trigger_time: 10.0,
            priority: 0,
            missed_spawns: 0,
            is_spawn_period_strict: false,
            last_spawn_attempt: 0.0,
            translate_z: 0.0
        };
        let feature1 = Feature {
            spawn_count: 0,
            ..feature0.clone()
        };
        let feature2 = Feature {
            spawn_count: 10,
            ..feature0.clone()
        };
        let feature3 = Feature {
            spawn_count: 100,
            ..feature0.clone()
        };
        let mut features = vec![feature0.clone(), feature1.clone(), feature2.clone(), feature3.clone()];

        trim_active_features(&mut features);
        assert!(features.iter().eq([feature0.clone(), feature2.clone(), feature3.clone()].iter()));
    }
}
