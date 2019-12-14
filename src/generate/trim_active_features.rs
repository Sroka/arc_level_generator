use std::collections::VecDeque;
use super::types::Feature;

pub fn trim_active_features(
    active_features: &mut VecDeque<&Feature>,
) {
    active_features.retain(|feature| feature.spawn_count > 0);
}

#[cfg(test)]
mod tests {
    use crate::generate::types::{Feature, Prefab};
    use super::trim_active_features;

    use std::collections::VecDeque;
    use std::iter::FromIterator;

    use nalgebra::{Vector3, Point3};
    use ncollide3d::bounding_volume::AABB;

    #[test]
    fn test_trim_active_features() {
        let prefab0 = Prefab {
            prefab_id: 0,
            position: Vector3::new(0.0, 0.0, 0.0),
            bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
            velocity: Vector3::new(1.0, 1.0, 1.0),
        };
        let feature0 = Feature {
            translate_x: false,
            translate_z: false,
            prefabs: &[prefab0],
            spawn_count: 1,
            spawns_per_second: 1.0,
            trigger_position: 10.0,
            priority: 0,
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
