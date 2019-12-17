use std::collections::VecDeque;
use super::types::{Feature};

pub fn drain_upcoming_features(upcoming_features: &mut VecDeque<Feature>,
                                   active_features: &mut VecDeque<Feature>,
                                   distance_travelled: f32,
) {
    if upcoming_features.is_empty() {
        return;
    }
    upcoming_features.retain(|feature| {
        if feature.trigger_position - feature.priority as f32 <= distance_travelled {
            active_features.push_back(feature.clone());
            false
        } else {
            true
        }
    })
}

#[cfg(test)]
mod tests {
    use crate::generator::types::{Feature, Prefab};
    use super::drain_upcoming_features;

    use std::collections::VecDeque;
    use std::iter::FromIterator;

    use nalgebra::{Vector3, Point3};
    use ncollide3d::bounding_volume::AABB;

    #[test]
    fn test_drain_upcoming_features() {
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
            missed_spawns: 0,
        };
        let feature1 = Feature {
            trigger_position: 100.0,
            priority: 0,
            ..feature0
        };
        let feature2 = Feature {
            trigger_position: 110.0,
            priority: 15,
            ..feature0
        };
        let feature3 = Feature {
            trigger_position: 110.0,
            priority: 5,
            ..feature0
        };

        let mut upcoming_features: VecDeque<Feature> = VecDeque::from_iter([feature0.clone(), feature1.clone(), feature2.clone(), feature3.clone()].iter().cloned());
        let mut active_features: VecDeque<Feature> = VecDeque::new();
        let mut distance_travelled = 0.0_f32;


        drain_upcoming_features(
            &mut upcoming_features,
            &mut active_features,
            distance_travelled,
        );

        let expected: Vec<Feature> = Vec::new();
        assert!(active_features.iter().eq(expected.iter()));

        distance_travelled = 11.0;

        drain_upcoming_features(
            &mut upcoming_features,
            &mut active_features,
            distance_travelled,
        );

        let expected = [feature0.clone()];
        assert!(active_features.iter().eq(expected.iter()));

        distance_travelled = 98.0;

        drain_upcoming_features(
            &mut upcoming_features,
            &mut active_features,
            distance_travelled,
        );

        let expected = [feature0.clone(), feature2.clone()];
        assert!(active_features.iter().eq(expected.iter()));

        distance_travelled = 102.0;

        drain_upcoming_features(
            &mut upcoming_features,
            &mut active_features,
            distance_travelled,
        );

        let expected = [feature0.clone(), feature2.clone(), feature1.clone()];
        assert!(active_features.iter().eq(expected.iter()));

        distance_travelled = 107.0;

        drain_upcoming_features(
            &mut upcoming_features,
            &mut active_features,
            distance_travelled,
        );

        let expected = [feature0.clone(), feature2.clone(), feature1.clone(), feature3.clone()];
        assert!(active_features.iter().eq(expected.iter()));
    }
}
