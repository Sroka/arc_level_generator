use super::types::Feature;
use crate::VisibleWorld;
use nalgebra::Vector3;

/// Activated not yet active features that should start spawning at a given travelled distance
/// and adds them to active entities queue
///
/// * `upcoming_features` - features that didn't yet start to spawn
/// * `active_features` - features that are actively spawning
/// * `distance_travelled` - distance travelled in a given world
pub fn drain_upcoming_features(upcoming_features: &mut Vec<Feature>,
                               active_features: &mut Vec<Feature>,
                               world: &VisibleWorld,
                               time_travelled: f32,
) {
    if upcoming_features.is_empty() {
        return;
    }
    upcoming_features.retain(|feature| {
        let trigger_adjusted = feature.trigger_time - feature.max_approach_time(&world, &Vector3::new(0., 0., 0.)) - feature.priority as f32;
        if time_travelled >= trigger_adjusted {
            active_features.push(feature.clone());
            false
        } else {
            true
        }
    })
}

#[cfg(test)]
mod tests {
    use crate::generator::types::{Feature, Prefab, Movement};
    use super::drain_upcoming_features;

    use nalgebra::{Vector3, Point3, Vector2, UnitQuaternion, Unit};
    use ncollide3d::bounding_volume::AABB;
    use crate::VisibleWorld;

    #[test]
    fn test_drain_upcoming_features() {
        let world = VisibleWorld {
            world_bounds: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(20., 20., 20.)),
        };
        let prefab0 = Prefab {
            prefab_id: 0,
            position: Vector3::new(0.0, 0.0, 0.0),
            rotation: UnitQuaternion::identity(),
            bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
            movement: Movement {
                baseline_velocity: Vector3::new(0., 0., -1.0),
                arcs_plane_normal: Unit::new_normalize(Vector3::new(1., 0., 0.)),
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
            spawn_count: 1,
            spawn_period: 1.0,
            trigger_time: 30.0,
            priority: 0,
            missed_spawns: 0,
            is_spawn_period_strict: false,
            last_spawn_attempt: 0.0,
        };
        let feature1 = Feature {
            trigger_time: 120.0,
            ..feature0.clone()
        };
        let feature2 = Feature {
            trigger_time: 130.0,
            ..feature0.clone()
        };
        let feature3 = Feature {
            trigger_time: 125.0,
            ..feature0.clone()
        };

        let mut upcoming_features: Vec<Feature> = vec![feature0.clone(), feature1.clone(), feature2.clone(), feature3.clone()];
        let mut active_features: Vec<Feature> = Vec::new();
        let mut distance_travelled = 0.0_f32;


        drain_upcoming_features(
            &mut upcoming_features,
            &mut active_features,
            &world,
            distance_travelled,
        );

        let expected: Vec<Feature> = Vec::new();
        dbg!(&active_features);
        dbg!(&expected);
        assert!(active_features.iter().eq(expected.iter()));

        distance_travelled = 11.0;

        drain_upcoming_features(
            &mut upcoming_features,
            &mut active_features,
            &world,
            distance_travelled,
        );

        let expected = [feature0.clone()];
        assert!(active_features.iter().eq(expected.iter()));

        distance_travelled = 98.0;

        drain_upcoming_features(
            &mut upcoming_features,
            &mut active_features,
            &world,
            distance_travelled,
        );

        let expected = [feature0.clone()];
        assert!(active_features.iter().eq(expected.iter()));

        distance_travelled = 105.0;

        drain_upcoming_features(
            &mut upcoming_features,
            &mut active_features,
            &world,
            distance_travelled,
        );

        let expected = [feature0.clone(), feature1.clone(), feature3.clone()];
        assert!(active_features.iter().eq(expected.iter()));

        distance_travelled = 117.0;

        drain_upcoming_features(
            &mut upcoming_features,
            &mut active_features,
            &world,
            distance_travelled,
        );

        let expected = [feature0.clone(), feature1.clone(), feature3.clone(), feature2.clone()];
        assert!(active_features.iter().eq(expected.iter()));
    }
}
