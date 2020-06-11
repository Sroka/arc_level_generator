use std::collections::VecDeque;
use super::types::{CollideableEntity, VisibleWorld};
use ncollide3d::bounding_volume::BoundingVolume;
use nalgebra::{Isometry3, Translation3, UnitQuaternion};

/// Removes obstacles after they leave bounded volume of a visible world
/// * `obstacles` - entity list to trim
/// * `world` - visible world
/// * `time_travelled` - current time passed in a generated world
///
pub fn trim_obstacles(
    obstacles: &mut VecDeque<CollideableEntity>,
    world: &VisibleWorld,
    time_travelled: f32,
) {
    obstacles.retain(|entity| {
        if time_travelled < entity.spawn_time {
            return true;
        };
        let position = entity.position(time_travelled);
        let current_entity_aabb = entity.bounding_box
            .transform_by(&Isometry3::from_parts(
                Translation3::from(position),
                entity.rotation,
            )
            );
        // FIXME Eh, just a hack for prefabs being spawned outside world
        if current_entity_aabb.maxs().z > world.world_bounds.mins().z {
            return true;
        }
        false
        // world.world_bounds.intersects(&current_entity_aabb)
    });
}

#[cfg(test)]
mod tests {
    use crate::generator::types::{CollideableEntity, VisibleWorld};
    use super::trim_obstacles;

    use std::collections::VecDeque;
    use std::iter::FromIterator;

    use nalgebra::{Vector3, Point3, UnitQuaternion};
    use ncollide3d::bounding_volume::AABB;

    #[test]
    fn test_trim_obstacles() {
        let world = VisibleWorld {
            world_bounds: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(10., 10., 10.)),
            travel_speed: 4.0,
        };
        let obstacle0 = CollideableEntity {
            spawn_position: Vector3::new(0., 0., 0.),
            spawn_time: 10.0,
            velocity: Vector3::new(0., 0., -1.),
            rotation: UnitQuaternion::from_euler_angles(0., std::f32::consts::FRAC_PI_4, 0.),
            bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0.5, 0.5, 0.5)),
            prefab_id: 0,
            priority: 0,
        };
        let obstacle1 = CollideableEntity {
            priority: 5,
            spawn_time: 30.0,
            rotation: UnitQuaternion::from_euler_angles(0., std::f32::consts::FRAC_PI_2, 0.),
            ..obstacle0.clone()
        };
        let mut obstacles: VecDeque<CollideableEntity> = VecDeque::from_iter([obstacle0.clone(), obstacle1.clone()].iter().cloned());
        trim_obstacles(&mut obstacles, &world, 0.);
        assert!(obstacles.iter().eq([obstacle0.clone(), obstacle1.clone()].iter()));

        trim_obstacles(&mut obstacles, &world, 15.);
        assert!(obstacles.iter().eq([obstacle0.clone(), obstacle1.clone()].iter()));

        trim_obstacles(&mut obstacles, &world, 20.0);
        assert!(obstacles.iter().eq([obstacle0.clone(), obstacle1.clone()].iter()));

        trim_obstacles(&mut obstacles, &world, 20.706);
        assert!(obstacles.iter().eq([obstacle0.clone(), obstacle1.clone()].iter()));

        trim_obstacles(&mut obstacles, &world, 20.7072);
        assert!(obstacles.iter().eq([obstacle1.clone()].iter()));

        trim_obstacles(&mut obstacles, &world, 40.49);
        assert!(obstacles.iter().eq([obstacle1.clone()].iter()));

        trim_obstacles(&mut obstacles, &world, 40.501);
        let expected: Vec<CollideableEntity> = Vec::new();
        assert!(obstacles.iter().eq(expected.iter()));
    }
}
