use crate::generator::types::Prefab;
use ncollide3d::bounding_volume::{AABB, BoundingVolume, local_point_cloud_aabb};
use nalgebra::{Point3, Vector3, Isometry3, Point};

/// Calculated total bounding volume that encloses the given array of prefabs
/// * `prefabs` - prefabs for which the bounding volume will be calculated
///
pub fn calculate_prefabs_spawn_bounds(prefabs: &[Prefab]) -> AABB<f32> {
    if prefabs.is_empty() {
        AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0., 0., 0.))
    } else {
        let head = &prefabs[0];
        let tails = &prefabs[1..];
        tails.iter()
            .fold(
                head.bounding_box.transform_by(&Isometry3::new(head.position, nalgebra::zero())),
                |aabb, prefab| aabb.merged(&prefab.bounding_box.transform_by(&Isometry3::new(prefab.position, nalgebra::zero()))),
            )
    }
}

pub fn calculate_prefabs_centers_bounds(prefabs: &[Prefab]) -> AABB<f32> {
    if prefabs.is_empty() {
        AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(0., 0., 0.))
    } else {
        let pts: Vec<Point3<f32>> = prefabs.iter()
            .map(|prefab| Point::from(prefab.position))
            .collect();
        local_point_cloud_aabb(&pts)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Vector3, UnitQuaternion};

    #[test]
    fn test_calculate_prefabs_spawn_bounds() {
        let prefab0 = Prefab {
            prefab_id: 0,
            position: Vector3::new(-10., 0., -10.),
            rotation: UnitQuaternion::identity(),
            bounding_box: AABB::from_half_extents(Point3::new(0., 0., 0.), Vector3::new(1., 1., 1.)),
            velocity: Vector3::new(1., 1., 1.),
        };
        let prefab1 = Prefab {
            position: Vector3::new(10., 0., 10.),
            ..prefab0.clone()
        };
        let prefab2 = Prefab {
            position: Vector3::new(20., 0., 10.),
            ..prefab1.clone()
        };
        let aabb = calculate_prefabs_spawn_bounds(&[prefab0.clone(), prefab1.clone(), prefab2.clone()]);
        assert_eq!(aabb, AABB::from_half_extents(Point3::new(5., 0., 0.), Vector3::new(16., 1., 11.)))
    }
}
