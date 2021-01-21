use nalgebra::{Vector3, Isometry, UnitQuaternion, U3, Unit};
use ncollide3d::interpolation::RigidMotion;
use std::ops::{Add, Sub};

#[derive(Debug)]
pub struct BiArcCurveMotion {
    /// The time at which this parametrization begins. Can be negative.
    pub t0: f32,
    /// The starting isometry at `t = self.t0`.
    pub start: Isometry<f32, U3, UnitQuaternion<f32>>,

    pub baseline_velocity: Vector3<f32>,
    pub arcs_plane_normal: Unit<Vector3<f32>>,

    pub approach_arc_angle: f32,
    pub approach_arc_center_distance: f32,
    pub approach_arc_radius: f32,
    pub approach_rotation_strength: f32,

    pub departure_arc_angle: f32,
    pub departure_arc_center_distance: f32,
    pub departure_arc_radius: f32,
    pub departure_rotation_strength: f32,
}

impl BiArcCurveMotion {
    pub fn new(t0: f32, start: Isometry<f32, U3, UnitQuaternion<f32>>, linear_velocity: Vector3<f32>, arcs_plane_normal: Unit<Vector3<f32>>, approach_arc_angle: f32, approach_arc_center_distance: f32, approach_arc_radius: f32, approach_rotation_strength: f32, departure_arc_angle: f32, departure_arc_center_distance: f32, departure_arc_radius: f32, departure_rotation_strength: f32) -> Self {
        BiArcCurveMotion { t0, start, baseline_velocity: linear_velocity, arcs_plane_normal, approach_arc_angle, approach_arc_center_distance, approach_arc_radius, approach_rotation_strength, departure_arc_angle, departure_arc_center_distance, departure_arc_radius, departure_rotation_strength }
    }
}

impl RigidMotion<f32> for BiArcCurveMotion {
    fn position_at_time(&self, t: f32) -> Isometry<f32, U3, UnitQuaternion<f32>> {
        let arc_direction = self.arcs_plane_normal.cross(&self.baseline_velocity);
        let baseline_position = t * &self.baseline_velocity;
        let baseline_distance = t.signum() * baseline_position.magnitude();
        let approach_easing_range = calculate_easing_range(self.approach_arc_radius, self.approach_arc_angle);
        let departure_easing_range = calculate_easing_range(self.departure_arc_radius, self.departure_arc_angle);
        let approach_easing_range_position = baseline_distance
            .max(-self.approach_arc_center_distance - approach_easing_range)
            .min(-self.approach_arc_center_distance)
            .add(self.approach_arc_center_distance)
            .abs();
        let approach_linear_range_position = baseline_distance
            .min(-self.approach_arc_center_distance - approach_easing_range)
            .add(self.approach_arc_center_distance + approach_easing_range)
            .abs();
        let departure_easing_range_position = baseline_distance
            .max(self.departure_arc_center_distance)
            .min(self.departure_arc_center_distance + departure_easing_range)
            .sub(self.departure_arc_center_distance)
            .abs();
        let departure_linear_range_position = baseline_distance
            .max(self.departure_arc_center_distance + departure_easing_range)
            .sub(self.departure_arc_center_distance + departure_easing_range)
            .abs();


        let approach_easing_range_shift = calculate_easing_range_shift(approach_easing_range_position, self.approach_arc_radius, self.approach_arc_angle);
        let approach_linear_range_shift = calculate_linear_range_shift(approach_linear_range_position, self.approach_arc_angle);
        let departure_easing_range_shift = calculate_easing_range_shift(departure_easing_range_position, self.departure_arc_radius, self.departure_arc_angle);
        let departure_linear_range_shift = calculate_linear_range_shift(departure_linear_range_position, self.departure_arc_angle);
        let shift = approach_easing_range_shift + approach_linear_range_shift + departure_easing_range_shift + departure_linear_range_shift;

        Isometry::from_parts(
            (&self.start.translation.vector + baseline_position + arc_direction * shift).into(),
            self.start.rotation,
        )
    }
}

impl BiArcCurveMotion {
    pub fn rotation_at_time(&self, t: f32) -> UnitQuaternion<f32> {
        let baseline_position = t * &self.baseline_velocity;
        let baseline_distance = t.signum() * baseline_position.magnitude();
        let approach_easing_range = calculate_easing_range(self.approach_arc_radius, self.approach_arc_angle);
        let departure_easing_range = calculate_easing_range(self.departure_arc_radius, self.departure_arc_angle);
        let approach_easing_range_position = baseline_distance
            .max(-self.approach_arc_center_distance - approach_easing_range)
            .min(-self.approach_arc_center_distance)
            .add(self.approach_arc_center_distance)
            .abs();
        let departure_easing_range_position = baseline_distance
            .max(self.departure_arc_center_distance)
            .min(self.departure_arc_center_distance + departure_easing_range)
            .sub(self.departure_arc_center_distance)
            .abs();

        UnitQuaternion::from_axis_angle(
            &self.arcs_plane_normal,
            -self.approach_rotation_strength *
                self.approach_arc_angle *
                approach_easing_range_position / approach_easing_range,
        ) *
            UnitQuaternion::from_axis_angle(
                &self.arcs_plane_normal,
                self.departure_rotation_strength *
                    self.departure_arc_angle *
                    departure_easing_range_position / departure_easing_range,
            )
            *
            self.start.rotation
    }
}

fn calculate_easing_range(radius: f32, angle: f32) -> f32 {
    angle.abs().sin() * radius
}

fn calculate_easing_range_shift(easing_range_position: f32, radius: f32, angle: f32) -> f32 {
    if radius < f32::EPSILON {
        return 0.;
    }
    return angle.signum() * radius * (1. - (easing_range_position / radius).asin().cos());
}

fn calculate_linear_range_shift(linear_range_position: f32, angle: f32) -> f32 {
    return angle.tan() * linear_range_position;
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Translation};

    #[test]
    fn test_position_at_time() {
        let motion = BiArcCurveMotion::new(
            0.,
            Isometry::from_parts(Translation::from(Vector3::new(0., 0., 0.)), UnitQuaternion::identity()),
            Vector3::new(0., 0., -1.),
            Unit::new_normalize(Vector3::new(1., 0., 0.)),
            45.0_f32.to_radians() as f32,
            10.,
            10.,
            1.,
            45.0_f32.to_radians() as f32,
            10.,
            10.,
            1.,
        );

        assert_eq!(motion.position_at_time(-27.07106781).translation.vector, Vector3::new(0., 12.9289322, 27.07106781));
        assert_eq!(motion.position_at_time(-17.07106781).translation.vector, Vector3::new(0., 2.9289322, 17.07106781));
        assert_eq!(motion.position_at_time(-10.).translation.vector, Vector3::new(0., 0., 10.));
        assert_eq!(motion.position_at_time(0.).translation.vector, Vector3::new(0., 0., 0.));
        assert_eq!(motion.position_at_time(10.).translation.vector, Vector3::new(0., 0., -10.));
        assert_eq!(motion.position_at_time(17.07106781).translation.vector, Vector3::new(0., 2.9289322, -17.07106781));
        assert_eq!(motion.position_at_time(27.07106781).translation.vector, Vector3::new(0., 12.9289322, -27.07106781));
    }


    #[test]
    fn test_position_at_time_with_negative_angles() {
        let motion = BiArcCurveMotion::new(
            0.,
            Isometry::from_parts(Translation::from(Vector3::new(0., 0., 0.)), UnitQuaternion::identity()),
            Vector3::new(0., 0., -1.),
            Unit::new_normalize(Vector3::new(1., 0., 0.)),
            -45.0_f32.to_radians() as f32,
            10.,
            10.,
            1.,
            -45.0_f32.to_radians() as f32,
            10.,
            10.,
            1.,
        );

        assert_eq!(motion.position_at_time(-27.07106781).translation.vector, Vector3::new(0., -12.9289322, 27.07106781));
        assert_eq!(motion.position_at_time(-17.07106781).translation.vector, Vector3::new(0., -2.9289322, 17.07106781));
        assert_eq!(motion.position_at_time(-10.).translation.vector, Vector3::new(0., 0., 10.));
        assert_eq!(motion.position_at_time(0.).translation.vector, Vector3::new(0., 0., 0.));
        assert_eq!(motion.position_at_time(10.).translation.vector, Vector3::new(0., 0., -10.));
        assert_eq!(motion.position_at_time(17.07106781).translation.vector, Vector3::new(0., -2.9289322, -17.07106781));
        assert_eq!(motion.position_at_time(27.07106781).translation.vector, Vector3::new(0., -12.9289322, -27.07106781));
    }

    #[test]
    fn test_rotation_at_time() {
        let motion = BiArcCurveMotion::new(
            0.,
            Isometry::from_parts(
                Translation::from(Vector3::new(0., 0., 0.)),
                UnitQuaternion::from_axis_angle(&Unit::new_normalize(Vector3::new(1., 0., 0.)), 45.0_f32.to_radians()),
            ),
            Vector3::new(0., 0., -1.),
            Unit::new_normalize(Vector3::new(1., 0., 0.)),
            45.0_f32.to_radians() as f32,
            10.,
            10.,
            1.,
            45.0_f32.to_radians() as f32,
            10.,
            10.,
            1.,
        );

        assert_relative_eq!(motion.rotation_at_time(-27.07106781), UnitQuaternion::from_axis_angle(&Unit::new_normalize(Vector3::new(1., 0., 0.)), 0.0_f32.to_radians()));
        assert_relative_eq!(motion.rotation_at_time(-17.07106781), UnitQuaternion::from_axis_angle(&Unit::new_normalize(Vector3::new(1., 0., 0.)), 0.0_f32.to_radians()));
        assert_relative_eq!(motion.rotation_at_time(-10.), UnitQuaternion::from_axis_angle(&Unit::new_normalize(Vector3::new(1., 0., 0.)), 45.0_f32.to_radians()));
        assert_relative_eq!(motion.rotation_at_time(0.), UnitQuaternion::from_axis_angle(&Unit::new_normalize(Vector3::new(1., 0., 0.)), 45.0_f32.to_radians()));
        assert_relative_eq!(motion.rotation_at_time(10.), UnitQuaternion::from_axis_angle(&Unit::new_normalize(Vector3::new(1., 0., 0.)), 45.0_f32.to_radians()));
        assert_relative_eq!(motion.rotation_at_time(17.07106781), UnitQuaternion::from_axis_angle(&Unit::new_normalize(Vector3::new(1., 0., 0.)), 90.0_f32.to_radians()));
        assert_relative_eq!(motion.rotation_at_time(27.07106781), UnitQuaternion::from_axis_angle(&Unit::new_normalize(Vector3::new(1., 0., 0.)), 90.0_f32.to_radians()));
    }


    #[test]
    fn test_rotation_at_time_with_negative_angles() {
        let motion = BiArcCurveMotion::new(
            0.,
            Isometry::from_parts(
                Translation::from(Vector3::new(0., 0., 0.)),
                UnitQuaternion::from_axis_angle(&Unit::new_normalize(Vector3::new(1., 0., 0.)), 45.0_f32.to_radians()),
            ),
            Vector3::new(0., 0., -1.),
            Unit::new_normalize(Vector3::new(1., 0., 0.)),
            45.0_f32.to_radians() as f32,
            10.,
            10.,
            1.,
            45.0_f32.to_radians() as f32,
            10.,
            10.,
            1.,
        );

        assert_relative_eq!(motion.rotation_at_time(-27.07106781), UnitQuaternion::from_axis_angle(&Unit::new_normalize(Vector3::new(1., 0., 0.)), 0.0_f32.to_radians()));
        assert_relative_eq!(motion.rotation_at_time(-17.07106781), UnitQuaternion::from_axis_angle(&Unit::new_normalize(Vector3::new(1., 0., 0.)), 0.0_f32.to_radians()));
        assert_relative_eq!(motion.rotation_at_time(-10.), UnitQuaternion::from_axis_angle(&Unit::new_normalize(Vector3::new(1., 0., 0.)), 45.0_f32.to_radians()));
        assert_relative_eq!(motion.rotation_at_time(0.), UnitQuaternion::from_axis_angle(&Unit::new_normalize(Vector3::new(1., 0., 0.)), 45.0_f32.to_radians()));
        assert_relative_eq!(motion.rotation_at_time(10.), UnitQuaternion::from_axis_angle(&Unit::new_normalize(Vector3::new(1., 0., 0.)), 45.0_f32.to_radians()));
        assert_relative_eq!(motion.rotation_at_time(17.07106781), UnitQuaternion::from_axis_angle(&Unit::new_normalize(Vector3::new(1., 0., 0.)), 90.0_f32.to_radians()));
        assert_relative_eq!(motion.rotation_at_time(27.07106781), UnitQuaternion::from_axis_angle(&Unit::new_normalize(Vector3::new(1., 0., 0.)), 90.0_f32.to_radians()));
    }


    #[test]
    fn test_rotation_at_time_orthogonal() {
        // TODO
        assert!(false);
        let motion = BiArcCurveMotion::new(
            0.,
            Isometry::from_parts(
                Translation::from(Vector3::new(0., 0., 0.)),
                UnitQuaternion::from_axis_angle(&Unit::new_normalize(Vector3::new(0., 1., 0.)), 45.0_f32.to_radians()),
            ),
            Vector3::new(0., 0., -1.),
            Unit::new_normalize(Vector3::new(1., 0., 0.)),
            45.0_f32.to_radians() as f32,
            10.,
            10.,
            1.,
            45.0_f32.to_radians() as f32,
            10.,
            10.,
            1.,
        );

        // assert_relative_eq!(motion.rotation_at_time(-27.07106781), UnitQuaternion::from_axis_angle(&Unit::new_normalize(Vector3::new(1., 0., 0.)), 0.0_f32.to_radians()));
        // assert_relative_eq!(motion.rotation_at_time(-17.07106781), UnitQuaternion::from_axis_angle(&Unit::new_normalize(Vector3::new(1., 0., 0.)), 0.0_f32.to_radians()));
        // assert_relative_eq!(motion.rotation_at_time(-10.), UnitQuaternion::from_axis_angle(&Unit::new_normalize(Vector3::new(1., 0., 0.)), 45.0_f32.to_radians()));
        // assert_relative_eq!(motion.rotation_at_time(0.), UnitQuaternion::from_axis_angle(&Unit::new_normalize(Vector3::new(1., 0., 0.)), 45.0_f32.to_radians()));
        // assert_relative_eq!(motion.rotation_at_time(10.), UnitQuaternion::from_axis_angle(&Unit::new_normalize(Vector3::new(1., 0., 0.)), 45.0_f32.to_radians()));
        // assert_relative_eq!(motion.rotation_at_time(17.07106781), UnitQuaternion::from_axis_angle(&Unit::new_normalize(Vector3::new(1., 0., 0.)), 90.0_f32.to_radians()));
        // assert_relative_eq!(motion.rotation_at_time(27.07106781), UnitQuaternion::from_axis_angle(&Unit::new_normalize(Vector3::new(1., 0., 0.)), 90.0_f32.to_radians()));
    }
}
