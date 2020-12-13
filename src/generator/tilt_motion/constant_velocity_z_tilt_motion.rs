use nalgebra::{Unit, Isometry, Vector3, UnitQuaternion, U3, Vector2, Translation};
use ncollide3d::interpolation::RigidMotion;
use float_cmp::{ApproxEq, F32Margin};

#[derive(Debug)]
pub struct ConstantVelocityZTiltMotion {
    /// The time at which this parametrization begins. Can be negative.
    pub t0: f32,
    /// The starting isometry at `t = self.t0`.
    pub start: Isometry<f32, U3, UnitQuaternion<f32>>,

    /// Velocity
    pub linear_velocity: Vector3<f32>,
    /// Tilt
    pub tilt_xy_direction: Vector2<f32>,
    pub tilt_angle: f32,
    pub tilt_distance: f32,
    pub tilt_easing_range: f32,
    pub tilt_rotation_strength: f32,
}

impl ConstantVelocityZTiltMotion {
    pub fn new(t0: f32,
               start: Isometry<f32, U3, UnitQuaternion<f32>>,
               linear_velocity: Vector3<f32>,
               tilt_xy_direction: Vector2<f32>,
               tilt_angle: f32,
               tilt_distance: f32,
               tilt_easing_range: f32,
               tilt_rotation_coefficient: f32) -> Self {
        ConstantVelocityZTiltMotion { t0, start, linear_velocity, tilt_xy_direction, tilt_angle, tilt_distance, tilt_easing_range, tilt_rotation_strength: tilt_rotation_coefficient }
    }

    pub fn new_from_tilted_start(t0: f32,
                                 start: Isometry<f32, U3, UnitQuaternion<f32>>,
                                 linear_velocity: Vector3<f32>,
                                 tilt_xy_direction: Vector2<f32>,
                                 tilt_angle: f32,
                                 tilt_distance: f32,
                                 tilt_easing_range: f32,
                                 tilt_rotation_coefficient: f32) -> Self {
        let doubly_tilted_motion = ConstantVelocityZTiltMotion { t0, start, linear_velocity, tilt_xy_direction, tilt_angle, tilt_distance, tilt_easing_range, tilt_rotation_strength: tilt_rotation_coefficient };
        let doubly_tilted_isometry = doubly_tilted_motion.position_at_time(t0);
        let mut tilt_position = doubly_tilted_isometry.translation.vector;
        tilt_position.x -= start.translation.vector.x;
        tilt_position.y -= start.translation.vector.y;
        let mut tilt_rotation = doubly_tilted_isometry.rotation * start.rotation.inverse();

        let mut start_position_with_stripped_tilt = start.translation.vector.clone();
        start_position_with_stripped_tilt.x -= tilt_position.x;
        start_position_with_stripped_tilt.y -= tilt_position.y;
        let mut start_rotation_with_stripped_tilt = start.rotation.clone();
        start_rotation_with_stripped_tilt = tilt_rotation.inverse() * start_rotation_with_stripped_tilt;

        ConstantVelocityZTiltMotion {
            t0,
            start: Isometry::from_parts(
                Translation::from(start_position_with_stripped_tilt),
                start_rotation_with_stripped_tilt,
            ),
            linear_velocity: doubly_tilted_motion.linear_velocity,
            tilt_xy_direction: doubly_tilted_motion.tilt_xy_direction,
            tilt_angle,
            tilt_distance,
            tilt_easing_range,
            tilt_rotation_strength: tilt_rotation_coefficient,
        }
    }
}

impl RigidMotion<f32> for ConstantVelocityZTiltMotion {
    fn position_at_time(&self, t: f32) -> Isometry<f32, U3, UnitQuaternion<f32>> {
        let mut position_tilted = self.start.translation.vector + self.linear_velocity * (t - self.t0);
        let mut rotation_tilted = self.start.rotation;
        if !self.tilt_angle.approx_eq(0.0, F32Margin::zero()) {
            let absolute_z_position = position_tilted.z.abs();
            let tilt_angle_radians = self.tilt_angle.to_radians();
            let direction_normalized = self.tilt_xy_direction.normalize();
            let easing_circle_radius = self.tilt_easing_range / tilt_angle_radians.sin();

            let easing_period_movement = (absolute_z_position - self.tilt_distance).max(0.0).min(self.tilt_easing_range);
            let linear_tilt_period_movement = (absolute_z_position - self.tilt_distance - self.tilt_easing_range).max(0.0);

            let easing_tilt = easing_circle_radius - (easing_circle_radius.powf(2.0) - easing_period_movement.powf(2.0)).sqrt();
            let linear_tilt = linear_tilt_period_movement * tilt_angle_radians.tan();
            let tilt = direction_normalized * (easing_tilt + linear_tilt);
            position_tilted.x += tilt.x;
            position_tilted.y += tilt.y;
            let rotation_axis = Unit::new_normalize(Vector3::new(self.tilt_xy_direction.x, self.tilt_xy_direction.y, 0.).cross(&Vector3::z_axis()));
            rotation_tilted = UnitQuaternion::from_axis_angle(&rotation_axis, position_tilted.z.signum() * (easing_period_movement / self.tilt_easing_range) * tilt_angle_radians) * rotation_tilted;
        }

        return Isometry::from_parts(
            position_tilted.into(),
            rotation_tilted,
        );
    }
}

#[cfg(test)]
mod tests {
    use crate::generator::tilt_motion::ConstantVelocityZTiltMotion;
    use nalgebra::{Isometry, Translation, Vector3, UnitQuaternion, Vector2};
    use ncollide3d::interpolation::RigidMotion;

    #[test]
    fn test_position_at_time_rotated() {
        let motion = ConstantVelocityZTiltMotion::new(6.,
                                                      Isometry::from_parts(Translation::from(Vector3::new(0., 0., 0.)), UnitQuaternion::identity()),
                                                      Vector3::new(0., 0., -5.),
                                                      Vector2::new(0., 1.),
                                                      45.,
                                                      10.,
                                                      10.,
                                                      1.,
        );
        assert_eq!(motion.position_at_time(0.).translation.vector, Vector3::new(0., 14.142136, 30.));
        assert_eq!(motion.position_at_time(0.).rotation, UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f32::consts::FRAC_PI_4));
        assert_eq!(motion.position_at_time(2.).translation.vector, Vector3::new(0., 4.1421356, 20.));
        assert_eq!(motion.position_at_time(2.).rotation, UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f32::consts::FRAC_PI_4));
        assert_eq!(motion.position_at_time(3.).translation.vector, Vector3::new(0., 0.9133787, 15.));
        assert_eq!(motion.position_at_time(3.).rotation, UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f32::consts::FRAC_PI_8));
        assert_eq!(motion.position_at_time(4.).translation.vector, Vector3::new(0., 0., 10.));
        assert_eq!(motion.position_at_time(4.).rotation, UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.));
        assert_eq!(motion.position_at_time(6.).translation.vector, Vector3::new(0., 0., 0.));
        assert_eq!(motion.position_at_time(6.).rotation, UnitQuaternion::identity());
        assert_eq!(motion.position_at_time(8.).translation.vector, Vector3::new(0., 0., -10.));
        assert_eq!(motion.position_at_time(8.).rotation, UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.));
        assert_eq!(motion.position_at_time(9.).translation.vector, Vector3::new(0., 0.9133787, -15.));
        assert_eq!(motion.position_at_time(9.).rotation, UnitQuaternion::from_axis_angle(&Vector3::x_axis(), -std::f32::consts::FRAC_PI_8));
        assert_eq!(motion.position_at_time(10.).translation.vector, Vector3::new(0., 4.1421356, -20.));
        assert_eq!(motion.position_at_time(10.).rotation, UnitQuaternion::from_axis_angle(&Vector3::x_axis(), -std::f32::consts::FRAC_PI_4));
        assert_eq!(motion.position_at_time(12.).translation.vector, Vector3::new(0., 14.142136, -30.));
        assert_eq!(motion.position_at_time(12.).rotation, UnitQuaternion::from_axis_angle(&Vector3::x_axis(), -std::f32::consts::FRAC_PI_4));
    }

    #[test]
    fn test_position_at_time_rotated_with_initial_offset() {
        let motion = ConstantVelocityZTiltMotion::new(6.,
                                                      Isometry::from_parts(Translation::from(Vector3::new(0., 10., 0.)), UnitQuaternion::identity()),
                                                      Vector3::new(0., 0., -5.),
                                                      Vector2::new(0., 1.),
                                                      45.,
                                                      10.,
                                                      10.,
                                                      1.,
        );
        assert_eq!(motion.position_at_time(0.).translation.vector, Vector3::new(0., 24.142136, 30.));
        assert_eq!(motion.position_at_time(0.).rotation, UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f32::consts::FRAC_PI_4));
        assert_eq!(motion.position_at_time(2.).translation.vector, Vector3::new(0., 14.1421356, 20.));
        assert_eq!(motion.position_at_time(2.).rotation, UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f32::consts::FRAC_PI_4));
        assert_eq!(motion.position_at_time(3.).translation.vector, Vector3::new(0., 10.9133787, 15.));
        assert_eq!(motion.position_at_time(3.).rotation, UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f32::consts::FRAC_PI_8));
        assert_eq!(motion.position_at_time(4.).translation.vector, Vector3::new(0., 10., 10.));
        assert_eq!(motion.position_at_time(4.).rotation, UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.));
        assert_eq!(motion.position_at_time(6.).translation.vector, Vector3::new(0., 10., 0.));
        assert_eq!(motion.position_at_time(6.).rotation, UnitQuaternion::identity());
        assert_eq!(motion.position_at_time(8.).translation.vector, Vector3::new(0., 10., -10.));
        assert_eq!(motion.position_at_time(8.).rotation, UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.));
        assert_eq!(motion.position_at_time(9.).translation.vector, Vector3::new(0., 10.9133787, -15.));
        assert_eq!(motion.position_at_time(9.).rotation, UnitQuaternion::from_axis_angle(&Vector3::x_axis(), -std::f32::consts::FRAC_PI_8));
        assert_eq!(motion.position_at_time(10.).translation.vector, Vector3::new(0., 14.1421356, -20.));
        assert_eq!(motion.position_at_time(10.).rotation, UnitQuaternion::from_axis_angle(&Vector3::x_axis(), -std::f32::consts::FRAC_PI_4));
        assert_eq!(motion.position_at_time(12.).translation.vector, Vector3::new(0., 24.142136, -30.));
        assert_eq!(motion.position_at_time(12.).rotation, UnitQuaternion::from_axis_angle(&Vector3::x_axis(), -std::f32::consts::FRAC_PI_4));
    }

    #[test]
    fn test_new_from_tilted_start() {
        let motion = ConstantVelocityZTiltMotion::new_from_tilted_start(0.,
                                                                        Isometry::from_parts(Translation::from(Vector3::new(0., 14.142136, 30.)), UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f32::consts::FRAC_PI_4)),
                                                                        Vector3::new(0., 0., -5.),
                                                                        Vector2::new(0., 1.),
                                                                        45.,
                                                                        10.,
                                                                        10.,
                                                                        1.,
        );
        assert_eq!(motion.position_at_time(0.).translation.vector, Vector3::new(0., 14.142136, 30.));
        assert_eq!(motion.position_at_time(0.).rotation, UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f32::consts::FRAC_PI_4));
        assert_eq!(motion.position_at_time(2.).translation.vector, Vector3::new(0., 4.1421356, 20.));
        assert_eq!(motion.position_at_time(2.).rotation, UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f32::consts::FRAC_PI_4));
        assert_eq!(motion.position_at_time(3.).translation.vector, Vector3::new(0., 0.9133787, 15.));
        assert_eq!(motion.position_at_time(3.).rotation, UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f32::consts::FRAC_PI_8));
        assert_eq!(motion.position_at_time(4.).translation.vector, Vector3::new(0., 0., 10.));
        assert_eq!(motion.position_at_time(4.).rotation, UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.));
        assert_eq!(motion.position_at_time(6.).translation.vector, Vector3::new(0., 0., 0.));
        assert_eq!(motion.position_at_time(6.).rotation, UnitQuaternion::identity());
        assert_eq!(motion.position_at_time(8.).translation.vector, Vector3::new(0., 0., -10.));
        assert_eq!(motion.position_at_time(8.).rotation, UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.));
        assert_eq!(motion.position_at_time(9.).translation.vector, Vector3::new(0., 0.9133787, -15.));
        assert_eq!(motion.position_at_time(9.).rotation, UnitQuaternion::from_axis_angle(&Vector3::x_axis(), -std::f32::consts::FRAC_PI_8));
        assert_eq!(motion.position_at_time(10.).translation.vector, Vector3::new(0., 4.1421356, -20.));
        assert_eq!(motion.position_at_time(10.).rotation, UnitQuaternion::from_axis_angle(&Vector3::x_axis(), -std::f32::consts::FRAC_PI_4));
        assert_eq!(motion.position_at_time(12.).translation.vector, Vector3::new(0., 14.142136, -30.));
        assert_eq!(motion.position_at_time(12.).rotation, UnitQuaternion::from_axis_angle(&Vector3::x_axis(), -std::f32::consts::FRAC_PI_4));
    }
}
