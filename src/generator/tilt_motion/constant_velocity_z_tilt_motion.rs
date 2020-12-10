use nalgebra::{Isometry, Vector3, UnitQuaternion, U3, Vector2};
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
}

impl ConstantVelocityZTiltMotion {
    pub fn new(t0: f32,
               start: Isometry<f32, U3, UnitQuaternion<f32>>,
               linear_velocity: Vector3<f32>,
               tilt_xy_direction: Vector2<f32>,
               tilt_angle: f32,
               tilt_distance: f32,
               tilt_easing_range: f32) -> Self {
        ConstantVelocityZTiltMotion { t0, start, linear_velocity, tilt_xy_direction, tilt_angle, tilt_distance, tilt_easing_range }
    }
}

impl RigidMotion<f32> for ConstantVelocityZTiltMotion {
    fn position_at_time(&self, t: f32) -> Isometry<f32, U3, UnitQuaternion<f32>> {
        let position: Vector3<f32> = self.start.translation.vector + self.linear_velocity * (t - self.t0);
        let mut position_tilted = position.clone();
        if !self.tilt_angle.approx_eq(0.0, F32Margin::zero()) {
            let absolute_z_position = position.z.abs();
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
        }

        return Isometry::from_parts(
            position_tilted.into(),
            self.start.rotation,
        );
    }
}
