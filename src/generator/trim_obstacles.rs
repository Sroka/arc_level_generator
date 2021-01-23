use std::collections::VecDeque;
use super::types::{CollidableEntity};

/// Removes obstacles after they leave bounded volume of a visible world
/// * `obstacles` - entity list to trim
/// * `time_travelled` - current time passed in a generated world
///
pub fn trim_obstacles(
    obstacles: &mut VecDeque<CollidableEntity>,
    time_travelled: f32,
) {
    obstacles.retain(|entity| {
        if time_travelled < entity.spawn_time - entity.movement_start_parameter + entity.movement_end_parameter {
            return true;
        };
        false
    });
}
