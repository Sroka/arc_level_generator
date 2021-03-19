use ncollide3d::bounding_volume::AABB;
use serde::{Serialize, Deserialize};
use crate::generator::types::{serialize_aabb, deserialize_aabb};

#[derive(Debug, Serialize, Deserialize)]
pub struct VisibleWorld {
    #[serde(serialize_with = "serialize_aabb", deserialize_with = "deserialize_aabb")]
    pub world_bounds: AABB<f32>,
}


