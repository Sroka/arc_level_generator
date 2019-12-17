use nalgebra::Vector3;
use crate::interop::types::{VisibleWorldDescription, FeatureDescription, PrefabDescription, EntitiesArrayDescription, EntityDescription};

#[no_mangle]
pub unsafe extern fn generate_entities(
    features_ptr: *const FeatureDescription,
    features_count: i32,
    prefabs_ptr: *const PrefabDescription,
    prefabs_count: i32,
    world: VisibleWorldDescription,
) -> EntitiesArrayDescription {
    let array = [EntityDescription {}];
    EntitiesArrayDescription {
        pointer: array.as_ptr(),
        length: array.len() as i32,
    }
}