use crate::interop::EntitiesArrayDescription;

#[no_mangle]
pub unsafe extern fn bind_deallocate_vec(entity_array_description: EntitiesArrayDescription) {
    let len = entity_array_description.length as usize;
    drop(Vec::from_raw_parts(entity_array_description.pointer, len, len));
}
