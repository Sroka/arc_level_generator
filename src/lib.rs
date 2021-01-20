mod generator;
mod interop;
#[macro_use]
extern crate approx;

pub use generator::generate;
pub use generator::{VisibleWorld, Feature, Prefab, CollideableEntity, Movement};
pub use interop::bind_generate;
pub use interop::bind_deallocate_vec;
pub use interop::{VisibleWorldDescription, FeatureDescription, PrefabDescription, MovementDescription};
