use serde::{Serialize, Deserialize, Serializer, Deserializer};
use nalgebra::{Point3, Vector3};
use ncollide3d::bounding_volume::{AABB};
use serde::ser::{SerializeStruct, SerializeSeq};
use serde::de::{Visitor, SeqAccess, Unexpected, Error};
use std::fmt::Formatter;
use serde::__private::fmt;

pub fn serialize_aabb<S>(aabb: &AABB<f32>, serializer: S) -> Result<S::Ok, S::Error> where S: Serializer,
{
    let mut state = serializer.serialize_seq(Some(6))?;
    state.serialize_element(&aabb.mins.x)?;
    state.serialize_element(&aabb.mins.y)?;
    state.serialize_element(&aabb.mins.z)?;
    state.serialize_element(&aabb.maxs.x)?;
    state.serialize_element(&aabb.maxs.y)?;
    state.serialize_element(&aabb.maxs.z)?;
    state.end()
}

pub fn deserialize_aabb<'de, D>(deserializer: D) -> Result<AABB<f32>, D::Error> where D: Deserializer<'de> {
    struct AABBVisitor;

    impl<'de> Visitor<'de> for AABBVisitor {
        type Value = AABB<f32>;

        fn expecting(&self, formatter: &mut Formatter) -> fmt::Result {
            formatter.write_str("A sequence of 6 floats")
        }

        fn visit_seq<A>(self, mut seq: A) -> Result<Self::Value, A::Error> where A: SeqAccess<'de>, {
            let mut vec = Vec::new();

            while let Some(elem) = seq.next_element()? {
                vec.push(elem);
            }
            Ok(AABB::new(Point3::new(vec[0], vec[1], vec[2]), Point3::new(vec[3], vec[4], vec[5])))
        }
    }
    deserializer.deserialize_seq(AABBVisitor)
}