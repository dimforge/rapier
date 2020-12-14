use crate::data::MaybeSerializableData;
#[cfg(feature = "dim3")]
use crate::geometry::contact_generator::PfmPfmContactManifoldGeneratorWorkspace;
use crate::geometry::contact_generator::{
    HeightFieldShapeContactGeneratorWorkspace, TriMeshShapeContactGeneratorWorkspace,
    WorkspaceSerializationTag,
};

// Note we have this newtype because it simplifies the serialization/deserialization code.
pub struct ContactGeneratorWorkspace(pub Box<dyn MaybeSerializableData>);

impl Clone for ContactGeneratorWorkspace {
    fn clone(&self) -> Self {
        ContactGeneratorWorkspace(self.0.clone_dyn())
    }
}

impl<T: MaybeSerializableData> From<T> for ContactGeneratorWorkspace {
    fn from(data: T) -> Self {
        Self(Box::new(data) as Box<dyn MaybeSerializableData>)
    }
}

#[cfg(feature = "serde-serialize")]
impl serde::Serialize for ContactGeneratorWorkspace {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        use crate::serde::ser::SerializeStruct;

        if let Some((tag, ser)) = self.0.as_serialize() {
            let mut state = serializer.serialize_struct("ContactGeneratorWorkspace", 2)?;
            state.serialize_field("tag", &tag)?;
            state.serialize_field("inner", ser)?;
            state.end()
        } else {
            Err(serde::ser::Error::custom(
                "Found a non-serializable contact generator workspace.",
            ))
        }
    }
}

#[cfg(feature = "serde-serialize")]
impl<'de> serde::Deserialize<'de> for ContactGeneratorWorkspace {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        struct Visitor {};
        impl<'de> serde::de::Visitor<'de> for Visitor {
            type Value = ContactGeneratorWorkspace;
            fn expecting(&self, formatter: &mut std::fmt::Formatter) -> std::fmt::Result {
                write!(formatter, "one shape type tag and the inner shape data")
            }

            fn visit_seq<A>(self, mut seq: A) -> Result<Self::Value, A::Error>
            where
                A: serde::de::SeqAccess<'de>,
            {
                use num::cast::FromPrimitive;

                let tag: u32 = seq
                    .next_element()?
                    .ok_or_else(|| serde::de::Error::invalid_length(0, &self))?;

                fn deser<'de, A, S: MaybeSerializableData + serde::Deserialize<'de>>(
                    seq: &mut A,
                ) -> Result<Box<dyn MaybeSerializableData>, A::Error>
                where
                    A: serde::de::SeqAccess<'de>,
                {
                    let workspace: S = seq.next_element()?.ok_or_else(|| {
                        serde::de::Error::custom("Failed to deserialize builtin workspace.")
                    })?;
                    Ok(Box::new(workspace) as Box<dyn MaybeSerializableData>)
                }

                let workspace = match WorkspaceSerializationTag::from_u32(tag) {
                    Some(WorkspaceSerializationTag::HeightfieldShapeContactGeneratorWorkspace) => {
                        deser::<A, HeightFieldShapeContactGeneratorWorkspace>(&mut seq)?
                    }
                    Some(WorkspaceSerializationTag::TriMeshShapeContactGeneratorWorkspace) => {
                        deser::<A, TriMeshShapeContactGeneratorWorkspace>(&mut seq)?
                    }
                    #[cfg(feature = "dim3")]
                    Some(WorkspaceSerializationTag::PfmPfmContactGeneratorWorkspace) => {
                        deser::<A, PfmPfmContactManifoldGeneratorWorkspace>(&mut seq)?
                    }
                    None => {
                        return Err(serde::de::Error::custom(
                            "found invalid contact generator workspace type to deserialize",
                        ))
                    }
                };

                Ok(ContactGeneratorWorkspace(workspace))
            }
        }

        deserializer.deserialize_struct("ContactGeneratorWorkspace", &["tag", "inner"], Visitor {})
    }
}
