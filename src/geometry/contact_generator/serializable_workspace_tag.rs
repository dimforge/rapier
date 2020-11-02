use num_derive::FromPrimitive;

#[derive(Copy, Clone, Debug, FromPrimitive)]
pub(super) enum WorkspaceSerializationTag {
    TrimeshShapeContactGeneratorWorkspace = 0,
    #[cfg(feature = "dim3")]
    PfmPfmContactGeneratorWorkspace,
    HeightfieldShapeContactGeneratorWorkspace,
}
