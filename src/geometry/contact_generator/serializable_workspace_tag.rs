use num_derive::FromPrimitive;

#[derive(Copy, Clone, Debug, FromPrimitive)]
pub(super) enum WorkspaceSerializationTag {
    TriMeshShapeContactGeneratorWorkspace = 0,
    #[cfg(feature = "dim3")]
    PfmPfmContactGeneratorWorkspace,
    HeightfieldShapeContactGeneratorWorkspace,
}
