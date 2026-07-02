use rapier_testbed3d::TestbedViewer;

pub async fn run(viewer: &mut TestbedViewer) -> anyhow::Result<()> {
    crate::dynamic_trimesh3::run_impl(viewer, true).await
}
