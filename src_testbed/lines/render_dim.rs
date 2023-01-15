pub mod r3d {
    use bevy::{
        core_pipeline::core_3d::Opaque3d,
        pbr::{
            DrawMesh, MeshPipeline, MeshPipelineKey, MeshUniform, SetMeshBindGroup,
            SetMeshViewBindGroup,
        },
        prelude::*,
        render::{
            mesh::MeshVertexBufferLayout,
            render_asset::RenderAssets,
            render_phase::{DrawFunctions, RenderPhase, SetItemPipeline},
            render_resource::{
                BlendState, ColorTargetState, ColorWrites, CompareFunction, DepthBiasState,
                DepthStencilState, FragmentState, FrontFace, MultisampleState, PipelineCache,
                PolygonMode, PrimitiveState, PrimitiveTopology, RenderPipelineDescriptor,
                SpecializedMeshPipeline, SpecializedMeshPipelineError, SpecializedMeshPipelines,
                StencilFaceState, StencilState, TextureFormat, VertexAttribute, VertexBufferLayout,
                VertexFormat, VertexState, VertexStepMode,
            },
            texture::BevyDefault,
            view::{ExtractedView, Msaa},
        },
        utils::Hashed,
    };

    use crate::lines::{DebugLinesConfig, RenderDebugLinesMesh, DEBUG_LINES_SHADER_HANDLE};

    #[derive(Resource)]
    pub(crate) struct DebugLinePipeline {
        mesh_pipeline: MeshPipeline,
        shader: Handle<Shader>,
        //always_in_front: bool,
    }
    impl FromWorld for DebugLinePipeline {
        fn from_world(render_world: &mut World) -> Self {
            //let config = render_world.get_resource::<DebugLinesConfig>().unwrap();
            DebugLinePipeline {
                mesh_pipeline: render_world.get_resource::<MeshPipeline>().unwrap().clone(),
                shader: DEBUG_LINES_SHADER_HANDLE.typed(),
                //always_in_front: config.always_in_front,
            }
        }
    }

    impl SpecializedMeshPipeline for DebugLinePipeline {
        type Key = (bool, MeshPipelineKey);

        fn specialize(
            &self,
            (depth_test, key): Self::Key,
            layout: &MeshVertexBufferLayout,
        ) -> Result<RenderPipelineDescriptor, SpecializedMeshPipelineError> {
            //use VertexFormat::{Float32x3, Float32x4};

            let mut shader_defs = Vec::new();
            shader_defs.push("LINES_3D".to_string());
            if depth_test {
                shader_defs.push("DEPTH_TEST_ENABLED".to_string());
            }

            let vertex_buffer_layout = layout.get_layout(&[
                Mesh::ATTRIBUTE_POSITION.at_shader_location(0),
                Mesh::ATTRIBUTE_COLOR.at_shader_location(1),
            ])?;
            let (label, blend, depth_write_enabled);
            if key.contains(MeshPipelineKey::TRANSPARENT_MAIN_PASS) {
                label = "transparent_mesh_pipeline".into();
                blend = Some(BlendState::ALPHA_BLENDING);
                // For the transparent pass, fragments that are closer will be alpha
                // blended but their depth is not written to the depth buffer.
                depth_write_enabled = false;
            } else {
                label = "opaque_mesh_pipeline".into();
                blend = Some(BlendState::REPLACE);
                // For the opaque and alpha mask passes, fragments that are closer
                // will replace the current fragment value in the output and the depth is
                // written to the depth buffer.
                depth_write_enabled = true;
            }

            Ok(RenderPipelineDescriptor {
                vertex: VertexState {
                    shader: self.shader.clone_weak(),
                    entry_point: "vertex".into(),
                    shader_defs: shader_defs.clone(),
                    buffers: vec![vertex_buffer_layout],
                },
                fragment: Some(FragmentState {
                    shader: self.shader.clone_weak(),
                    shader_defs,
                    entry_point: "fragment".into(),
                    targets: vec![Some(ColorTargetState {
                        format: TextureFormat::bevy_default(),
                        blend,
                        write_mask: ColorWrites::ALL,
                    })],
                }),
                layout: Some(vec![self.mesh_pipeline.view_layout.clone()]),
                primitive: PrimitiveState {
                    front_face: FrontFace::Ccw,
                    cull_mode: None,
                    unclipped_depth: false,
                    polygon_mode: PolygonMode::Fill,
                    conservative: false,
                    topology: PrimitiveTopology::LineList,
                    strip_index_format: None,
                },
                depth_stencil: Some(DepthStencilState {
                    format: TextureFormat::Depth32Float,
                    depth_write_enabled,
                    depth_compare: CompareFunction::Greater,
                    stencil: StencilState {
                        front: StencilFaceState::IGNORE,
                        back: StencilFaceState::IGNORE,
                        read_mask: 0,
                        write_mask: 0,
                    },
                    bias: DepthBiasState {
                        constant: 0,
                        slope_scale: 0.0,
                        clamp: 0.0,
                    },
                }),
                multisample: MultisampleState {
                    count: key.msaa_samples(),
                    mask: !0,
                    alpha_to_coverage_enabled: false,
                },
                label: Some(label),
            })
        }
    }

    pub(crate) fn queue(
        opaque_3d_draw_functions: Res<DrawFunctions<Opaque3d>>,
        debug_line_pipeline: Res<DebugLinePipeline>,
        mut pipelines: ResMut<SpecializedMeshPipelines<DebugLinePipeline>>,
        mut pipeline_cache: ResMut<PipelineCache>,
        render_meshes: Res<RenderAssets<Mesh>>,
        msaa: Res<Msaa>,
        material_meshes: Query<(Entity, &MeshUniform, &Handle<Mesh>), With<RenderDebugLinesMesh>>,
        config: Res<DebugLinesConfig>,
        mut views: Query<(&ExtractedView, &mut RenderPhase<Opaque3d>)>,
    ) {
        let draw_custom = opaque_3d_draw_functions
            .read()
            .get_id::<DrawDebugLines>()
            .unwrap();
        let key = MeshPipelineKey::from_msaa_samples(msaa.samples);
        for (view, mut transparent_phase) in views.iter_mut() {
            let view_matrix = view.transform.compute_matrix();
            let view_row_2 = view_matrix.row(2);
            for (entity, mesh_uniform, mesh_handle) in material_meshes.iter() {
                if let Some(mesh) = render_meshes.get(mesh_handle) {
                    let pipeline = pipelines
                        .specialize(
                            &mut pipeline_cache,
                            &debug_line_pipeline,
                            (config.depth_test, key),
                            &mesh.layout,
                        )
                        .unwrap();
                    transparent_phase.add(Opaque3d {
                        entity,
                        pipeline,
                        draw_function: draw_custom,
                        distance: view_row_2.dot(mesh_uniform.transform.col(3)),
                    });
                }
            }
        }
    }

    pub(crate) type DrawDebugLines = (
        SetItemPipeline,
        SetMeshViewBindGroup<0>,
        SetMeshBindGroup<1>,
        DrawMesh,
    );
}

pub mod r2d {
    use bevy::{
        asset::Handle,
        core_pipeline::core_2d::Transparent2d,
        prelude::*,
        render::{
            mesh::MeshVertexBufferLayout,
            render_asset::RenderAssets,
            render_phase::{DrawFunctions, RenderPhase, SetItemPipeline},
            render_resource::{
                BlendState, ColorTargetState, ColorWrites, CompareFunction, DepthBiasState,
                DepthStencilState, FragmentState, FrontFace, MultisampleState, PipelineCache,
                PolygonMode, PrimitiveState, PrimitiveTopology, RenderPipelineDescriptor, Shader,
                SpecializedMeshPipeline, SpecializedMeshPipelineError, SpecializedMeshPipelines,
                StencilFaceState, StencilState, TextureFormat, VertexAttribute, VertexBufferLayout,
                VertexFormat, VertexState, VertexStepMode,
            },
            texture::BevyDefault,
            view::{Msaa, VisibleEntities},
        },
        sprite::{
            DrawMesh2d, Mesh2dHandle, Mesh2dPipeline, Mesh2dPipelineKey, Mesh2dUniform,
            SetMesh2dBindGroup, SetMesh2dViewBindGroup,
        },
        utils::FloatOrd,
    };

    use crate::lines::{RenderDebugLinesMesh, DEBUG_LINES_SHADER_HANDLE};

    #[derive(Resource)]
    pub(crate) struct DebugLinePipeline {
        mesh_pipeline: Mesh2dPipeline,
        shader: Handle<Shader>,
    }
    impl FromWorld for DebugLinePipeline {
        fn from_world(render_world: &mut World) -> Self {
            DebugLinePipeline {
                mesh_pipeline: render_world
                    .get_resource::<Mesh2dPipeline>()
                    .unwrap()
                    .clone(),
                shader: DEBUG_LINES_SHADER_HANDLE.typed(),
            }
        }
    }

    impl SpecializedMeshPipeline for DebugLinePipeline {
        type Key = Mesh2dPipelineKey;

        fn specialize(
            &self,
            key: Self::Key,
            layout: &MeshVertexBufferLayout,
        ) -> Result<RenderPipelineDescriptor, SpecializedMeshPipelineError> {
            /*
                        let mut shader_defs = Vec::new();
                        shader_defs.push("LINES_3D".to_string());
                        if depth_test {
                            shader_defs.push("DEPTH_TEST_ENABLED".to_string());
                        }
            */

            let vertex_buffer_layout = layout.get_layout(&[
                Mesh::ATTRIBUTE_POSITION.at_shader_location(0),
                Mesh::ATTRIBUTE_COLOR.at_shader_location(1),
            ])?;

            Ok(RenderPipelineDescriptor {
                vertex: VertexState {
                    shader: self.shader.clone_weak(),
                    entry_point: "vertex".into(),
                    shader_defs: vec![],
                    buffers: vec![vertex_buffer_layout],
                },
                fragment: Some(FragmentState {
                    shader: self.shader.clone_weak(),
                    shader_defs: vec![],
                    entry_point: "fragment".into(),
                    targets: vec![Some(ColorTargetState {
                        format: TextureFormat::bevy_default(),
                        blend: Some(BlendState::ALPHA_BLENDING),
                        write_mask: ColorWrites::ALL,
                    })],
                }),
                layout: Some(vec![self.mesh_pipeline.view_layout.clone()]),
                primitive: PrimitiveState {
                    front_face: FrontFace::Ccw,
                    cull_mode: None,
                    unclipped_depth: false,
                    polygon_mode: PolygonMode::Fill,
                    conservative: false,
                    topology: PrimitiveTopology::LineList,
                    strip_index_format: None,
                },
                depth_stencil: None,
                multisample: MultisampleState {
                    count: key.msaa_samples(),
                    mask: !0,
                    alpha_to_coverage_enabled: false,
                },
                label: None,
            })
        }
    }

    pub(crate) fn queue(
        draw2d_functions: Res<DrawFunctions<Transparent2d>>,
        debug_line_pipeline: Res<DebugLinePipeline>,
        mut pipeline_cache: ResMut<PipelineCache>,
        mut specialized_pipelines: ResMut<SpecializedMeshPipelines<DebugLinePipeline>>,
        render_meshes: Res<RenderAssets<Mesh>>,
        msaa: Res<Msaa>,
        material_meshes: Query<(&Mesh2dUniform, &Mesh2dHandle), With<RenderDebugLinesMesh>>,
        mut views: Query<(&VisibleEntities, &mut RenderPhase<Transparent2d>)>,
    ) {
        for (view, mut phase) in views.iter_mut() {
            let draw_mesh2d = draw2d_functions.read().get_id::<DrawDebugLines>().unwrap();
            let msaa_key = Mesh2dPipelineKey::from_msaa_samples(msaa.samples);

            for visible_entity in &view.entities {
                if let Ok((uniform, mesh_handle)) = material_meshes.get(*visible_entity) {
                    if let Some(mesh) = render_meshes.get(&mesh_handle.0) {
                        let mesh_key = msaa_key
                            | Mesh2dPipelineKey::from_primitive_topology(
                                PrimitiveTopology::LineList,
                            );
                        let mesh_z = uniform.transform.w_axis.z;
                        let pipeline = specialized_pipelines
                            .specialize(
                                &mut pipeline_cache,
                                &debug_line_pipeline,
                                mesh_key,
                                &mesh.layout,
                            )
                            .unwrap();
                        phase.add(Transparent2d {
                            entity: *visible_entity,
                            draw_function: draw_mesh2d,
                            pipeline,
                            sort_key: FloatOrd(mesh_z),
                            batch_range: None,
                        });
                    }
                }
            }
        }
    }

    pub(crate) type DrawDebugLines = (
        SetItemPipeline,
        SetMesh2dViewBindGroup<0>,
        SetMesh2dBindGroup<1>,
        DrawMesh2d,
    );
}
