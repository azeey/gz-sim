use std::collections::HashMap;
use std::f32::consts::FRAC_PI_2;
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};

use bevy::asset::RenderAssetUsages;
use bevy::gltf::GltfAssetLabel;
use bevy::image::{ImageAddressMode, ImageFilterMode, ImageSampler, ImageSamplerDescriptor};
use bevy::light::{DirectionalLight, PointLight, RectLight, SpotLight};
use bevy::mesh::{Indices, PrimitiveTopology};
use bevy::prelude::*;
use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat};
use crossbeam_channel::Receiver;

use crate::camera_control::OrbitCameraState;
use crate::protocol::{
    DespawnEntityWire, LightUpsertWire, MSG_CLEAR_WORLD, MSG_DESPAWN_ENTITY, MSG_LIGHT_UPSERT,
    MSG_MESH_DATA, MSG_SNAPSHOT_REQUEST, MSG_SPAWN_VISUAL, MSG_TEXTURE_DATA, MSG_TRANSFORM_BATCH,
    MSG_WORLD_CONFIG, MeshDataWire, PbrMaterialWire, PoseWire, QUALITY_SHADOWS, QUALITY_SSAO,
    SnapshotRequestWire, SpawnVisualWire, TextureDataWire, TransformBatchHeaderWire,
    TransformItemWire, WorldConfigWire, encode_packet, parse_header, read_f32_vec, read_pod,
    read_u32_vec,
};
use crate::quality::QualityState;
use crate::swapchain::ExternalSwapchain;
use crate::zenoh_ffi::{ReceivedSample, ZenohPublisher, ZenohSubscriber};

#[derive(Component)]
pub struct GazeboWorldRoot;

#[derive(Component)]
pub struct SdfLightMeta {
    pub cast_shadows: bool,
}

#[derive(Resource)]
pub struct SceneSyncState {
    pub _scene_sub: Option<ZenohSubscriber>,
    pub scene_rx: Option<Receiver<ReceivedSample>>,
    pub snapshot_pub: Option<ZenohPublisher>,
    pub client_id: u64,
    pub received_snapshot: bool,
    pub last_snapshot_req: Instant,
    pub world_root: Option<Entity>,
    pub shm_sample_count: u64,
    pub raw_sample_count: u64,

    /// Gazebo entity_id -> Bevy parent entity (receives TransformBatch updates)
    pub parent_entities: HashMap<u64, Entity>,
    /// visual_id -> Bevy visual entity
    pub visual_entities: HashMap<u64, Entity>,
    /// Gazebo light entity_id -> Bevy light entity
    pub light_entities: HashMap<u64, Entity>,
    /// mesh_id -> Bevy Mesh handle
    pub streamed_meshes: HashMap<u64, Handle<Mesh>>,
    /// texture_id -> Bevy Image handle
    pub streamed_textures: HashMap<u64, Handle<Image>>,
}

impl SceneSyncState {
    pub fn new(
        scene_sub: Option<ZenohSubscriber>,
        scene_rx: Option<Receiver<ReceivedSample>>,
        snapshot_pub: Option<ZenohPublisher>,
    ) -> Self {
        let client_id = std::process::id() as u64;
        Self {
            _scene_sub: scene_sub,
            scene_rx,
            snapshot_pub,
            client_id,
            received_snapshot: false,
            last_snapshot_req: Instant::now() - Duration::from_secs(5),
            world_root: None,
            shm_sample_count: 0,
            raw_sample_count: 0,
            parent_entities: HashMap::new(),
            visual_entities: HashMap::new(),
            light_entities: HashMap::new(),
            streamed_meshes: HashMap::new(),
            streamed_textures: HashMap::new(),
        }
    }
}

pub fn spawn_world_root_system(mut commands: Commands, mut sync: ResMut<SceneSyncState>) {
    // Rotate Gazebo right-handed Z-up -> Bevy right-handed Y-up by Rx(-90 deg)
    let root = commands
        .spawn((
            GazeboWorldRoot,
            Transform::from_rotation(Quat::from_rotation_x(-FRAC_PI_2)),
            Visibility::default(),
        ))
        .id();
    sync.world_root = Some(root);
}

fn pose_to_transform(p: &PoseWire) -> Transform {
    let q = Quat::from_xyzw(p.qx, p.qy, p.qz, p.qw);
    let rot = if q.length_squared() > 1e-8 {
        q.normalize()
    } else {
        Quat::IDENTITY
    };
    Transform {
        translation: Vec3::new(p.px, p.py, p.pz),
        rotation: rot,
        scale: Vec3::ONE,
    }
}

fn build_standard_material(
    mat: &PbrMaterialWire,
    double_sided: bool,
    alpha_mode: u8,
    textures: &HashMap<u64, Handle<Image>>,
) -> StandardMaterial {
    let base_color = Color::srgba(
        mat.base_color[0],
        mat.base_color[1],
        mat.base_color[2],
        mat.base_color[3],
    );
    let emissive = LinearRgba::rgb(mat.emissive[0], mat.emissive[1], mat.emissive[2]);

    let bevy_alpha = match alpha_mode {
        1 => AlphaMode::Mask(mat.alpha_cutoff),
        2 => AlphaMode::Blend,
        _ => {
            if mat.base_color[3] < 0.99 {
                AlphaMode::Blend
            } else {
                AlphaMode::Opaque
            }
        }
    };

    let base_tex_id = mat.base_color_tex;
    let normal_tex_id = mat.normal_tex;
    let mr_tex_id = mat.metallic_roughness_tex;
    let occ_tex_id = mat.occlusion_tex;
    let emis_tex_id = mat.emissive_tex;

    let base_color_texture = (base_tex_id != 0)
        .then(|| textures.get(&base_tex_id).cloned())
        .flatten();
    let normal_map_texture = (normal_tex_id != 0)
        .then(|| textures.get(&normal_tex_id).cloned())
        .flatten();
    let metallic_roughness_texture = (mr_tex_id != 0)
        .then(|| textures.get(&mr_tex_id).cloned())
        .flatten();
    let occlusion_texture = (occ_tex_id != 0)
        .then(|| textures.get(&occ_tex_id).cloned())
        .flatten();
    let emissive_texture = (emis_tex_id != 0)
        .then(|| textures.get(&emis_tex_id).cloned())
        .flatten();

    // When a metallic_roughness_texture is bound, Bevy multiplies factors * texture;
    // make sure roughness/metallic aren't forced to 0 when map supplies them.
    let perceptual_roughness = if metallic_roughness_texture.is_some() {
        mat.roughness.max(0.05)
    } else {
        mat.roughness.clamp(0.04, 1.0)
    };
    let metallic = mat.metallic.clamp(0.0, 1.0);

    StandardMaterial {
        base_color,
        base_color_texture,
        emissive,
        emissive_texture,
        metallic,
        perceptual_roughness,
        metallic_roughness_texture,
        normal_map_texture,
        occlusion_texture,
        reflectance: mat.reflectance.clamp(0.0, 1.0),
        double_sided,
        cull_mode: if double_sided { None } else { Some(wgpu::Face::Back) },
        alpha_mode: bevy_alpha,
        ..default()
    }
}

pub fn process_scene_events_system(
    mut commands: Commands,
    mut sync: ResMut<SceneSyncState>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut images: ResMut<Assets<Image>>,
    asset_server: Res<AssetServer>,
    mut clear_color: ResMut<ClearColor>,
    mut quality: ResMut<QualityState>,
    mut cam_state: ResMut<OrbitCameraState>,
    swapchain: Res<ExternalSwapchain>,
    mut transform_q: Query<&mut Transform>,
) {
    // Periodically ask Gazebo for a full scene snapshot until we receive one
    if !sync.received_snapshot && sync.last_snapshot_req.elapsed() >= Duration::from_millis(500) {
        if let Some(ref pub_h) = sync.snapshot_pub {
            let req = SnapshotRequestWire {
                client_id: sync.client_id,
            };
            let pkt = encode_packet(MSG_SNAPSHOT_REQUEST, 1, &req);
            pub_h.publish(&pkt, false);
        }
        sync.last_snapshot_req = Instant::now();
    }

    let Some(root_entity) = sync.world_root else {
        return;
    };

    let mut samples = Vec::new();
    if let Some(ref rx) = sync.scene_rx {
        while let Ok(s) = rx.try_recv() {
            samples.push(s);
        }
    }

    for sample in samples {
        if sample.is_shm {
            sync.shm_sample_count += 1;
        } else {
            sync.raw_sample_count += 1;
        }
        let Some((hdr, payload)) = parse_header(&sample.data) else {
            continue;
        };

        match hdr.msg_type {
            MSG_CLEAR_WORLD => {
                for (_, e) in sync.parent_entities.drain() {
                    commands.entity(e).despawn();
                }
                sync.visual_entities.clear();
                for (_, e) in sync.light_entities.drain() {
                    commands.entity(e).despawn();
                }
                info!("Cleared scene entities for fresh Gazebo snapshot");
            }
            MSG_WORLD_CONFIG => {
                if let Some((cfg, off)) = read_pod::<WorldConfigWire>(payload, 0) {
                    sync.received_snapshot = true;
                    clear_color.0 = Color::srgba(
                        cfg.background[0],
                        cfg.background[1],
                        cfg.background[2],
                        1.0,
                    );
                    let amb_lum = (cfg.ambient[0] + cfg.ambient[1] + cfg.ambient[2]) / 3.0;
                    quality.ambient_color =
                        Color::srgb(cfg.ambient[0], cfg.ambient[1], cfg.ambient[2]);
                    quality.ambient_brightness = (amb_lum * 600.0).clamp(20.0, 900.0);
                    quality.world_has_sky = cfg.has_sky != 0;
                    quality.sky_intensity = if cfg.sky_intensity > 0.01 {
                        cfg.sky_intensity
                    } else {
                        1.0
                    };

                    // Optional HDR environment map path (e.g. indoor warehouse IBL probe)
                    let env_len = cfg.env_map_len as usize;
                    let env_path = if env_len > 0 && off + env_len <= payload.len() {
                        Some(String::from_utf8_lossy(&payload[off..off + env_len]).into_owned())
                    } else {
                        None
                    };
                    if env_path != quality.env_map_path {
                        quality.env_map_path = env_path.clone();
                        quality.env_map_cubemap = env_path
                            .as_deref()
                            .and_then(|p| crate::env_map::load_equirect_hdr_as_cubemap(p, 256))
                            .map(|img| images.add(img));
                        quality.env_map_dirty = true;
                    }
                    quality.env_map_intensity = quality.sky_intensity;
                    quality.dirty = true;

                    if cfg.has_camera_pose != 0 {
                        let pose = cfg.camera_pose;
                        cam_state.set_from_gazebo_pose(&pose, cfg.camera_hfov);
                    }
                    info!(
                        "Received WorldConfig (sky={}, env_map={:?}, env_intensity={:.1}, ambient={:?}, bg={:?}, shm_msgs={}, raw_msgs={})",
                        cfg.has_sky != 0,
                        quality.env_map_path,
                        quality.env_map_intensity,
                        { cfg.ambient },
                        { cfg.background },
                        sync.shm_sample_count,
                        sync.raw_sample_count
                    );
                }
            }
            MSG_TEXTURE_DATA => {
                if let Some((td, off)) = read_pod::<TextureDataWire>(payload, 0) {
                    let tex_id = td.texture_id;
                    let w = td.width;
                    let h = td.height;
                    let data_len = td.data_len as usize;
                    if w > 0 && h > 0 && off + data_len <= payload.len() {
                        let format = if td.format == 0 {
                            TextureFormat::Rgba8UnormSrgb
                        } else {
                            TextureFormat::Rgba8Unorm
                        };
                        let mut img = Image::new(
                            Extent3d {
                                width: w,
                                height: h,
                                depth_or_array_layers: 1,
                            },
                            TextureDimension::D2,
                            payload[off..off + data_len].to_vec(),
                            format,
                            RenderAssetUsages::default(),
                        );
                        img.sampler = ImageSampler::Descriptor(ImageSamplerDescriptor {
                            address_mode_u: ImageAddressMode::Repeat,
                            address_mode_v: ImageAddressMode::Repeat,
                            address_mode_w: ImageAddressMode::Repeat,
                            mag_filter: ImageFilterMode::Linear,
                            min_filter: ImageFilterMode::Linear,
                            mipmap_filter: ImageFilterMode::Linear,
                            anisotropy_clamp: 16,
                            ..default()
                        });
                        let handle = images.add(img);
                        sync.streamed_textures.insert(tex_id, handle);
                    }
                }
            }
            MSG_MESH_DATA => {
                if let Some((md, mut off)) = read_pod::<MeshDataWire>(payload, 0) {
                    let mesh_id = md.mesh_id;
                    let vcount = md.vertex_count as usize;
                    let icount = md.index_count as usize;
                    if vcount == 0 {
                        continue;
                    }
                    let Some((pos_flat, next_off)) = read_f32_vec(payload, off, vcount * 3) else {
                        continue;
                    };
                    off = next_off;

                    let positions: Vec<[f32; 3]> = pos_flat
                        .chunks_exact(3)
                        .map(|c| [c[0], c[1], c[2]])
                        .collect();

                    let mut mesh = Mesh::new(
                        PrimitiveTopology::TriangleList,
                        RenderAssetUsages::default(),
                    );
                    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);

                    let mut has_normals = false;
                    if md.has_normals != 0 {
                        if let Some((n_flat, next_off)) = read_f32_vec(payload, off, vcount * 3) {
                            off = next_off;
                            let normals: Vec<[f32; 3]> = n_flat
                                .chunks_exact(3)
                                .map(|c| [c[0], c[1], c[2]])
                                .collect();
                            mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
                            has_normals = true;
                        }
                    }

                    let mut has_uvs = false;
                    if md.has_uvs != 0 {
                        if let Some((uv_flat, next_off)) = read_f32_vec(payload, off, vcount * 2) {
                            off = next_off;
                            let uvs: Vec<[f32; 2]> =
                                uv_flat.chunks_exact(2).map(|c| [c[0], c[1]]).collect();
                            mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, uvs);
                            has_uvs = true;
                        }
                    }

                    let mut has_tangents = false;
                    if md.has_tangents != 0 {
                        if let Some((t_flat, next_off)) = read_f32_vec(payload, off, vcount * 4) {
                            off = next_off;
                            let tangents: Vec<[f32; 4]> = t_flat
                                .chunks_exact(4)
                                .map(|c| [c[0], c[1], c[2], c[3]])
                                .collect();
                            mesh.insert_attribute(Mesh::ATTRIBUTE_TANGENT, tangents);
                            has_tangents = true;
                        }
                    }

                    if icount > 0 {
                        if let Some((indices, _)) = read_u32_vec(payload, off, icount) {
                            mesh.insert_indices(Indices::U32(indices));
                        }
                    }

                    if !has_normals {
                        mesh.compute_smooth_normals();
                    }
                    if !has_tangents && has_uvs {
                        let _ = mesh.generate_tangents();
                    }

                    let handle = meshes.add(mesh);
                    sync.streamed_meshes.insert(mesh_id, handle);
                }
            }
            MSG_SPAWN_VISUAL => {
                let Some((sv, off)) = read_pod::<SpawnVisualWire>(payload, 0) else {
                    continue;
                };
                sync.received_snapshot = true;
                let parent_id = sv.parent_entity_id;
                let visual_id = sv.visual_id;

                // Ensure parent entity exists under GazeboWorldRoot
                let parent_entity = if let Some(&existing) = sync.parent_entities.get(&parent_id) {
                    existing
                } else {
                    let parent_pose = sv.parent_pose;
                    let e = commands
                        .spawn((
                            pose_to_transform(&parent_pose),
                            Visibility::default(),
                            ChildOf(root_entity),
                        ))
                        .id();
                    sync.parent_entities.insert(parent_id, e);
                    e
                };

                // Replace previous visual if re-spawned
                if let Some(old_vis) = sync.visual_entities.remove(&visual_id) {
                    commands.entity(old_vis).despawn();
                }

                let local_pose = sv.local_pose;
                let mut local_tf = pose_to_transform(&local_pose);
                let p = sv.params;
                let mat_wire = sv.material;

                match sv.kind {
                    0 => {
                        // Box
                        let mesh_h = meshes.add(Cuboid::new(p[0].max(1e-4), p[1].max(1e-4), p[2].max(1e-4)));
                        let mat_h = materials.add(build_standard_material(
                            &mat_wire,
                            sv.double_sided != 0,
                            sv.alpha_mode,
                            &sync.streamed_textures,
                        ));
                        let ve = commands
                            .spawn((
                                Mesh3d(mesh_h),
                                MeshMaterial3d(mat_h),
                                local_tf,
                                ChildOf(parent_entity),
                            ))
                            .id();
                        sync.visual_entities.insert(visual_id, ve);
                    }
                    1 => {
                        // Sphere
                        let mesh_h = meshes.add(Sphere::new(p[0].max(1e-4)).mesh().uv(36, 20));
                        let mat_h = materials.add(build_standard_material(
                            &mat_wire,
                            sv.double_sided != 0,
                            sv.alpha_mode,
                            &sync.streamed_textures,
                        ));
                        let ve = commands
                            .spawn((
                                Mesh3d(mesh_h),
                                MeshMaterial3d(mat_h),
                                local_tf,
                                ChildOf(parent_entity),
                            ))
                            .id();
                        sync.visual_entities.insert(visual_id, ve);
                    }
                    2 => {
                        // Cylinder (Gazebo Z-axis -> rotate Bevy Y-axis by +90 deg X)
                        let mesh_h = meshes.add(Cylinder::new(p[0].max(1e-4), p[1].max(1e-4)));
                        let mat_h = materials.add(build_standard_material(
                            &mat_wire,
                            sv.double_sided != 0,
                            sv.alpha_mode,
                            &sync.streamed_textures,
                        ));
                        local_tf.rotation = local_tf.rotation * Quat::from_rotation_x(FRAC_PI_2);
                        let ve = commands
                            .spawn((
                                Mesh3d(mesh_h),
                                MeshMaterial3d(mat_h),
                                local_tf,
                                ChildOf(parent_entity),
                            ))
                            .id();
                        sync.visual_entities.insert(visual_id, ve);
                    }
                    3 => {
                        // Capsule (Gazebo Z-axis -> rotate Bevy Y-axis by +90 deg X)
                        let mesh_h = meshes.add(Capsule3d::new(p[0].max(1e-4), p[1].max(1e-4)));
                        let mat_h = materials.add(build_standard_material(
                            &mat_wire,
                            sv.double_sided != 0,
                            sv.alpha_mode,
                            &sync.streamed_textures,
                        ));
                        local_tf.rotation = local_tf.rotation * Quat::from_rotation_x(FRAC_PI_2);
                        let ve = commands
                            .spawn((
                                Mesh3d(mesh_h),
                                MeshMaterial3d(mat_h),
                                local_tf,
                                ChildOf(parent_entity),
                            ))
                            .id();
                        sync.visual_entities.insert(visual_id, ve);
                    }
                    4 => {
                        // Plane (Gazebo normal +Z -> rotate Bevy Plane3d normal +Y by +90 deg X)
                        let mesh_h = meshes.add(
                            Plane3d::default()
                                .mesh()
                                .size(p[0].max(1e-2), p[1].max(1e-2)),
                        );
                        let mat_h = materials.add(build_standard_material(
                            &mat_wire,
                            true,
                            sv.alpha_mode,
                            &sync.streamed_textures,
                        ));
                        local_tf.rotation = local_tf.rotation * Quat::from_rotation_x(FRAC_PI_2);
                        let ve = commands
                            .spawn((
                                Mesh3d(mesh_h),
                                MeshMaterial3d(mat_h),
                                local_tf,
                                ChildOf(parent_entity),
                            ))
                            .id();
                        sync.visual_entities.insert(visual_id, ve);
                    }
                    5 => {
                        // StreamedMesh (.dae / .obj / .stl extracted by gz-common over Zenoh SHM)
                        let mesh_id = sv.mesh_id;
                        if let Some(mesh_h) = sync.streamed_meshes.get(&mesh_id).cloned() {
                            local_tf.scale = Vec3::new(p[0], p[1], p[2]);
                            let mat_h = materials.add(build_standard_material(
                                &mat_wire,
                                sv.double_sided != 0,
                                sv.alpha_mode,
                                &sync.streamed_textures,
                            ));
                            let ve = commands
                                .spawn((
                                    Mesh3d(mesh_h),
                                    MeshMaterial3d(mat_h),
                                    local_tf,
                                    ChildOf(parent_entity),
                                ))
                                .id();
                            sync.visual_entities.insert(visual_id, ve);
                        } else {
                            warn!("SpawnVisual referenced missing streamed mesh_id={mesh_id}");
                        }
                    }
                    6 => {
                        // GltfUri (.glb / .gltf loaded natively by Bevy's glTF PBR loader!)
                        let uri_len = sv.uri_len as usize;
                        if uri_len > 0 && off + uri_len <= payload.len() {
                            let uri =
                                String::from_utf8_lossy(&payload[off..off + uri_len]).into_owned();
                            local_tf.scale = Vec3::new(p[0], p[1], p[2]);
                            let scene_handle: Handle<WorldAsset> =
                                asset_server.load(GltfAssetLabel::Scene(0).from_asset(uri.clone()));
                            let ve = commands
                                .spawn((
                                    WorldAssetRoot(scene_handle),
                                    local_tf,
                                    ChildOf(parent_entity),
                                ))
                                .id();
                            sync.visual_entities.insert(visual_id, ve);
                            info!("Spawned native glTF scene '{uri}' (visual={visual_id})");
                        }
                    }
                    7 => {
                        // Cone (Gazebo Z-axis -> rotate Bevy Y-axis by +90 deg X)
                        let mesh_h = meshes.add(Cone::new(p[0].max(1e-4), p[1].max(1e-4)));
                        let mat_h = materials.add(build_standard_material(
                            &mat_wire,
                            sv.double_sided != 0,
                            sv.alpha_mode,
                            &sync.streamed_textures,
                        ));
                        local_tf.rotation = local_tf.rotation * Quat::from_rotation_x(FRAC_PI_2);
                        let ve = commands
                            .spawn((
                                Mesh3d(mesh_h),
                                MeshMaterial3d(mat_h),
                                local_tf,
                                ChildOf(parent_entity),
                            ))
                            .id();
                        sync.visual_entities.insert(visual_id, ve);
                    }
                    8 => {
                        // Ellipsoid (unit sphere scaled by semi-axes rx, ry, rz)
                        let mesh_h = meshes.add(Sphere::new(1.0).mesh().uv(36, 20));
                        let mat_h = materials.add(build_standard_material(
                            &mat_wire,
                            sv.double_sided != 0,
                            sv.alpha_mode,
                            &sync.streamed_textures,
                        ));
                        local_tf.scale = Vec3::new(p[0].max(1e-4), p[1].max(1e-4), p[2].max(1e-4));
                        let ve = commands
                            .spawn((
                                Mesh3d(mesh_h),
                                MeshMaterial3d(mat_h),
                                local_tf,
                                ChildOf(parent_entity),
                            ))
                            .id();
                        sync.visual_entities.insert(visual_id, ve);
                    }
                    _ => {}
                }
            }
            MSG_DESPAWN_ENTITY => {
                if let Some((de, _)) = read_pod::<DespawnEntityWire>(payload, 0) {
                    let eid = de.entity_id;
                    if let Some(e) = sync.parent_entities.remove(&eid) {
                        commands.entity(e).despawn();
                    }
                    if let Some(e) = sync.light_entities.remove(&eid) {
                        commands.entity(e).despawn();
                    }
                }
            }
            MSG_LIGHT_UPSERT => {
                if let Some((lw, _)) = read_pod::<LightUpsertWire>(payload, 0) {
                    sync.received_snapshot = true;
                    let eid = lw.entity_id;
                    if let Some(old) = sync.light_entities.remove(&eid) {
                        commands.entity(old).despawn();
                    }

                    let r_gz_to_bevy = Quat::from_rotation_x(-FRAC_PI_2);
                    let pose = lw.pose;
                    let pos_gz = Vec3::new(pose.px, pose.py, pose.pz);
                    let q_gz = Quat::from_xyzw(pose.qx, pose.qy, pose.qz, pose.qw);
                    let q_gz = if q_gz.length_squared() > 1e-8 {
                        q_gz.normalize()
                    } else {
                        Quat::IDENTITY
                    };
                    let dir_local = Vec3::new(lw.direction[0], lw.direction[1], lw.direction[2]);
                    let dir_gz = (q_gz * dir_local).normalize_or(Vec3::NEG_Z);

                    let pos_bevy = r_gz_to_bevy * pos_gz;
                    let dir_bevy = (r_gz_to_bevy * dir_gz).normalize_or(Vec3::NEG_Y);
                    let up = if dir_bevy.y.abs() > 0.98 {
                        Vec3::Z
                    } else {
                        Vec3::Y
                    };
                    let light_tf = Transform::from_translation(pos_bevy).looking_to(dir_bevy, up);

                    let color = Color::srgb(lw.color[0], lw.color[1], lw.color[2]);
                    let use_shadows =
                        (quality.flags & QUALITY_SHADOWS) != 0 && lw.cast_shadows != 0;
                    let use_pcss = use_shadows && (quality.flags & QUALITY_SSAO) != 0;
                    let meta = SdfLightMeta {
                        cast_shadows: lw.cast_shadows != 0,
                    };

                    let ent = match lw.light_type {
                        0 => {
                            // Directional light (SDF intensity ~1.0 -> ~10,000 lux daylight)
                            commands
                                .spawn((
                                    DirectionalLight {
                                        color,
                                        illuminance: (lw.intensity.max(0.05) * 8000.0).min(100_000.0),
                                        shadow_maps_enabled: use_shadows,
                                        contact_shadows_enabled: false,
                                        soft_shadow_size: if use_pcss { Some(1.5) } else { None },
                                        ..default()
                                    },
                                    light_tf,
                                    meta,
                                ))
                                .id()
                        }
                        1 => {
                            // Point light (SDF intensity -> lumens)
                            let range = lw.range.clamp(1.0, 200.0);
                            commands
                                .spawn((
                                    PointLight {
                                        color,
                                        intensity: lw.intensity.max(0.1) * 15_000.0,
                                        range,
                                        radius: 0.35,
                                        shadow_maps_enabled: use_shadows,
                                        contact_shadows_enabled: false,
                                        soft_shadows_enabled: use_pcss,
                                        ..default()
                                    },
                                    light_tf,
                                    meta,
                                ))
                                .id()
                        }
                        2 => {
                            // Spot light (wide physical radius enables soft PCSS penumbra under TAA)
                            let range = lw.range.clamp(1.0, 200.0);
                            let outer = (lw.outer_angle * 0.5).clamp(0.05, FRAC_PI_2 - 0.02);
                            let inner = (lw.inner_angle * 0.5).clamp(0.01, outer);
                            commands
                                .spawn((
                                    SpotLight {
                                        color,
                                        intensity: lw.intensity.max(0.1) * 25_000.0,
                                        range,
                                        radius: 0.70,
                                        inner_angle: inner,
                                        outer_angle: outer,
                                        shadow_maps_enabled: use_shadows,
                                        contact_shadows_enabled: false,
                                        soft_shadows_enabled: use_pcss,
                                        shadow_depth_bias: 0.03,
                                        shadow_normal_bias: 0.6,
                                        ..default()
                                    },
                                    light_tf,
                                    meta,
                                ))
                                .id()
                        }
                        _ => {
                            // Rectangular LTC area light (light_type == 3).
                            // In Gazebo Z-up, identity pose lies in XY and emits along -Z;
                            // Rx(-90 deg) rotates it cleanly into Bevy Y-up (emits along -Y).
                            let rect_tf = Transform {
                                translation: pos_bevy,
                                rotation: r_gz_to_bevy * q_gz,
                                scale: Vec3::ONE,
                            };
                            commands
                                .spawn((
                                    RectLight {
                                        color,
                                        intensity: lw.intensity.max(1.0),
                                        range: lw.range.clamp(1.0, 200.0),
                                        width: lw.inner_angle.max(0.05),
                                        height: lw.outer_angle.max(0.05),
                                    },
                                    rect_tf,
                                    meta,
                                ))
                                .id()
                        }
                    };
                    sync.light_entities.insert(eid, ent);
                }
            }
            MSG_TRANSFORM_BATCH => {
                if let Some((_bh, mut off)) = read_pod::<TransformBatchHeaderWire>(payload, 0) {
                    let count = hdr.count as usize;
                    for _ in 0..count {
                        let Some((item, next_off)) = read_pod::<TransformItemWire>(payload, off)
                        else {
                            break;
                        };
                        off = next_off;
                        let eid = item.entity_id;
                        if let Some(&ent) = sync.parent_entities.get(&eid) {
                            if let Ok(mut tf) = transform_q.get_mut(ent) {
                                let pose = item.pose;
                                let next_tf = pose_to_transform(&pose);
                                tf.translation = next_tf.translation;
                                tf.rotation = next_tf.rotation;
                            }
                        }
                    }
                }
            }
            _ => {}
        }
    }

    swapchain
        .shared
        .entity_count
        .store(sync.visual_entities.len() as u32, Ordering::Relaxed);
}
