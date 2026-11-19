mod camera_control;
mod env_map;
mod mipmap;
mod protocol;
mod quality;
mod scene_sync;
mod swapchain;
mod zenoh_ffi;

use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::time::Duration;

use bevy::app::ScheduleRunnerPlugin;
use bevy::asset::{AssetEventSystems, UnapprovedPathMode};
use bevy::gltf::GltfPlugin;
use bevy::image::ImageSamplerDescriptor;
use bevy::light::DirectionalLight;
use bevy::log::LogPlugin;
use bevy::prelude::*;
use bevy::window::{ExitCondition, WindowPlugin};
use bevy::winit::WinitPlugin;

use camera_control::{
    GuiInputChannel, OrbitCameraState, process_gui_input_system, spawn_viewport_camera_system,
    update_viewport_camera_system,
};
use protocol::{
    QUALITY_AGX, QUALITY_BLOOM, QUALITY_DEFAULT, QUALITY_IBL, QUALITY_MSAA, QUALITY_SHADOWS,
    QUALITY_SSAO,
};
use quality::{QualityState, apply_quality_settings_system};
use scene_sync::{
    SceneSyncState, SdfLightMeta, process_scene_events_system, spawn_world_root_system,
};
use swapchain::{ExternalSwapchain, ExternalSwapchainPlugin, SwapchainSharedState};
use zenoh_ffi::ZenohSession;

struct CliArgs {
    session: String,
    endpoint: String,
    width: u32,
    height: u32,
    fps: f64,
    quality_flags: u32,
    standalone_demo: bool,
    screenshot_out: Option<PathBuf>,
    screenshot_after_frames: u64,
    exit_after_screenshot: bool,
    camera_pose_override: Option<[f32; 6]>, // x y z roll pitch yaw (Gazebo coords)
    camera_hfov: f32,
}

fn parse_args() -> CliArgs {
    let mut args = CliArgs {
        session: std::env::var("GZ_EXT_RENDER_SESSION").unwrap_or_else(|_| "default".into()),
        endpoint: std::env::var("GZ_EXT_RENDER_ENDPOINT")
            .unwrap_or_else(|_| "tcp/127.0.0.1:17447".into()),
        width: 1280,
        height: 720,
        fps: 60.0,
        quality_flags: QUALITY_DEFAULT,
        standalone_demo: false,
        screenshot_out: None,
        screenshot_after_frames: 60,
        exit_after_screenshot: false,
        camera_pose_override: None,
        camera_hfov: 1.0472,
    };

    let argv: Vec<String> = std::env::args().collect();
    let mut i = 1;
    while i < argv.len() {
        match argv[i].as_str() {
            "--session" if i + 1 < argv.len() => {
                args.session = argv[i + 1].clone();
                i += 2;
            }
            "--endpoint" if i + 1 < argv.len() => {
                args.endpoint = argv[i + 1].clone();
                i += 2;
            }
            "--width" if i + 1 < argv.len() => {
                args.width = argv[i + 1].parse().unwrap_or(1280);
                i += 2;
            }
            "--height" if i + 1 < argv.len() => {
                args.height = argv[i + 1].parse().unwrap_or(720);
                i += 2;
            }
            "--fps" if i + 1 < argv.len() => {
                args.fps = argv[i + 1].parse().unwrap_or(60.0);
                i += 2;
            }
            "--preset" if i + 1 < argv.len() => {
                args.quality_flags = match argv[i + 1].as_str() {
                    "strict" => QUALITY_SHADOWS | QUALITY_MSAA,
                    "ssao" => {
                        QUALITY_IBL | QUALITY_SSAO | QUALITY_BLOOM | QUALITY_AGX | QUALITY_SHADOWS
                    }
                    _ => QUALITY_DEFAULT,
                };
                i += 2;
            }
            "--standalone-demo" => {
                args.standalone_demo = true;
                i += 1;
            }
            "--screenshot" if i + 1 < argv.len() => {
                args.screenshot_out = Some(PathBuf::from(&argv[i + 1]));
                i += 2;
            }
            "--screenshot-after-frames" if i + 1 < argv.len() => {
                args.screenshot_after_frames = argv[i + 1].parse().unwrap_or(60);
                i += 2;
            }
            "--exit-after-screenshot" => {
                args.exit_after_screenshot = true;
                i += 1;
            }
            "--camera-pose" if i + 6 < argv.len() => {
                let mut v = [0.0f32; 6];
                for k in 0..6 {
                    v[k] = argv[i + 1 + k].parse().unwrap_or(0.0);
                }
                args.camera_pose_override = Some(v);
                i += 7;
            }
            "--camera-hfov" if i + 1 < argv.len() => {
                args.camera_hfov = argv[i + 1].parse().unwrap_or(1.0472);
                i += 2;
            }
            _ => {
                i += 1;
            }
        }
    }
    args
}

#[derive(Resource)]
struct AutoScreenshotConfig {
    output: Option<PathBuf>,
    after_frames: u64,
    exit_after: bool,
    requested: bool,
    wait_for_snapshot: bool,
    snapshot_frame: Option<u64>,
}

fn auto_screenshot_system(
    mut cfg: ResMut<AutoScreenshotConfig>,
    swapchain: Res<ExternalSwapchain>,
    sync: Res<SceneSyncState>,
    mut exit_writer: MessageWriter<AppExit>,
) {
    let Some(out_path) = cfg.output.clone() else {
        return;
    };
    let cur_frame = swapchain.shared.frame_counter.load(Ordering::Acquire);

    if cfg.wait_for_snapshot && !sync.received_snapshot {
        return;
    }
    if cfg.snapshot_frame.is_none() {
        cfg.snapshot_frame = Some(cur_frame);
    }
    let elapsed_frames = cur_frame.saturating_sub(cfg.snapshot_frame.unwrap_or(0));

    if !cfg.requested && elapsed_frames >= cfg.after_frames {
        if let Ok(mut pending) = swapchain.shared.pending_screenshot.lock() {
            *pending = Some(out_path.clone());
            cfg.requested = true;
            info!(
                "Triggering auto-screenshot to {} at frame {cur_frame}",
                out_path.display()
            );
        }
    }

    if cfg.requested && cfg.exit_after {
        let saved = swapchain
            .shared
            .last_saved_screenshot_frame
            .load(Ordering::Acquire);
        if saved > 0 {
            info!("Screenshot captured at frame {saved}; exiting cleanly.");
            exit_writer.write(AppExit::Success);
        }
    }
}

#[derive(Component)]
struct DemoOrbitBox;

fn spawn_standalone_demo_system(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    sync: Res<SceneSyncState>,
    mut cam_state: ResMut<OrbitCameraState>,
) {
    let Some(root) = sync.world_root else {
        return;
    };

    // Checkerboard-like ground plane
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(20.0, 20.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.32, 0.34, 0.38),
            perceptual_roughness: 0.75,
            metallic: 0.05,
            ..default()
        })),
        Transform::from_rotation(Quat::from_rotation_x(std::f32::consts::FRAC_PI_2)),
        ChildOf(root),
    ));

    // Metallic sphere + orbiting crimson PBR box
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(0.65).mesh().uv(48, 24))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.88, 0.72, 0.32),
            metallic: 0.95,
            perceptual_roughness: 0.12,
            ..default()
        })),
        Transform::from_xyz(0.0, 0.0, 0.65),
        ChildOf(root),
    ));

    commands.spawn((
        DemoOrbitBox,
        Mesh3d(meshes.add(Cuboid::new(0.7, 0.7, 0.7))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.85, 0.16, 0.14),
            metallic: 0.25,
            perceptual_roughness: 0.28,
            ..default()
        })),
        Transform::from_xyz(1.8, 0.0, 0.7),
        ChildOf(root),
    ));

    // Directional sunlight
    commands.spawn((
        DirectionalLight {
            color: Color::srgb(1.0, 0.97, 0.92),
            illuminance: 16_000.0,
            shadow_maps_enabled: true,
            ..default()
        },
        Transform::from_xyz(4.0, 6.0, 4.0).looking_at(Vec3::ZERO, Vec3::Y),
        SdfLightMeta { cast_shadows: true },
    ));

    cam_state.focus = Vec3::new(0.0, 0.6, 0.0);
    cam_state.distance = 4.6;
    cam_state.yaw = 0.65;
    cam_state.pitch = 0.35;
    cam_state.save_initial();
    cam_state.dirty = true;
}

fn animate_standalone_demo_system(
    time: Res<Time>,
    mut q: Query<&mut Transform, With<DemoOrbitBox>>,
) {
    let t = time.elapsed_secs();
    for mut tf in &mut q {
        tf.translation = Vec3::new(1.8 * t.cos(), 1.8 * t.sin(), 0.7 + 0.15 * (2.0 * t).sin());
        tf.rotation = Quat::from_rotation_z(t * 0.9) * Quat::from_rotation_x(t * 0.4);
    }
}

fn euler_rpy_to_quat(roll: f32, pitch: f32, yaw: f32) -> Quat {
    // Gazebo SDF convention: intrinsic Z-Y'-X'' (yaw, pitch, roll)
    Quat::from_rotation_z(yaw) * Quat::from_rotation_y(pitch) * Quat::from_rotation_x(roll)
}

fn main() {
    let args = parse_args();
    let prefix = format!("gz_ext/{}", args.session);
    let service_name = format!("org.gazebosim.ext_renderer.{}", args.session);

    // Open Zenoh 1.8 peer listener with POSIX SHM enabled
    let z_sess = ZenohSession::open(&args.endpoint, true, 16 * 1024 * 1024);
    let (scene_sub, scene_rx, snapshot_pub, input_sub, input_rx, slot_sub, slot_rx, frame_pub) =
        if let Some(ref s) = z_sess {
            let (ss, sr) = s
                .declare_subscriber(&format!("{prefix}/scene/events"))
                .unzip();
            let sp = s.declare_publisher(&format!("{prefix}/scene/snapshot_req"));
            let (is, ir) = s
                .declare_subscriber(&format!("{prefix}/gui/input"))
                .unzip();
            let (ls, lr) = s
                .declare_subscriber(&format!("{prefix}/gui/slot_acquired"))
                .unzip();
            let fp = s.declare_publisher(&format!("{prefix}/gui/frame_ready"));
            (ss, sr, sp, is, ir, ls, lr, fp)
        } else {
            (None, None, None, None, None, None, None, None)
        };

    let shared = Arc::new(SwapchainSharedState::new(&service_name, frame_pub));
    shared
        .quality_flags
        .store(args.quality_flags, Ordering::Relaxed);

    let mut cam_state = OrbitCameraState::default();
    if let Some([x, y, z, roll, pitch, yaw]) = args.camera_pose_override {
        let q = euler_rpy_to_quat(roll, pitch, yaw);
        let pw = protocol::PoseWire {
            px: x,
            py: y,
            pz: z,
            qx: q.x,
            qy: q.y,
            qz: q.z,
            qw: q.w,
        };
        cam_state.set_from_gazebo_pose(&pw, args.camera_hfov);
    }

    let mut quality = QualityState::default();
    quality.flags = args.quality_flags;
    if args.standalone_demo {
        quality.world_has_sky = true;
    }

    let frame_dt = Duration::from_secs_f64(1.0 / args.fps.clamp(5.0, 240.0));

    let mut default_sampler = ImageSamplerDescriptor::linear();
    default_sampler.set_anisotropic_filter(16);

    let mut app = App::new();
    app.add_plugins(
        DefaultPlugins
            .set(LogPlugin {
                filter: "info,bevy_gltf::loader=error,wgpu=error".into(),
                ..default()
            })
            .set(ImagePlugin {
                default_sampler: default_sampler.clone(),
            })
            .set(GltfPlugin {
                default_sampler,
                ..default()
            })
            .set(AssetPlugin {
                unapproved_path_mode: UnapprovedPathMode::Allow,
                ..default()
            })
            .set(WindowPlugin {
                primary_window: None,
                exit_condition: ExitCondition::DontExit,
                ..default()
            })
            .disable::<WinitPlugin>(),
    )
    .add_plugins(ScheduleRunnerPlugin::run_loop(frame_dt))
    .add_plugins(ExternalSwapchainPlugin {
        shared: shared.clone(),
        initial_width: args.width,
        initial_height: args.height,
    })
    .insert_resource(cam_state)
    .insert_resource(quality)
    .insert_resource(SceneSyncState::new(scene_sub, scene_rx, snapshot_pub))
    .insert_resource(GuiInputChannel {
        _input_sub: input_sub,
        input_rx,
        _slot_sub: slot_sub,
        slot_rx,
        screenshot_dir: std::env::current_dir().unwrap_or_else(|_| PathBuf::from("/tmp")),
        screenshot_seq: 0,
    })
    .insert_resource(AutoScreenshotConfig {
        output: args.screenshot_out.clone(),
        after_frames: args.screenshot_after_frames,
        exit_after: args.exit_after_screenshot,
        requested: false,
        wait_for_snapshot: !args.standalone_demo,
        snapshot_frame: None,
    })
    .add_systems(
        Startup,
        (spawn_world_root_system, spawn_viewport_camera_system).chain(),
    )
    .add_systems(
        Update,
        (
            process_scene_events_system,
            process_gui_input_system,
            apply_quality_settings_system,
            update_viewport_camera_system,
            auto_screenshot_system,
        )
            .chain(),
    )
    .add_systems(
        PostUpdate,
        mipmap::generate_texture_mipmaps_system.after(AssetEventSystems),
    );

    if args.standalone_demo {
        app.add_systems(
            Startup,
            spawn_standalone_demo_system.after(spawn_world_root_system),
        )
        .add_systems(Update, animate_standalone_demo_system);
    }

    // Keep ZenohSession alive for the full duration of the app
    let _keep_session = z_sess;
    app.run();
}
