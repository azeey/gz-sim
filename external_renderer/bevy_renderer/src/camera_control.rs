use std::f32::consts::FRAC_PI_2;
use std::path::PathBuf;
use std::sync::atomic::Ordering;

use bevy::camera::{RenderTarget, ShadowLodOrigin};
use bevy::prelude::*;
use crossbeam_channel::Receiver;

use crate::protocol::{
    GuiInputWire, MSG_GUI_INPUT, MSG_SLOT_ACQUIRED, PoseWire, QUALITY_AGX, QUALITY_BLOOM,
    QUALITY_IBL, QUALITY_SSAO, SlotAcquiredWire, parse_header, read_pod,
};
use crate::quality::QualityState;
use crate::swapchain::{ExternalSwapchain, VIEWPORT_HANDLE};
use crate::zenoh_ffi::{ReceivedSample, ZenohSubscriber};

#[derive(Component)]
pub struct MainViewportCamera;

/// Orbit/pan/zoom camera controller operating in Bevy Y-up world space
/// (matching Gazebo Z-up via the global Rx(-90 deg) GazeboWorldRoot transform).
#[derive(Resource)]
pub struct OrbitCameraState {
    pub focus: Vec3,
    pub yaw: f32,
    pub pitch: f32,
    pub distance: f32,
    pub hfov_rad: f32,
    pub initial_focus: Vec3,
    pub initial_yaw: f32,
    pub initial_pitch: f32,
    pub initial_distance: f32,
    pub initial_hfov_rad: f32,
    pub dragging_orbit: bool,
    pub dragging_pan: bool,
    pub last_mouse: Vec2,
    pub dirty: bool,
}

impl Default for OrbitCameraState {
    fn default() -> Self {
        let mut s = Self {
            focus: Vec3::ZERO,
            yaw: 0.75,
            pitch: 0.35,
            distance: 5.0,
            hfov_rad: 1.0472, // 60 deg
            initial_focus: Vec3::ZERO,
            initial_yaw: 0.75,
            initial_pitch: 0.35,
            initial_distance: 5.0,
            initial_hfov_rad: 1.0472,
            dragging_orbit: false,
            dragging_pan: false,
            last_mouse: Vec2::ZERO,
            dirty: true,
        };
        s.save_initial();
        s
    }
}

impl OrbitCameraState {
    pub fn save_initial(&mut self) {
        self.initial_focus = self.focus;
        self.initial_yaw = self.yaw;
        self.initial_pitch = self.pitch;
        self.initial_distance = self.distance;
        self.initial_hfov_rad = self.hfov_rad;
    }

    pub fn reset(&mut self) {
        self.focus = self.initial_focus;
        self.yaw = self.initial_yaw;
        self.pitch = self.initial_pitch;
        self.distance = self.initial_distance;
        self.hfov_rad = self.initial_hfov_rad;
        self.dirty = true;
    }

    /// Convert a Gazebo Z-up camera pose (+X forward, +Z up) into Bevy Y-up orbit parameters.
    pub fn set_from_gazebo_pose(&mut self, pose: &PoseWire, hfov_rad: f32) {
        let r_gz_to_bevy = Quat::from_rotation_x(-FRAC_PI_2);
        let p_gz = Vec3::new(pose.px, pose.py, pose.pz);
        let q_gz = Quat::from_xyzw(pose.qx, pose.qy, pose.qz, pose.qw).normalize();

        let pos_bevy = r_gz_to_bevy * p_gz;
        let fwd_bevy = (r_gz_to_bevy * (q_gz * Vec3::X)).normalize_or_zero();
        if fwd_bevy.length_squared() < 1e-6 {
            return;
        }

        // Choose focus along forward ray: intersect ground plane y=0 if looking downward, else 3m ahead.
        let dist = if fwd_bevy.y < -0.05 && pos_bevy.y > 0.05 {
            (-pos_bevy.y / fwd_bevy.y).clamp(1.0, 25.0)
        } else {
            pos_bevy.length().clamp(1.5, 15.0)
        };
        self.focus = pos_bevy + fwd_bevy * dist;
        let offset = pos_bevy - self.focus;
        self.distance = offset.length().max(0.2);
        let dir = offset / self.distance; // from focus toward camera
        self.pitch = dir.y.clamp(-0.999, 0.999).asin();
        self.yaw = dir.x.atan2(dir.z);
        let hfov_norm = if hfov_rad > 3.2 {
            hfov_rad.to_radians()
        } else {
            hfov_rad
        };
        if hfov_norm > 0.05 && hfov_norm < 3.0 {
            self.hfov_rad = hfov_norm;
        }
        self.save_initial();
        self.dirty = true;
    }

    pub fn compute_transform(&self) -> Transform {
        let cos_p = self.pitch.cos();
        let offset = Vec3::new(
            self.distance * cos_p * self.yaw.sin(),
            self.distance * self.pitch.sin(),
            self.distance * cos_p * self.yaw.cos(),
        );
        Transform::from_translation(self.focus + offset).looking_at(self.focus, Vec3::Y)
    }
}

#[derive(Resource)]
pub struct GuiInputChannel {
    pub _input_sub: Option<ZenohSubscriber>,
    pub input_rx: Option<Receiver<ReceivedSample>>,
    pub _slot_sub: Option<ZenohSubscriber>,
    pub slot_rx: Option<Receiver<ReceivedSample>>,
    pub screenshot_dir: PathBuf,
    pub screenshot_seq: u32,
}

pub fn spawn_viewport_camera_system(mut commands: Commands, cam_state: Res<OrbitCameraState>) {
    commands.spawn((
        MainViewportCamera,
        ShadowLodOrigin,
        Camera3d::default(),
        Camera {
            order: 0,
            ..default()
        },
        RenderTarget::TextureView(VIEWPORT_HANDLE),
        Projection::Perspective(PerspectiveProjection {
            fov: 0.785,
            near: 0.02,
            far: 1000.0,
            ..default()
        }),
        cam_state.compute_transform(),
    ));
}

pub fn process_gui_input_system(
    mut gui_ch: ResMut<GuiInputChannel>,
    mut cam_state: ResMut<OrbitCameraState>,
    mut swapchain: ResMut<ExternalSwapchain>,
    mut quality: ResMut<QualityState>,
) {
    // 1. Drain Qt active-slot feedback (triple-buffer mailbox coordination)
    if let Some(ref rx) = gui_ch.slot_rx {
        while let Ok(sample) = rx.try_recv() {
            if let Some((hdr, payload)) = parse_header(&sample.data) {
                if hdr.msg_type == MSG_SLOT_ACQUIRED {
                    if let Some((wire, _)) = read_pod::<SlotAcquiredWire>(payload, 0) {
                        swapchain
                            .shared
                            .set_qt_active_slot(wire.generation, wire.active_slot);
                    }
                }
            }
        }
    }

    // 2. Drain Qt input / resize / quality / screenshot commands
    let mut samples = Vec::new();
    if let Some(ref rx) = gui_ch.input_rx {
        while let Ok(sample) = rx.try_recv() {
            samples.push(sample);
        }
    }

    for sample in samples {
        let Some((hdr, payload)) = parse_header(&sample.data) else {
            continue;
        };
        if hdr.msg_type != MSG_GUI_INPUT {
            continue;
        }
        let Some((ev, off)) = read_pod::<GuiInputWire>(payload, 0) else {
            continue;
        };

        const INPUT_MOUSE_DOWN: u8 = 0;
        const INPUT_MOUSE_UP: u8 = 1;
        const INPUT_MOUSE_MOVE: u8 = 2;
        const INPUT_WHEEL: u8 = 3;
        const INPUT_KEY_DOWN: u8 = 4;
        const INPUT_RESIZE: u8 = 6;
        const INPUT_SET_CAMERA_POSE: u8 = 7;
        const INPUT_QUALITY_CONFIG: u8 = 8;
        const INPUT_SAVE_SCREENSHOT: u8 = 9;
        const INPUT_RESET_CAMERA: u8 = 10;

        // Qt button bitmask: Left=1, Right=2, Middle=4; Shift modifier bit=0x02000000 (or bit 0 normalized)
        const BTN_LEFT: u32 = 1;
        const BTN_RIGHT: u32 = 2;
        const BTN_MIDDLE: u32 = 4;
        const MOD_SHIFT: u32 = 1;

        match ev.kind {
            INPUT_MOUSE_DOWN => {
                cam_state.last_mouse = Vec2::new(ev.x, ev.y);
                let shift = (ev.modifiers & MOD_SHIFT) != 0;
                if ev.button == BTN_LEFT && !shift {
                    cam_state.dragging_orbit = true;
                } else if ev.button == BTN_MIDDLE || ev.button == BTN_RIGHT || (ev.button == BTN_LEFT && shift) {
                    cam_state.dragging_pan = true;
                }
            }
            INPUT_MOUSE_UP => {
                if ev.button == BTN_LEFT {
                    cam_state.dragging_orbit = false;
                    cam_state.dragging_pan = false;
                }
                if ev.button == BTN_MIDDLE || ev.button == BTN_RIGHT {
                    cam_state.dragging_pan = false;
                }
            }
            INPUT_MOUSE_MOVE => {
                let cur = Vec2::new(ev.x, ev.y);
                let delta = if ev.dx.abs() > 0.0 || ev.dy.abs() > 0.0 {
                    Vec2::new(ev.dx, ev.dy)
                } else {
                    cur - cam_state.last_mouse
                };
                cam_state.last_mouse = cur;

                if cam_state.dragging_orbit {
                    let sensitivity = 0.006;
                    cam_state.yaw -= delta.x * sensitivity;
                    cam_state.pitch = (cam_state.pitch + delta.y * sensitivity)
                        .clamp(-FRAC_PI_2 + 0.03, FRAC_PI_2 - 0.03);
                    cam_state.dirty = true;
                } else if cam_state.dragging_pan {
                    let tf = cam_state.compute_transform();
                    let pan_scale = cam_state.distance * 0.0018;
                    let right = tf.right().as_vec3();
                    let up = tf.up().as_vec3();
                    cam_state.focus += (-right * delta.x + up * delta.y) * pan_scale;
                    cam_state.dirty = true;
                }
            }
            INPUT_WHEEL => {
                let factor = (-ev.dy * 0.0012).exp();
                cam_state.distance = (cam_state.distance * factor).clamp(0.15, 250.0);
                cam_state.dirty = true;
            }
            INPUT_KEY_DOWN => {
                // ASCII key codes forwarded from Qt
                match (ev.key_code & 0xFF) as u8 {
                    b'R' | b'r' => cam_state.reset(),
                    b'1' => {
                        quality.flags ^= QUALITY_IBL;
                        quality.dirty = true;
                        swapchain.shared.quality_flags.store(quality.flags, Ordering::Relaxed);
                    }
                    b'2' => {
                        quality.flags ^= QUALITY_SSAO;
                        quality.dirty = true;
                        swapchain.shared.quality_flags.store(quality.flags, Ordering::Relaxed);
                    }
                    b'3' => {
                        quality.flags ^= QUALITY_BLOOM;
                        quality.dirty = true;
                        swapchain.shared.quality_flags.store(quality.flags, Ordering::Relaxed);
                    }
                    b'4' => {
                        quality.flags ^= QUALITY_AGX;
                        quality.dirty = true;
                        swapchain.shared.quality_flags.store(quality.flags, Ordering::Relaxed);
                    }
                    b'P' | b'p' => {
                        gui_ch.screenshot_seq += 1;
                        let path = gui_ch
                            .screenshot_dir
                            .join(format!("bevy_capture_{:04}.png", gui_ch.screenshot_seq));
                        if let Ok(mut pending) = swapchain.shared.pending_screenshot.lock() {
                            *pending = Some(path);
                        }
                    }
                    _ => {}
                }
            }
            INPUT_RESIZE => {
                // Keep exact viewport aspect ratio while capping HiDPI pixel count for smooth 60 FPS
                let dpr = ev.dpr.clamp(1.0, 2.0);
                let mut pw = (ev.width as f32) * dpr;
                let mut ph = (ev.height as f32) * dpr;
                let max_dim = 1920.0f32;
                let cur_max = pw.max(ph);
                if cur_max > max_dim {
                    let s = max_dim / cur_max;
                    pw *= s;
                    ph *= s;
                }
                swapchain.request_resize(pw.round() as u32, ph.round() as u32);
                cam_state.dirty = true;
            }
            INPUT_SET_CAMERA_POSE => {
                let pose = ev.camera_pose;
                cam_state.set_from_gazebo_pose(&pose, ev.camera_hfov);
            }
            INPUT_QUALITY_CONFIG => {
                quality.flags = ev.quality_flags;
                quality.dirty = true;
                swapchain
                    .shared
                    .quality_flags
                    .store(quality.flags, Ordering::Relaxed);
            }
            INPUT_SAVE_SCREENSHOT => {
                let path_len = ev.path_len as usize;
                let path = if path_len > 0 && off + path_len <= payload.len() {
                    let s = String::from_utf8_lossy(&payload[off..off + path_len]).into_owned();
                    PathBuf::from(s)
                } else {
                    gui_ch.screenshot_seq += 1;
                    gui_ch
                        .screenshot_dir
                        .join(format!("bevy_capture_{:04}.png", gui_ch.screenshot_seq))
                };
                if let Ok(mut pending) = swapchain.shared.pending_screenshot.lock() {
                    *pending = Some(path);
                }
            }
            INPUT_RESET_CAMERA => {
                cam_state.reset();
            }
            _ => {}
        }
    }
}

pub fn update_viewport_camera_system(
    mut cam_state: ResMut<OrbitCameraState>,
    swapchain: Res<ExternalSwapchain>,
    mut cam_q: Query<(&mut Transform, &mut Projection), With<MainViewportCamera>>,
) {
    if !cam_state.dirty {
        return;
    }
    let Ok((mut tf, mut proj)) = cam_q.single_mut() else {
        return;
    };
    *tf = cam_state.compute_transform();
    let aspect = (swapchain.width.max(1) as f32) / (swapchain.height.max(1) as f32);
    let vfov = 2.0 * ((cam_state.hfov_rad * 0.5).tan() / aspect).atan();
    if let Projection::Perspective(ref mut p) = *proj {
        p.fov = vfov.clamp(0.1, 2.5);
        p.aspect_ratio = aspect;
    }
    cam_state.dirty = false;
}
