use std::ffi::{CString, c_char, c_void};
use std::path::PathBuf;
use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Instant;

use bevy::camera::ManualTextureViewHandle;
use bevy::prelude::*;
use bevy::render::extract_resource::{ExtractResource, ExtractResourcePlugin};
use bevy::render::render_resource::TextureView;
use bevy::render::renderer::{RenderDevice, RenderGraph, RenderGraphSystems, RenderQueue};
use bevy::render::texture::{ManualTextureView, ManualTextureViews};
use bevy::render::RenderApp;

use crate::protocol::{FrameReadyWire, MSG_FRAME_READY, SWAPCHAIN_SLOTS, encode_packet};
use crate::zenoh_ffi::ZenohPublisher;

pub const VIEWPORT_HANDLE: ManualTextureViewHandle = ManualTextureViewHandle(1);

#[cfg(target_os = "macos")]
#[repr(C)]
struct MacosSwapchainServer {
    _private: [u8; 0],
}

#[cfg(target_os = "macos")]
unsafe extern "C" {
    fn macos_swapchain_server_start(service_name: *const c_char) -> *mut MacosSwapchainServer;
    fn macos_swapchain_allocate(
        srv: *mut MacosSwapchainServer,
        raw_mtl_device: *mut c_void,
        width: u32,
        height: u32,
        generation: u32,
        out_mtl_textures: *mut *mut c_void,
    ) -> bool;
    fn macos_swapchain_read_slot_rgba(
        srv: *mut MacosSwapchainServer,
        slot: u32,
        out_rgba: *mut u8,
        out_capacity: usize,
        out_width: *mut u32,
        out_height: *mut u32,
    ) -> bool;
    fn macos_swapchain_server_stop(srv: *mut MacosSwapchainServer);
}

struct PlatformServerHandle {
    #[cfg(target_os = "macos")]
    raw: *mut MacosSwapchainServer,
}

unsafe impl Send for PlatformServerHandle {}
unsafe impl Sync for PlatformServerHandle {}

impl Drop for PlatformServerHandle {
    fn drop(&mut self) {
        #[cfg(target_os = "macos")]
        if !self.raw.is_null() {
            unsafe { macos_swapchain_server_stop(self.raw) };
            self.raw = std::ptr::null_mut();
        }
    }
}

/// Shared synchronization state between Main World, Render World, and Qt slot feedback.
pub struct SwapchainSharedState {
    platform_server: PlatformServerHandle,
    pub frame_pub: Option<ZenohPublisher>,
    pub qt_active_slot: AtomicU32,
    pub latest_ready_slot: AtomicU32,
    pub current_generation: AtomicU32,
    pub frame_counter: AtomicU64,
    pub entity_count: AtomicU32,
    pub quality_flags: AtomicU32,
    pub pending_screenshot: Mutex<Option<PathBuf>>,
    pub fps_tracker: Mutex<FpsTracker>,
    pub last_saved_screenshot_frame: AtomicU64,
}

pub struct FpsTracker {
    last_frame: Instant,
    smoothed_fps: f32,
    last_render_ms: f32,
}

impl Default for FpsTracker {
    fn default() -> Self {
        Self {
            last_frame: Instant::now(),
            smoothed_fps: 60.0,
            last_render_ms: 0.0,
        }
    }
}

impl SwapchainSharedState {
    pub fn new(service_name: &str, frame_pub: Option<ZenohPublisher>) -> Self {
        #[cfg(target_os = "macos")]
        let raw = {
            let c_name = CString::new(service_name).expect("valid C service name");
            let srv = unsafe { macos_swapchain_server_start(c_name.as_ptr()) };
            if srv.is_null() {
                warn!("Failed to start macOS Mach swapchain server on {service_name}");
            } else {
                info!("Started macOS IOSurface Mach server on '{service_name}'");
            }
            srv
        };

        Self {
            platform_server: PlatformServerHandle {
                #[cfg(target_os = "macos")]
                raw,
            },
            frame_pub,
            qt_active_slot: AtomicU32::new(u32::MAX),
            latest_ready_slot: AtomicU32::new(u32::MAX),
            current_generation: AtomicU32::new(0),
            frame_counter: AtomicU64::new(0),
            entity_count: AtomicU32::new(0),
            quality_flags: AtomicU32::new(crate::protocol::QUALITY_DEFAULT),
            pending_screenshot: Mutex::new(None),
            fps_tracker: Mutex::new(FpsTracker::default()),
            last_saved_screenshot_frame: AtomicU64::new(0),
        }
    }

    pub fn set_qt_active_slot(&self, generation: u32, active_slot: u32) {
        if generation == self.current_generation.load(Ordering::Acquire) {
            self.qt_active_slot.store(active_slot, Ordering::Release);
        }
    }

    pub fn pick_write_slot(&self) -> u32 {
        let qt_slot = self.qt_active_slot.load(Ordering::Acquire);
        let ready_slot = self.latest_ready_slot.load(Ordering::Acquire);
        for candidate in 0..(SWAPCHAIN_SLOTS as u32) {
            if candidate != qt_slot && candidate != ready_slot {
                return candidate;
            }
        }
        // Fallback: pick any slot not actively held by Qt
        for candidate in 0..(SWAPCHAIN_SLOTS as u32) {
            if candidate != qt_slot {
                return candidate;
            }
        }
        0
    }

    pub fn save_slot_png(&self, slot: u32, path: &PathBuf) -> bool {
        #[cfg(target_os = "macos")]
        {
            if self.platform_server.raw.is_null() {
                return false;
            }
            let mut w: u32 = 0;
            let mut h: u32 = 0;
            // Allocate generous buffer up to 8K RGBA; exact size verified inside C
            let cap = 4096usize * 4096usize * 4usize;
            let mut buf = vec![0u8; cap];
            let ok = unsafe {
                macos_swapchain_read_slot_rgba(
                    self.platform_server.raw,
                    slot,
                    buf.as_mut_ptr(),
                    buf.len(),
                    &mut w,
                    &mut h,
                )
            };
            if !ok || w == 0 || h == 0 {
                return false;
            }
            let used = (w as usize) * (h as usize) * 4;
            if let Some(parent) = path.parent() {
                let _ = std::fs::create_dir_all(parent);
            }
            match image::save_buffer(path, &buf[..used], w, h, image::ColorType::Rgba8) {
                Ok(()) => {
                    info!("Saved screenshot ({w}x{h}) to {}", path.display());
                    return true;
                }
                Err(e) => {
                    error!("Failed to save screenshot {}: {e}", path.display());
                    return false;
                }
            }
        }
        #[cfg(not(target_os = "macos"))]
        {
            let _ = (slot, path);
            false
        }
    }
}

struct SwapchainSlot {
    // Keep wgpu::Texture alive as long as TextureView is used
    _wgpu_texture: wgpu::Texture,
    manual_view: ManualTextureView,
}

#[derive(Resource)]
pub struct ExternalSwapchain {
    pub shared: Arc<SwapchainSharedState>,
    pub width: u32,
    pub height: u32,
    pub generation: u32,
    pub pending_resize: Option<(u32, u32)>,
    slots: Vec<SwapchainSlot>,
}

impl ExternalSwapchain {
    pub fn new(shared: Arc<SwapchainSharedState>, initial_width: u32, initial_height: u32) -> Self {
        Self {
            shared,
            width: initial_width.max(64),
            height: initial_height.max(64),
            generation: 0,
            pending_resize: Some((initial_width.max(64), initial_height.max(64))),
            slots: Vec::new(),
        }
    }

    pub fn request_resize(&mut self, width: u32, height: u32) {
        let w = width.clamp(64, 4096);
        let h = height.clamp(64, 4096);
        if w != self.width || h != self.height || self.slots.is_empty() {
            self.pending_resize = Some((w, h));
        }
    }

    pub fn reallocate_if_needed(&mut self, render_device: &RenderDevice) -> bool {
        let Some((w, h)) = self.pending_resize.take() else {
            return false;
        };

        let next_gen = self.generation + 1;
        #[cfg(target_os = "macos")]
        {
            let wgpu_dev = render_device.wgpu_device();
            let raw_dev_ptr: Option<*mut c_void> = unsafe {
                wgpu_dev.as_hal::<wgpu_hal::api::Metal>().map(|hal_dev| {
                    objc2::rc::Retained::as_ptr(hal_dev.raw_device()) as *mut c_void
                })
            };
            let Some(raw_dev_ptr) = raw_dev_ptr else {
                error!("RenderDevice is not backed by Metal!");
                return false;
            };

            let mut raw_textures: [*mut c_void; SWAPCHAIN_SLOTS] =
                [std::ptr::null_mut(); SWAPCHAIN_SLOTS];
            let ok = unsafe {
                macos_swapchain_allocate(
                    self.shared.platform_server.raw,
                    raw_dev_ptr,
                    w,
                    h,
                    next_gen,
                    raw_textures.as_mut_ptr(),
                )
            };
            if !ok {
                error!("macos_swapchain_allocate({w}x{h}, gen={next_gen}) failed");
                return false;
            }

            let mut new_slots = Vec::with_capacity(SWAPCHAIN_SLOTS);
            for (idx, &raw_tex_ptr) in raw_textures.iter().enumerate() {
                let retained_tex: objc2::rc::Retained<
                    objc2::runtime::ProtocolObject<dyn objc2_metal::MTLTexture>,
                > = unsafe {
                    objc2::rc::Retained::from_raw(raw_tex_ptr as *mut _)
                        .expect("non-null retained MTLTexture")
                };

                let format = wgpu::TextureFormat::Bgra8UnormSrgb;
                let hal_tex = unsafe {
                    wgpu_hal::metal::Device::texture_from_raw(
                        retained_tex,
                        format,
                        objc2_metal::MTLTextureType::Type2D,
                        1,
                        1,
                        wgpu_hal::CopyExtent {
                            width: w,
                            height: h,
                            depth: 1,
                        },
                    )
                };

                let desc = wgpu::TextureDescriptor {
                    label: Some(match idx {
                        0 => "gz_ext_iosurface_slot_0",
                        1 => "gz_ext_iosurface_slot_1",
                        _ => "gz_ext_iosurface_slot_2",
                    }),
                    size: wgpu::Extent3d {
                        width: w,
                        height: h,
                        depth_or_array_layers: 1,
                    },
                    mip_level_count: 1,
                    sample_count: 1,
                    dimension: wgpu::TextureDimension::D2,
                    format,
                    usage: wgpu::TextureUsages::RENDER_ATTACHMENT
                        | wgpu::TextureUsages::TEXTURE_BINDING
                        | wgpu::TextureUsages::COPY_SRC,
                    view_formats: &[],
                };

                let wgpu_tex = unsafe {
                    wgpu_dev.create_texture_from_hal::<wgpu_hal::api::Metal>(hal_tex, &desc)
                };
                let wgpu_view = wgpu_tex.create_view(&wgpu::TextureViewDescriptor::default());
                let manual_view = ManualTextureView {
                    texture_view: TextureView::from(wgpu_view),
                    size: UVec2::new(w, h),
                    view_format: format,
                };
                new_slots.push(SwapchainSlot {
                    _wgpu_texture: wgpu_tex,
                    manual_view,
                });
            }

            self.slots = new_slots;
            self.width = w;
            self.height = h;
            self.generation = next_gen;
            self.shared
                .current_generation
                .store(next_gen, Ordering::Release);
            self.shared.qt_active_slot.store(u32::MAX, Ordering::Release);
            self.shared
                .latest_ready_slot
                .store(u32::MAX, Ordering::Release);
            info!("Allocated triple-buffered IOSurface swapchain {w}x{h} (gen={next_gen})");
            true
        }

        #[cfg(not(target_os = "macos"))]
        {
            let _ = (render_device, w, h, next_gen);
            false
        }
    }

    pub fn slot_view(&self, slot: u32) -> Option<ManualTextureView> {
        self.slots
            .get(slot as usize)
            .map(|s| s.manual_view.clone())
    }
}

/// Extracted from Main World -> Render World each frame so RenderGraphSystems::Finish
/// knows which slot was rendered and publishes FrameReady after GPU completion.
#[derive(Resource, Clone, ExtractResource)]
pub struct ActiveFrameTarget {
    pub shared: Arc<SwapchainSharedState>,
    pub generation: u32,
    pub slot: u32,
    pub width: u32,
    pub height: u32,
    pub frame_id: u64,
    pub active: bool,
}

pub struct ExternalSwapchainPlugin {
    pub shared: Arc<SwapchainSharedState>,
    pub initial_width: u32,
    pub initial_height: u32,
}

impl Plugin for ExternalSwapchainPlugin {
    fn build(&self, app: &mut App) {
        let swapchain =
            ExternalSwapchain::new(self.shared.clone(), self.initial_width, self.initial_height);
        let active_frame = ActiveFrameTarget {
            shared: self.shared.clone(),
            generation: 0,
            slot: 0,
            width: self.initial_width,
            height: self.initial_height,
            frame_id: 0,
            active: false,
        };

        app.insert_resource(swapchain)
            .insert_resource(active_frame)
            .add_plugins(ExtractResourcePlugin::<ActiveFrameTarget>::default())
            .add_systems(Startup, init_swapchain_system)
            .add_systems(Last, prepare_next_swapchain_slot_system);

        if let Some(render_app) = app.get_sub_app_mut(RenderApp) {
            render_app.add_systems(
                RenderGraph,
                finish_and_publish_frame_system.in_set(RenderGraphSystems::Finish),
            );
        }
    }
}

fn init_swapchain_system(
    render_device: Res<RenderDevice>,
    mut swapchain: ResMut<ExternalSwapchain>,
    mut manual_views: ResMut<ManualTextureViews>,
    mut active_frame: ResMut<ActiveFrameTarget>,
) {
    swapchain.reallocate_if_needed(&render_device);
    if let Some(view) = swapchain.slot_view(0) {
        manual_views.insert(VIEWPORT_HANDLE, view);
        active_frame.generation = swapchain.generation;
        active_frame.slot = 0;
        active_frame.width = swapchain.width;
        active_frame.height = swapchain.height;
        active_frame.active = true;
    }
}

fn prepare_next_swapchain_slot_system(
    render_device: Res<RenderDevice>,
    mut swapchain: ResMut<ExternalSwapchain>,
    mut manual_views: ResMut<ManualTextureViews>,
    mut active_frame: ResMut<ActiveFrameTarget>,
) {
    swapchain.reallocate_if_needed(&render_device);
    let slot = swapchain.shared.pick_write_slot();
    if let Some(view) = swapchain.slot_view(slot) {
        manual_views.insert(VIEWPORT_HANDLE, view);
        let frame_id = swapchain.shared.frame_counter.fetch_add(1, Ordering::AcqRel) + 1;
        active_frame.generation = swapchain.generation;
        active_frame.slot = slot;
        active_frame.width = swapchain.width;
        active_frame.height = swapchain.height;
        active_frame.frame_id = frame_id;
        active_frame.active = true;
    } else {
        active_frame.active = false;
    }
}

fn finish_and_publish_frame_system(
    active_frame: Option<Res<ActiveFrameTarget>>,
    render_device: Res<RenderDevice>,
    render_queue: Res<RenderQueue>,
) {
    let Some(frame) = active_frame else {
        return;
    };
    if !frame.active || frame.generation == 0 {
        return;
    }

    let t0 = Instant::now();
    // Register completion barrier and wait for GPU work on this IOSurface slot to complete.
    // On Apple Silicon unified memory this ensures Qt sees 100% completed, tear-free pixels
    // without needing cross-process MTLSharedEvent plumbing yet.
    let done = Arc::new(std::sync::atomic::AtomicBool::new(false));
    let done_cb = done.clone();
    render_queue.on_submitted_work_done(move || {
        done_cb.store(true, Ordering::Release);
    });
    let _ = render_device.poll(wgpu::PollType::Wait {
        submission_index: None,
        timeout: Some(std::time::Duration::from_millis(500)),
    });
    let wait_ms = t0.elapsed().as_secs_f32() * 1000.0;

    let shared = &frame.shared;
    shared.latest_ready_slot.store(frame.slot, Ordering::Release);

    // Check pending lossless screenshot directly from the completed IOSurface!
    let screenshot_path = shared
        .pending_screenshot
        .lock()
        .ok()
        .and_then(|mut g| g.take());
    if let Some(path) = screenshot_path {
        if shared.save_slot_png(frame.slot, &path) {
            shared
                .last_saved_screenshot_frame
                .store(frame.frame_id, Ordering::Release);
        }
    }

    let (fps, render_ms) = if let Ok(mut tracker) = shared.fps_tracker.lock() {
        let now = Instant::now();
        let dt = now.duration_since(tracker.last_frame).as_secs_f32();
        tracker.last_frame = now;
        if dt > 1e-4 && dt < 2.0 {
            let inst_fps = 1.0 / dt;
            tracker.smoothed_fps = tracker.smoothed_fps * 0.9 + inst_fps * 0.1;
        }
        tracker.last_render_ms = tracker.last_render_ms * 0.8 + wait_ms * 0.2;
        (tracker.smoothed_fps, tracker.last_render_ms)
    } else {
        (60.0, wait_ms)
    };

    if let Some(ref pub_handle) = shared.frame_pub {
        let wire = FrameReadyWire {
            generation: frame.generation,
            slot: frame.slot,
            width: frame.width,
            height: frame.height,
            frame_id: frame.frame_id,
            render_ms,
            fps,
            entity_count: shared.entity_count.load(Ordering::Relaxed),
            quality_flags: shared.quality_flags.load(Ordering::Relaxed),
        };
        let pkt = encode_packet(MSG_FRAME_READY, 1, &wire);
        // Tiny 56-byte control packet: raw bytes avoids SHM fragmentation overhead
        pub_handle.publish(&pkt, false);
    }
}
