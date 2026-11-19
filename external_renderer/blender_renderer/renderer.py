"""Blender 5.0 adaptive EEVEE / Cycles Metal GPU external renderer session."""

from __future__ import annotations

import contextlib
import os
import tempfile
import time
from pathlib import Path
from typing import Optional

import bpy

from bridge import (
    BLENDER_AGX,
    BLENDER_CYCLES,
    BLENDER_DEFAULT,
    BLENDER_DENOISE,
    BLENDER_HIGH_QUALITY,
    BLENDER_IBL,
    BLENDER_RT_GI,
    BLENDER_SHADOWS,
    INPUT_KEY_DOWN,
    INPUT_MOUSE_DOWN,
    INPUT_MOUSE_MOVE,
    INPUT_MOUSE_UP,
    INPUT_QUALITY_CONFIG,
    INPUT_RESET_CAMERA,
    INPUT_RESIZE,
    INPUT_SAVE_SCENE,
    INPUT_SAVE_SCREENSHOT,
    INPUT_SET_CAMERA_POSE,
    INPUT_WHEEL,
    MSG_CLEAR_WORLD,
    MSG_DESPAWN_ENTITY,
    MSG_GUI_INPUT,
    MSG_LIGHT_UPSERT,
    MSG_MESH_DATA,
    MSG_SPAWN_VISUAL,
    MSG_TEXTURE_DATA,
    MSG_TRANSFORM_BATCH,
    MSG_WORLD_CONFIG,
    NativeBridge,
    decode_despawn,
    decode_gui_input,
    decode_light_upsert,
    decode_mesh_data,
    decode_spawn_visual,
    decode_texture_data,
    decode_transform_batch,
    decode_world_config,
    parse_header,
)
from camera_control import OrbitCameraController
from lighting import LightingManager
from scene_sync import SceneSynchronizer


@contextlib.contextmanager
def _suppress_stdout(enabled: bool):
    if not enabled:
        yield
        return
    devnull_fd = os.open(os.devnull, os.O_WRONLY)
    saved_fd = os.dup(1)
    try:
        os.dup2(devnull_fd, 1)
        yield
    finally:
        os.dup2(saved_fd, 1)
        os.close(saved_fd)
        os.close(devnull_fd)


class BlenderRendererSession:
    """Owns the Blender scene, Zenoh/IOSurface bridge, and adaptive EEVEE/Cycles scheduler."""

    STAGE_INTERACTIVE = 0
    STAGE_SETTLED = 1
    STAGE_PHOTOREAL = 2
    STAGE_DONE = 3

    def __init__(
        self,
        session: str,
        endpoint: str,
        width: int,
        height: int,
        initial_flags: int = BLENDER_DEFAULT,
        camera_pose: Optional[tuple[float, float, float, float, float, float]] = None,
        camera_hfov: float = 1.0472,
        screenshot_path: Optional[str] = None,
        screenshot_after_frames: int = 1,
        save_blend_path: Optional[str] = None,
        verbose: bool = False,
        show_blender: bool = False,
    ) -> None:
        self.session = session
        self.endpoint = endpoint
        self.quality_flags = initial_flags
        self.verbose = verbose
        self.show_blender = show_blender

        self.screenshot_path = screenshot_path
        self.screenshot_after_frames = max(1, screenshot_after_frames)
        self.save_blend_path = save_blend_path
        self.should_exit = False

        self.screenshot_seq = 0
        self.blend_seq = 0
        self.work_dir = Path.cwd()
        self.tga_path = os.path.join(tempfile.gettempdir(), f"gz_blender_frame_{os.getpid()}.tga")

        # 1. Clear default startup scene objects (Cube, Light, Camera) cleanly
        scene = bpy.context.scene
        for obj in list(bpy.data.objects):
            bpy.data.objects.remove(obj, do_unlink=True)
        for col in list(bpy.data.collections):
            if col != scene.collection:
                bpy.data.collections.remove(col)

        # Dedicated collection for Gazebo entities & lights
        self.gz_collection = bpy.data.collections.new("GazeboScene")
        scene.collection.children.link(self.gz_collection)
        # Keep Blender's active collection on the root Scene Collection so user-added
        # objects in --show-blender mode stay separate from Gazebo ClearWorld snapshots.
        try:
            bpy.context.view_layer.active_layer_collection = (
                bpy.context.view_layer.layer_collection
            )
        except Exception:
            pass

        # Create viewport camera
        cam_data = bpy.data.cameras.new("GzViewportCamera")
        self.cam_obj = bpy.data.objects.new("GzViewportCamera", cam_data)
        scene.collection.objects.link(self.cam_obj)
        scene.camera = self.cam_obj

        self.camera = OrbitCameraController()
        if camera_pose is not None:
            self.camera.set_from_euler6(*camera_pose, camera_hfov)
        self.camera.apply_to_blender_camera(self.cam_obj)

        self.lighting = LightingManager(self.gz_collection)
        self.sync = SceneSynchronizer(self.gz_collection)

        # Blender-side live edit watcher state (triggers Gazebo rerender on Blender edits)
        self.auto_sync_blender = True
        self.blender_edit_debounce_s = 0.45 if show_blender else 0.05
        self._suppress_watch_depth = 0
        self._blender_edit_pending = False
        self._last_blender_edit_time = 0.0
        self._last_blender_edit_reason = ""
        self._blender_interaction_until = 0.0
        self._last_settings_poll = 0.0
        self._settings_fingerprint = None
        self._debug_viewport_initialized = False
        self._user_cycles_samples: Optional[int] = None
        self._user_eevee_samples: Optional[int] = None
        self._user_res_pct: int = 100
        self._handlers_registered = False

        # 2. Configure Blender 5.0 Metal GPU render settings
        with self._suppress_watch():
            self._init_render_settings(scene, width, height)
            self._apply_engine_flags(scene)
            self._settings_fingerprint = self._capture_settings_fingerprint(scene)

        self._register_blender_handlers()

        # 3. Open native Zenoh SHM + Mach IOSurface triple-buffered bridge
        self.bridge = NativeBridge(session, endpoint, width, height)
        self.client_id = os.getpid()

        self.received_snapshot = False
        self.last_snapshot_req = 0.0
        self.snapshot_frames_rendered = 0
        self.total_frames = 0

        # Adaptive refinement state
        self.stage = self.STAGE_INTERACTIVE
        self.last_change_time = time.monotonic()
        self.last_frame_time = time.monotonic()
        self.last_heartbeat_time = time.monotonic()
        self.last_render_ms = 0.0
        self.last_sample_count = 0
        self.fps_ema = 0.0
        self.settle_delay_s = 0.55 if screenshot_path else 0.22
        self._world_dirty = True
        self._engine_dirty = False
        self._need_initial_frame = True

        # Ask Gazebo server for an immediate snapshot
        self.bridge.request_snapshot(self.client_id)
        self.last_snapshot_req = time.monotonic()

        print(
            f"[gz_blender] Session '{session}' ready ({width}x{height}, "
            f"engine={'Cycles Metal' if (self.quality_flags & BLENDER_CYCLES) else 'EEVEE Metal'}, "
            f"flags=0x{self.quality_flags:02x})",
            flush=True,
        )

    @contextlib.contextmanager
    def _suppress_watch(self):
        self._suppress_watch_depth += 1
        try:
            yield
        finally:
            self._suppress_watch_depth -= 1

    def _on_depsgraph_update(self, _scene, depsgraph) -> None:
        if self._suppress_watch_depth > 0 or not self.auto_sync_blender:
            return
        if not depsgraph or not depsgraph.updates:
            return
        # Ignore bare Scene selection/active-object clicks (no geometry/transform/shading change)
        changed_ids = []
        for u in depsgraph.updates:
            id_type = u.id.bl_rna.identifier
            if (
                id_type == "Scene"
                and not u.is_updated_geometry
                and not u.is_updated_transform
                and not u.is_updated_shading
            ):
                continue
            changed_ids.append(f"{id_type}:{u.id.name}")
        if changed_ids:
            self.notify_blender_edit(changed_ids[0])

    def _on_undo_redo(self, _scene) -> None:
        if self._suppress_watch_depth > 0 or not self.auto_sync_blender:
            return
        self.notify_blender_edit("undo/redo")

    def _register_blender_handlers(self) -> None:
        if self._handlers_registered:
            return
        bpy.app.handlers.depsgraph_update_post.append(self._on_depsgraph_update)
        bpy.app.handlers.undo_post.append(self._on_undo_redo)
        bpy.app.handlers.redo_post.append(self._on_undo_redo)
        self._handlers_registered = True

    def _unregister_blender_handlers(self) -> None:
        if not self._handlers_registered:
            return
        for lst, fn in (
            (bpy.app.handlers.depsgraph_update_post, self._on_depsgraph_update),
            (bpy.app.handlers.undo_post, self._on_undo_redo),
            (bpy.app.handlers.redo_post, self._on_undo_redo),
        ):
            try:
                lst.remove(fn)
            except ValueError:
                pass
        self._handlers_registered = False

    def notify_blender_edit(self, reason: str = "scene edit") -> None:
        self._blender_edit_pending = True
        self._last_blender_edit_time = time.monotonic()
        self._last_blender_edit_reason = reason

    def note_blender_ui_interaction(self, active: bool) -> None:
        now = time.monotonic()
        if active:
            self._blender_interaction_until = now + 0.75
        else:
            self._blender_interaction_until = 0.0
            if self._blender_edit_pending:
                # Start debounce cleanly from mouse release so rerender kicks in promptly
                self._last_blender_edit_time = now - max(0.0, self.blender_edit_debounce_s - 0.15)

    def trigger_rerender_now(self) -> None:
        scene = bpy.context.scene
        self._sync_from_blender_scene(scene)
        self._settings_fingerprint = self._capture_settings_fingerprint(scene)
        self._blender_edit_pending = False
        self._blender_interaction_until = 0.0
        self.stage = self.STAGE_SETTLED
        self.last_change_time = time.monotonic()
        print("[gz_blender] Manual rerender triggered from Blender UI", flush=True)

    def _capture_settings_fingerprint(self, scene) -> tuple:
        r = scene.render
        vs = scene.view_settings
        c = scene.cycles
        e = scene.eevee
        cam = scene.camera
        cam_sig = None
        if cam is not None and cam.data is not None:
            cd = cam.data
            dof = cd.dof
            cam_sig = (
                cam.name,
                round(cd.lens, 3),
                round(cd.clip_start, 4),
                round(cd.clip_end, 2),
                bool(dof.use_dof),
                round(dof.focus_distance, 4),
                round(dof.aperture_fstop, 3),
            )
        world_sig = None
        if scene.world is not None:
            w = scene.world
            bg_vals = []
            wnt = getattr(w, "node_tree", None)
            if wnt is not None:
                for n in wnt.nodes:
                    if n.type == "BACKGROUND":
                        col = n.inputs["Color"].default_value
                        bg_vals.append(
                            (
                                n.name,
                                round(col[0], 3),
                                round(col[1], 3),
                                round(col[2], 3),
                                round(n.inputs["Strength"].default_value, 4),
                            )
                        )
            world_sig = (w.name, tuple(bg_vals))
        vis_sig = tuple((o.name, bool(o.hide_render)) for o in bpy.data.objects)
        return (
            r.engine,
            bool(r.film_transparent),
            bool(r.use_compositing),
            int(r.resolution_percentage),
            vs.view_transform,
            vs.look,
            round(vs.exposure, 4),
            round(vs.gamma, 4),
            bool(vs.use_curve_mapping),
            c.device,
            int(c.samples),
            bool(c.use_adaptive_sampling),
            round(c.adaptive_threshold, 5),
            bool(c.use_denoising),
            c.denoiser,
            c.denoising_input_passes,
            bool(c.denoising_use_gpu),
            int(c.max_bounces),
            int(c.diffuse_bounces),
            int(c.glossy_bounces),
            int(c.transmission_bounces),
            int(c.transparent_max_bounces),
            int(c.volume_bounces),
            round(c.sample_clamp_direct, 3),
            round(c.sample_clamp_indirect, 3),
            bool(c.caustics_reflective),
            bool(c.caustics_refractive),
            bool(c.use_light_tree),
            bool(c.use_fast_gi),
            int(c.ao_bounces_render),
            round(c.film_exposure, 4),
            int(e.taa_render_samples),
            bool(e.use_raytracing),
            int(e.gi_diffuse_bounces),
            bool(e.use_shadows),
            round(e.shadow_resolution_scale, 3),
            e.shadow_pool_size,
            bool(e.use_taa_reprojection),
            cam_sig,
            world_sig,
            vis_sig,
        )

    def _sync_from_blender_scene(self, scene) -> None:
        """Mirror user edits from Blender's Render/Color properties back into quality_flags & targets."""
        flags = self.quality_flags
        is_cycles = scene.render.engine == "CYCLES"
        if is_cycles:
            flags |= BLENDER_CYCLES
        else:
            flags &= ~BLENDER_CYCLES

        if scene.cycles.use_denoising:
            flags |= BLENDER_DENOISE
        else:
            flags &= ~BLENDER_DENOISE

        if scene.view_settings.view_transform == "AgX":
            flags |= BLENDER_AGX
        else:
            flags &= ~BLENDER_AGX

        if is_cycles:
            if scene.cycles.diffuse_bounces >= 2:
                flags |= BLENDER_RT_GI
            else:
                flags &= ~BLENDER_RT_GI
        else:
            if scene.eevee.use_raytracing:
                flags |= BLENDER_RT_GI
            else:
                flags &= ~BLENDER_RT_GI
            if scene.eevee.use_shadows:
                flags |= BLENDER_SHADOWS
            else:
                flags &= ~BLENDER_SHADOWS

        self.quality_flags = flags
        self._user_cycles_samples = max(1, int(scene.cycles.samples))
        self._user_eevee_samples = max(1, int(scene.eevee.taa_render_samples))
        self._user_res_pct = max(10, min(200, int(scene.render.resolution_percentage)))

    def _check_blender_settings_changed(self, scene) -> None:
        if not self.auto_sync_blender or self._suppress_watch_depth > 0:
            return
        now = time.monotonic()
        if (now - self._last_settings_poll) < 0.10:
            return
        self._last_settings_poll = now
        fp = self._capture_settings_fingerprint(scene)
        if self._settings_fingerprint is not None and fp != self._settings_fingerprint:
            self._settings_fingerprint = fp
            self._sync_from_blender_scene(scene)
            reason = (
                self._last_blender_edit_reason
                if self._blender_edit_pending and self._last_blender_edit_reason
                else "render settings"
            )
            self.notify_blender_edit(reason)
        elif self._settings_fingerprint is None:
            self._settings_fingerprint = fp

    def _init_render_settings(self, scene, width: int, height: int) -> None:
        scene.render.resolution_x = max(64, width)
        scene.render.resolution_y = max(64, height)
        scene.render.resolution_percentage = 100
        scene.render.pixel_aspect_x = 1.0
        scene.render.pixel_aspect_y = 1.0
        scene.render.use_persistent_data = True
        scene.render.use_compositing = False
        scene.render.use_sequencer = False
        scene.render.use_file_extension = False
        scene.render.image_settings.file_format = "TARGA_RAW"
        scene.render.image_settings.color_mode = "RGBA"
        scene.render.film_transparent = False

        # Enable Apple Silicon Metal GPU device for Cycles (disable CPU to avoid slow tile tail)
        try:
            prefs = bpy.context.preferences.addons["cycles"].preferences
            prefs.compute_device_type = "METAL"
            prefs.get_devices()
            found_metal = False
            for d in prefs.devices:
                if d.type == "METAL":
                    d.use = True
                    found_metal = True
                else:
                    d.use = False
            scene.cycles.device = "GPU" if found_metal else "CPU"
            print(
                f"[gz_blender] Cycles compute device: {scene.cycles.device} "
                f"(devices={[(d.name, d.type, d.use) for d in prefs.devices]})",
                flush=True,
            )
        except Exception as exc:
            print(f"[gz_blender] Warning configuring Cycles Metal GPU: {exc}", flush=True)

        # Cycles photorealism settings
        scene.cycles.use_adaptive_sampling = True
        scene.cycles.adaptive_threshold = 0.02
        scene.cycles.denoiser = "OPENIMAGEDENOISE"
        scene.cycles.denoising_input_passes = "RGB_ALBEDO_NORMAL"
        scene.cycles.denoising_use_gpu = True
        scene.cycles.sample_clamp_indirect = 10.0

        # EEVEE 5.0 photorealism settings
        scene.eevee.shadow_pool_size = "1024"
        scene.eevee.shadow_resolution_scale = 1.0
        scene.eevee.use_taa_reprojection = True
        scene.eevee.gi_diffuse_bounces = 2

        self.lighting.apply_color_management(scene, self.quality_flags)
        self.lighting.apply_world(scene, self.quality_flags)

    def _mark_dirty(self) -> None:
        self.stage = self.STAGE_INTERACTIVE
        self.last_change_time = time.monotonic()

    def _target_samples(self) -> tuple[int, int]:
        """Return canonical target (eevee_samples, cycles_samples), respecting Blender UI overrides."""
        high_q = bool(self.quality_flags & BLENDER_HIGH_QUALITY)
        eevee_target = (
            self._user_eevee_samples
            if self._user_eevee_samples is not None
            else (32 if high_q else 16)
        )
        cycles_target = (
            self._user_cycles_samples
            if self._user_cycles_samples is not None
            else (128 if high_q else 64)
        )
        return max(1, eevee_target), max(1, cycles_target)

    def _apply_engine_flags(self, scene) -> None:
        """Push Gazebo HUD quality_flags into Blender scene (only when changed via Gazebo HUD)."""
        flags = self.quality_flags
        use_cycles = bool(flags & BLENDER_CYCLES)
        rt_gi = bool(flags & BLENDER_RT_GI)
        denoise = bool(flags & BLENDER_DENOISE)
        shadows = bool(flags & BLENDER_SHADOWS)

        self.lighting.apply_color_management(scene, flags)
        self.lighting.apply_shadow_flag(flags)

        scene.render.engine = "CYCLES" if use_cycles else "BLENDER_EEVEE"
        scene.eevee.use_raytracing = rt_gi
        scene.eevee.use_shadows = shadows
        scene.eevee.shadow_pool_size = "1024"

        scene.cycles.use_denoising = denoise
        if rt_gi:
            scene.cycles.max_bounces = 8
            scene.cycles.diffuse_bounces = 4
            scene.cycles.glossy_bounces = 4
            scene.cycles.transmission_bounces = 8
            scene.cycles.transparent_max_bounces = 8
        else:
            scene.cycles.max_bounces = 2
            scene.cycles.diffuse_bounces = 1
            scene.cycles.glossy_bounces = 1
            scene.cycles.transmission_bounces = 2
            scene.cycles.transparent_max_bounces = 4

        eevee_target, cycles_target = self._target_samples()
        scene.eevee.taa_render_samples = eevee_target
        scene.cycles.samples = cycles_target
        scene.render.resolution_percentage = self._user_res_pct

    def _apply_engine_and_stage(self, scene, stage: int) -> tuple[int, bool]:
        """Configure per-stage samples/resolution without clobbering user Blender render edits.
        Returns (sample_count, is_still_refining).
        """
        flags = self.quality_flags

        if self._engine_dirty:
            self._apply_engine_flags(scene)
            self._engine_dirty = False

        if self._world_dirty:
            self.lighting.apply_world(scene, flags)
            self._world_dirty = False

        use_cycles = scene.render.engine == "CYCLES"
        eevee_target, cycles_target = self._target_samples()
        base_pct = self._user_res_pct

        if not use_cycles:
            if stage == self.STAGE_INTERACTIVE and eevee_target > 4:
                scene.render.resolution_percentage = max(25, int(round(base_pct * 0.75)))
                samples = min(4, eevee_target)
                scene.eevee.taa_render_samples = samples
                return samples, True
            else:
                scene.render.resolution_percentage = base_pct
                scene.eevee.taa_render_samples = eevee_target
                return eevee_target, False

        # Cycles Metal GPU path tracing
        if stage == self.STAGE_INTERACTIVE and cycles_target > 2:
            scene.render.resolution_percentage = max(25, int(round(base_pct * 0.55)))
            samples = min(2, cycles_target)
            scene.cycles.samples = samples
            return samples, True
        elif stage == self.STAGE_SETTLED and cycles_target > 16:
            scene.render.resolution_percentage = base_pct
            samples = min(16, cycles_target)
            scene.cycles.samples = samples
            return samples, True
        else:
            scene.render.resolution_percentage = base_pct
            scene.cycles.samples = cycles_target
            return cycles_target, False

    def _restore_canonical_render_targets(self, scene) -> None:
        """Restore canonical target samples & resolution after an interactive pass so
        Blender's UI always displays the user's true target settings (never flickers 2 spp / 55%)."""
        eevee_target, cycles_target = self._target_samples()
        scene.eevee.taa_render_samples = eevee_target
        scene.cycles.samples = cycles_target
        scene.render.resolution_percentage = self._user_res_pct

    def _drain_scene_events(self) -> bool:
        """Drain all queued Gazebo scene packets (coalescing transforms). Returns True if dirty."""
        now = time.monotonic()
        if not self.received_snapshot and (now - self.last_snapshot_req) >= 1.5:
            self.bridge.request_snapshot(self.client_id)
            self.last_snapshot_req = now

        changed = False
        latest_transforms = None
        new_lights = False

        while True:
            item = self.bridge.pop_scene()
            if item is None:
                break
            raw, is_shm = item
            if is_shm:
                self.sync.shm_msgs += 1
            else:
                self.sync.raw_msgs += 1

            parsed = parse_header(raw)
            if parsed is None:
                continue
            msg_type, count, payload = parsed

            if msg_type == MSG_CLEAR_WORLD:
                self.lighting.clear_lights()
                self.sync.clear_world()
                changed = True
                self._world_dirty = True
                print("[gz_blender] Cleared scene for fresh Gazebo snapshot", flush=True)

            elif msg_type == MSG_WORLD_CONFIG:
                cfg = decode_world_config(payload)
                if cfg is not None:
                    self.received_snapshot = True
                    self.lighting.world_cfg = cfg
                    self._world_dirty = True
                    if cfg.has_camera_pose:
                        self.camera.set_from_gazebo_pose(cfg.camera_pose, cfg.camera_hfov)
                    changed = True
                    print(
                        f"[gz_blender] Received WorldConfig (sky={cfg.has_sky}, "
                        f"ambient={[round(c, 2) for c in cfg.ambient[:3]]}, "
                        f"bg={[round(c, 2) for c in cfg.background[:3]]}, "
                        f"shm_msgs={self.sync.shm_msgs}, raw_msgs={self.sync.raw_msgs})",
                        flush=True,
                    )

            elif msg_type == MSG_TEXTURE_DATA:
                td = decode_texture_data(payload)
                if td is not None:
                    self.sync.ingest_texture(td)

            elif msg_type == MSG_MESH_DATA:
                md = decode_mesh_data(payload)
                if md is not None:
                    self.sync.ingest_mesh(md)

            elif msg_type == MSG_SPAWN_VISUAL:
                sv = decode_spawn_visual(payload)
                if sv is not None:
                    self.received_snapshot = True
                    self.sync.spawn_visual(sv)
                    changed = True

            elif msg_type == MSG_DESPAWN_ENTITY:
                eid = decode_despawn(payload)
                if eid is not None:
                    if self.sync.despawn_entity(eid) or self.lighting.remove_light(eid):
                        changed = True

            elif msg_type == MSG_LIGHT_UPSERT:
                lw = decode_light_upsert(payload)
                if lw is not None:
                    self.received_snapshot = True
                    self.lighting.upsert_light(lw, self.quality_flags)
                    new_lights = True
                    changed = True

            elif msg_type == MSG_TRANSFORM_BATCH:
                _, items = decode_transform_batch(payload, count)
                if items:
                    latest_transforms = items

        if new_lights:
            # Refresh world sky/indoor HDRI heuristics once lights arrive
            self._world_dirty = True

        if latest_transforms:
            if self.sync.apply_transforms(latest_transforms):
                changed = True

        return changed

    def _drain_gui_inputs(self) -> bool:
        """Drain all queued Qt GUI inputs (coalescing mouse moves/resizes). Returns True if dirty."""
        changed = False
        pending_resize: Optional[tuple[int, int]] = None

        while True:
            raw = self.bridge.pop_input()
            if raw is None:
                break
            parsed = parse_header(raw)
            if parsed is None:
                continue
            msg_type, _, payload = parsed
            if msg_type != MSG_GUI_INPUT:
                continue
            ev = decode_gui_input(payload)
            if ev is None:
                continue

            if ev.kind == INPUT_MOUSE_DOWN:
                self.camera.on_mouse_down(ev.x, ev.y, ev.button, ev.modifiers)
            elif ev.kind == INPUT_MOUSE_UP:
                self.camera.on_mouse_up(ev.button)
            elif ev.kind == INPUT_MOUSE_MOVE:
                if self.camera.on_mouse_move(ev.x, ev.y, ev.dx, ev.dy):
                    changed = True
            elif ev.kind == INPUT_WHEEL:
                if self.camera.on_wheel(ev.dy):
                    changed = True
            elif ev.kind == INPUT_KEY_DOWN:
                ch = ev.key_code & 0xFF
                if ch in (ord("R"), ord("r")):
                    self.camera.reset()
                    changed = True
                elif ch == ord("1"):
                    self.quality_flags ^= BLENDER_CYCLES
                    self._engine_dirty = True
                    changed = True
                elif ch == ord("2"):
                    self.quality_flags ^= BLENDER_IBL
                    self._world_dirty = True
                    changed = True
                elif ch == ord("3"):
                    self.quality_flags ^= BLENDER_RT_GI
                    self._engine_dirty = True
                    changed = True
                elif ch == ord("4"):
                    self.quality_flags ^= BLENDER_DENOISE
                    self._engine_dirty = True
                    changed = True
                elif ch == ord("5"):
                    self.quality_flags ^= BLENDER_AGX
                    self._engine_dirty = True
                    changed = True
                elif ch in (ord("P"), ord("p")):
                    self._save_interactive_screenshot()
                elif ch in (ord("B"), ord("b")):
                    self._save_blend_file()
            elif ev.kind == INPUT_RESIZE:
                dpr = max(1.0, min(2.0, ev.dpr))
                pw = float(ev.width) * dpr
                ph = float(ev.height) * dpr
                max_dim = 1920.0
                cur_max = max(pw, ph)
                if cur_max > max_dim:
                    s = max_dim / cur_max
                    pw *= s
                    ph *= s
                pending_resize = (max(64, int(round(pw))), max(64, int(round(ph))))
            elif ev.kind == INPUT_SET_CAMERA_POSE:
                self.camera.set_from_gazebo_pose(ev.camera_pose, ev.camera_hfov)
                changed = True
            elif ev.kind == INPUT_QUALITY_CONFIG:
                old_flags = self.quality_flags
                self.quality_flags = ev.quality_flags
                if (old_flags ^ self.quality_flags) & BLENDER_IBL:
                    self._world_dirty = True
                if (old_flags ^ self.quality_flags) & BLENDER_HIGH_QUALITY:
                    # Explicit Gazebo preset toggle resets custom Blender sample override
                    self._user_cycles_samples = None
                    self._user_eevee_samples = None
                self._engine_dirty = True
                changed = True
                print(
                    f"[gz_blender] Applied quality flags=0x{self.quality_flags:02x} "
                    f"(engine={'Cycles' if (self.quality_flags & BLENDER_CYCLES) else 'EEVEE'}, "
                    f"IBL={bool(self.quality_flags & BLENDER_IBL)}, "
                    f"RT_GI={bool(self.quality_flags & BLENDER_RT_GI)}, "
                    f"Denoise={bool(self.quality_flags & BLENDER_DENOISE)}, "
                    f"AgX={bool(self.quality_flags & BLENDER_AGX)}, "
                    f"Shadows={bool(self.quality_flags & BLENDER_SHADOWS)})",
                    flush=True,
                )
            elif ev.kind == INPUT_SAVE_SCREENSHOT:
                self._save_interactive_screenshot(ev.path or None)
            elif ev.kind == INPUT_SAVE_SCENE:
                self._save_blend_file(ev.path or None)
            elif ev.kind == INPUT_RESET_CAMERA:
                self.camera.reset()
                changed = True

        if pending_resize is not None:
            rw, rh = pending_resize
            if rw != self.bridge.width or rh != self.bridge.height:
                if self.bridge.resize(rw, rh):
                    scene = bpy.context.scene
                    scene.render.resolution_x = rw
                    scene.render.resolution_y = rh
                    changed = True

        return changed

    def _save_interactive_screenshot(self, custom_path: Optional[str] = None) -> None:
        if custom_path:
            out = Path(custom_path)
        else:
            self.screenshot_seq += 1
            out = self.work_dir / f"blender_capture_{self.screenshot_seq:04d}.png"
        if self.bridge.save_png(str(out)):
            print(f"[gz_blender] Saved screenshot to {out}", flush=True)

    def _save_blend_file(self, custom_path: Optional[str] = None) -> None:
        if custom_path:
            out = Path(custom_path)
        else:
            out = self.work_dir / "gz_blender_scene.blend"
        try:
            bpy.ops.wm.save_as_mainfile(filepath=str(out), copy=True)
            print(f"[gz_blender] Saved live Blender scene to {out}", flush=True)
        except Exception as exc:
            print(f"[gz_blender] Failed to save .blend '{out}': {exc}", flush=True)

    def _sync_debug_viewport(self) -> None:
        if not self.show_blender or self._debug_viewport_initialized:
            return
        wm = bpy.context.window_manager
        for win in wm.windows:
            for area in win.screen.areas:
                if area.type == "VIEW_3D":
                    space = area.spaces.active
                    if space and space.region_3d:
                        space.region_3d.view_perspective = "CAMERA"
                        if space.shading.type == "SOLID":
                            space.shading.type = "RENDERED"
                        self._debug_viewport_initialized = True

    def step(self) -> bool:
        """Run one poll + optional adaptive render tick. Returns True if a frame was rendered."""
        scene = bpy.context.scene
        with self._suppress_watch():
            dirty = self._drain_scene_events()
            dirty = self._drain_gui_inputs() or dirty

            if self.camera.dirty:
                self.camera.apply_to_blender_camera(self.cam_obj)
                dirty = True

            if dirty:
                self._settings_fingerprint = self._capture_settings_fingerprint(scene)

        # Detect live user edits made directly inside Blender's UI (objects, lights, materials,
        # render/color settings) and debounce so we never stall mid-drag.
        self._check_blender_settings_changed(scene)

        now = time.monotonic()
        ui_busy = self.show_blender and (now < self._blender_interaction_until)
        if (
            self._blender_edit_pending
            and not ui_busy
            and (now - self._last_blender_edit_time) >= self.blender_edit_debounce_s
        ):
            reason = self._last_blender_edit_reason or "scene edit"
            self._blender_edit_pending = False
            self._sync_from_blender_scene(scene)
            self._settings_fingerprint = self._capture_settings_fingerprint(scene)
            dirty = True
            print(
                f"[gz_blender] Blender edit detected ({reason}) -> rerendering for Gazebo",
                flush=True,
            )

        if dirty or self._need_initial_frame:
            self._need_initial_frame = False
            self._mark_dirty()

        # Pause blocking renders while user is actively dragging inside Blender's UI
        if ui_busy:
            return False

        # Decide whether to render now based on adaptive stage state machine
        should_render = False
        stage_to_render = self.stage

        if self.stage == self.STAGE_INTERACTIVE:
            # Render immediately on dirty / drag / moving objects
            if dirty or self.total_frames == 0:
                should_render = True
                stage_to_render = self.STAGE_INTERACTIVE
            elif (now - self.last_change_time) >= self.settle_delay_s and not (
                self.camera.dragging_orbit or self.camera.dragging_pan
            ):
                # Settled! Advance directly to full-resolution settled pass
                self.stage = self.STAGE_SETTLED
                stage_to_render = self.STAGE_SETTLED
                should_render = True
        elif self.stage == self.STAGE_SETTLED:
            should_render = True
            stage_to_render = self.STAGE_SETTLED
        elif self.stage == self.STAGE_PHOTOREAL:
            should_render = True
            stage_to_render = self.STAGE_PHOTOREAL
        else:
            # STAGE_DONE: sleep quietly at 0% GPU, sending a 2Hz FrameReady heartbeat so
            # late-joining Qt GUIs immediately attach and display the latest IOSurface slot!
            if self.total_frames > 0 and (now - self.last_heartbeat_time) >= 0.5:
                self.bridge.reannounce_frame(
                    render_ms=self.last_render_ms,
                    fps=self.fps_ema,
                    entity_count=self.sync.entity_count,
                    quality_flags=self.quality_flags,
                    refining=False,
                    sample_count=self.last_sample_count,
                )
                self.last_heartbeat_time = now
            return False

        if not should_render:
            return False

        # In headless automated screenshot mode, wait briefly for the entire Gazebo snapshot
        # (all visuals, lights, streamed textures/meshes) to finish arriving before final capture.
        # Also handle continuously animated worlds by advancing after a few snapshot frames.
        if self.screenshot_path and self.received_snapshot and stage_to_render == self.STAGE_INTERACTIVE:
            settled_or_animated = (not dirty and (now - self.last_change_time) >= 0.55) or (
                self.snapshot_frames_rendered >= max(2, self.screenshot_after_frames)
            )
            if settled_or_animated:
                stage_to_render = (
                    self.STAGE_PHOTOREAL
                    if (scene.render.engine == "CYCLES")
                    else self.STAGE_SETTLED
                )
                self.stage = stage_to_render

        with self._suppress_watch():
            sample_count, refining = self._apply_engine_and_stage(scene, stage_to_render)
            render_pct = scene.render.resolution_percentage

            # Ensure TARGA_RAW RGBA output format for zero-copy IOSurface blit without overwriting
            # other user render settings.
            scene.render.use_file_extension = False
            scene.render.image_settings.file_format = "TARGA_RAW"
            scene.render.image_settings.color_mode = "RGBA"

            t0 = time.perf_counter()
            with _suppress_stdout(not self.verbose):
                bpy.ops.render.render(write_still=False)
                bpy.data.images["Render Result"].save_render(self.tga_path)
            t1 = time.perf_counter()
            render_ms = (t1 - t0) * 1000.0

            self._restore_canonical_render_targets(scene)
            self._settings_fingerprint = self._capture_settings_fingerprint(scene)

        dt = max(1e-4, time.monotonic() - self.last_frame_time)
        self.last_frame_time = time.monotonic()
        self.last_heartbeat_time = self.last_frame_time
        self.last_render_ms = render_ms
        self.last_sample_count = sample_count
        inst_fps = 1000.0 / max(render_ms, 1.0)
        self.fps_ema = inst_fps if self.fps_ema <= 0.0 else (self.fps_ema * 0.7 + inst_fps * 0.3)

        self.bridge.present_tga_file(
            self.tga_path,
            render_ms=render_ms,
            fps=self.fps_ema,
            entity_count=self.sync.entity_count,
            quality_flags=self.quality_flags,
            refining=refining,
            sample_count=sample_count,
        )

        self.total_frames += 1
        if self.received_snapshot:
            self.snapshot_frames_rendered += 1

        self._sync_debug_viewport()

        # Advance refinement state machine (unless new inputs already queued mid-render!)
        if self.bridge.has_pending() or self._blender_edit_pending:
            self.stage = self.STAGE_INTERACTIVE
            self.last_change_time = time.monotonic()
        else:
            use_cycles = scene.render.engine == "CYCLES"
            if stage_to_render == self.STAGE_INTERACTIVE:
                self.stage = self.STAGE_INTERACTIVE if refining else self.STAGE_DONE
            elif stage_to_render == self.STAGE_SETTLED:
                self.stage = (
                    self.STAGE_PHOTOREAL if (use_cycles and refining) else self.STAGE_DONE
                )
            else:
                self.stage = self.STAGE_DONE

        if self.verbose or stage_to_render in (self.STAGE_SETTLED, self.STAGE_PHOTOREAL):
            eng = "Cycles" if (scene.render.engine == "CYCLES") else "EEVEE"
            print(
                f"[gz_blender] Frame #{self.total_frames} ({eng} {sample_count} spp, "
                f"scale={render_pct}%, {render_ms:.1f} ms, "
                f"vis={self.sync.entity_count}, stage={stage_to_render})",
                flush=True,
            )

        # Automated E2E capture exit condition
        if (
            self.screenshot_path
            and self.received_snapshot
            and not refining
            and self.snapshot_frames_rendered >= self.screenshot_after_frames
        ):
            self.bridge.save_png(self.screenshot_path)
            print(
                f"[gz_blender] Saved auto-screenshot ({self.bridge.width}x{self.bridge.height}) "
                f"to {self.screenshot_path}",
                flush=True,
            )
            if self.save_blend_path:
                self._save_blend_file(self.save_blend_path)
            self.should_exit = True

        return True

    def close(self) -> None:
        self._unregister_blender_handlers()
        self.bridge.close()
        try:
            if os.path.exists(self.tga_path):
                os.unlink(self.tga_path)
        except OSError:
            pass
