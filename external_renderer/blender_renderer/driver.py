"""Entry point executed inside Blender 5.0 (--background daemon or --show-blender UI modal)."""

from __future__ import annotations

import argparse
import signal
import sys
import time
from pathlib import Path

# Add sibling blender_renderer directory to sys.path
_THIS_DIR = Path(__file__).resolve().parent
if str(_THIS_DIR) not in sys.path:
    sys.path.insert(0, str(_THIS_DIR))

import bpy

from bridge import (
    BLENDER_CYCLES,
    BLENDER_DEFAULT,
    BLENDER_HIGH_QUALITY,
)
from renderer import BlenderRendererSession


def _parse_args() -> argparse.Namespace:
    argv = sys.argv
    if "--" in argv:
        argv = argv[argv.index("--") + 1 :]
    else:
        argv = []

    p = argparse.ArgumentParser(description="Gazebo External Renderer (Blender 5.0 Metal)")
    p.add_argument("--session", default="default")
    p.add_argument("--endpoint", default="tcp/127.0.0.1:17447")
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--engine", choices=["eevee", "cycles"], default="eevee")
    p.add_argument("--quality", choices=["balanced", "photoreal"], default="balanced")
    p.add_argument("--camera-pose", nargs=6, type=float, default=None)
    p.add_argument("--camera-hfov", type=float, default=1.0472)
    p.add_argument("--show-blender", action="store_true")
    p.add_argument("--screenshot-path", default=None)
    p.add_argument("--screenshot-after-frames", type=int, default=1)
    p.add_argument("--save-blend", default=None)
    p.add_argument("--verbose", action="store_true")
    return p.parse_args(argv)


_ACTIVE_SESSION: BlenderRendererSession | None = None


class WM_OT_gz_trigger_rerender(bpy.types.Operator):
    """Immediately rerender the current Blender scene and push it to Gazebo."""

    bl_idname = "wm.gz_trigger_rerender"
    bl_label = "Rerender Gazebo Now"
    bl_description = "Push current Blender scene and render settings to Gazebo immediately"

    def execute(self, context):
        if _ACTIVE_SESSION is not None:
            _ACTIVE_SESSION.trigger_rerender_now()
        return {"FINISHED"}


class WM_OT_gz_toggle_auto_sync(bpy.types.Operator):
    """Toggle automatic Gazebo rerender when editing the scene or render settings in Blender."""

    bl_idname = "wm.gz_toggle_auto_sync"
    bl_label = "Toggle Auto-Sync to Gazebo"
    bl_description = "Automatically rerender Gazebo whenever objects, lights, materials, or render settings change in Blender"

    def execute(self, context):
        if _ACTIVE_SESSION is not None:
            _ACTIVE_SESSION.auto_sync_blender = not _ACTIVE_SESSION.auto_sync_blender
            state = "ON" if _ACTIVE_SESSION.auto_sync_blender else "OFF"
            print(f"[gz_blender] Auto-sync Blender edits to Gazebo: {state}", flush=True)
        return {"FINISHED"}


class VIEW3D_PT_gz_external_renderer(bpy.types.Panel):
    """Sidebar panel in Blender's 3D Viewport (N -> Gazebo) for live Gazebo sync control."""

    bl_label = "Gazebo Sync"
    bl_idname = "VIEW3D_PT_gz_external_renderer"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Gazebo"

    def draw(self, context):
        layout = self.layout
        sess = _ACTIVE_SESSION
        if sess is None:
            layout.label(text="Session inactive")
            return
        scene = context.scene
        eng = "Cycles GPU" if scene.render.engine == "CYCLES" else "EEVEE Metal"
        col = layout.column(align=True)
        col.label(text=f"Session: {sess.session}")
        col.label(text=f"Engine: {eng} ({sess.last_sample_count} spp)")
        col.label(text=f"Last render: {sess.last_render_ms:.0f} ms")
        layout.separator()
        auto_label = (
            "Auto-Sync on Edit: ON" if sess.auto_sync_blender else "Auto-Sync on Edit: OFF"
        )
        layout.operator("wm.gz_toggle_auto_sync", text=auto_label, depress=sess.auto_sync_blender)
        layout.operator("wm.gz_trigger_rerender", text="Rerender Gazebo Now", icon="RENDER_STILL")


class WM_OT_gz_external_renderer(bpy.types.Operator):
    """Modal timer operator used in --show-blender debug UI mode."""

    bl_idname = "wm.gz_external_renderer"
    bl_label = "Gazebo External Renderer Sync"

    _timer = None
    _mouse_down = False

    def modal(self, context, event):
        global _ACTIVE_SESSION
        if _ACTIVE_SESSION is None or _ACTIVE_SESSION.should_exit:
            self.cancel(context)
            return {"FINISHED"}
        etype = event.type
        if etype in ("LEFTMOUSE", "MIDDLEMOUSE", "RIGHTMOUSE"):
            if event.value == "PRESS":
                self._mouse_down = True
                _ACTIVE_SESSION.note_blender_ui_interaction(True)
            elif event.value == "RELEASE":
                self._mouse_down = False
                _ACTIVE_SESSION.note_blender_ui_interaction(False)
        elif etype == "MOUSEMOVE" and self._mouse_down:
            _ACTIVE_SESSION.note_blender_ui_interaction(True)
        elif etype in ("ESC", "RET", "NUMPAD_ENTER") and event.value == "PRESS":
            self._mouse_down = False
            _ACTIVE_SESSION.note_blender_ui_interaction(False)
        elif etype == "TIMER":
            _ACTIVE_SESSION.step()
        return {"PASS_THROUGH"}

    def execute(self, context):
        wm = context.window_manager
        self._mouse_down = False
        self._timer = wm.event_timer_add(0.016, window=context.window)
        wm.modal_handler_add(self)
        return {"RUNNING_MODAL"}

    def cancel(self, context):
        global _ACTIVE_SESSION
        if self._timer is not None:
            context.window_manager.event_timer_remove(self._timer)
            self._timer = None
        if _ACTIVE_SESSION is not None:
            _ACTIVE_SESSION.close()
            _ACTIVE_SESSION = None


_UI_CLASSES = (
    WM_OT_gz_trigger_rerender,
    WM_OT_gz_toggle_auto_sync,
    VIEW3D_PT_gz_external_renderer,
    WM_OT_gz_external_renderer,
)


def main() -> None:
    global _ACTIVE_SESSION
    args = _parse_args()

    flags = BLENDER_DEFAULT
    if args.engine == "cycles" or args.quality == "photoreal":
        flags |= BLENDER_CYCLES
    if args.quality == "photoreal":
        flags |= BLENDER_HIGH_QUALITY

    cam_pose = tuple(args.camera_pose) if args.camera_pose else None

    session = BlenderRendererSession(
        session=args.session,
        endpoint=args.endpoint,
        width=args.width,
        height=args.height,
        initial_flags=flags,
        camera_pose=cam_pose,
        camera_hfov=args.camera_hfov,
        screenshot_path=args.screenshot_path,
        screenshot_after_frames=args.screenshot_after_frames,
        save_blend_path=args.save_blend,
        verbose=args.verbose,
        show_blender=args.show_blender,
    )
    _ACTIVE_SESSION = session

    def _handle_sig(_signum, _frame):
        session.should_exit = True

    signal.signal(signal.SIGINT, _handle_sig)
    signal.signal(signal.SIGTERM, _handle_sig)

    if bpy.app.background or not args.show_blender:
        # Headless daemon main loop (low-latency poll, 0% GPU when settled)
        try:
            while not session.should_exit:
                rendered = session.step()
                if not rendered and not session.should_exit:
                    time.sleep(0.008)
        finally:
            session.close()
            _ACTIVE_SESSION = None
    else:
        # Debug UI mode: register Gazebo Sync sidebar panel + modal timer in Blender's WindowManager
        for cls in _UI_CLASSES:
            bpy.utils.register_class(cls)

        def _start_modal():
            if bpy.context.window:
                bpy.ops.wm.gz_external_renderer("INVOKE_DEFAULT")
                return None
            return 0.1

        bpy.app.timers.register(_start_modal, first_interval=0.1)


if __name__ == "__main__":
    main()
