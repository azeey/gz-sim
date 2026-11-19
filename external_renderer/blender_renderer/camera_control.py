"""Z-up orbit/pan/zoom camera controller for Blender matching Gazebo's camera conventions."""

from __future__ import annotations

import math
from mathutils import Euler, Matrix, Quaternion, Vector

from bridge import Pose


class OrbitCameraController:
    """Native Blender Z-up orbit camera controller matching Gazebo mouse interactions."""

    def __init__(self) -> None:
        self.focus = Vector((0.0, 0.0, 0.5))
        self.yaw = -0.85  # Azimuth around +Z (radians)
        self.pitch = 0.42  # Elevation above XY plane (radians)
        self.distance = 6.0
        self.hfov_rad = 1.0472  # 60 deg

        self.initial_focus = self.focus.copy()
        self.initial_yaw = self.yaw
        self.initial_pitch = self.pitch
        self.initial_distance = self.distance
        self.initial_hfov_rad = self.hfov_rad

        self.dragging_orbit = False
        self.dragging_pan = False
        self.last_mouse = (0.0, 0.0)
        self.dirty = True

    def save_initial(self) -> None:
        self.initial_focus = self.focus.copy()
        self.initial_yaw = self.yaw
        self.initial_pitch = self.pitch
        self.initial_distance = self.distance
        self.initial_hfov_rad = self.hfov_rad

    def reset(self) -> None:
        self.focus = self.initial_focus.copy()
        self.yaw = self.initial_yaw
        self.pitch = self.initial_pitch
        self.distance = self.initial_distance
        self.hfov_rad = self.initial_hfov_rad
        self.dirty = True

    def set_from_gazebo_pose(self, pose: Pose, hfov_rad: float) -> None:
        """Convert Gazebo Z-up camera pose (+X forward, +Z up) into orbit parameters."""
        pos = Vector((pose.px, pose.py, pose.pz))
        # Blender Quaternion is (w, x, y, z); wire Pose is (qx, qy, qz, qw)
        q = Quaternion((pose.qw, pose.qx, pose.qy, pose.qz))
        if q.magnitude > 1e-4:
            q.normalize()
        else:
            q = Quaternion((1.0, 0.0, 0.0, 0.0))

        fwd = q @ Vector((1.0, 0.0, 0.0))
        if fwd.length_squared < 1e-6:
            return
        fwd.normalize()

        # Intersect ground plane z=0 if looking downward, else place focus ahead
        if fwd.z < -0.05 and pos.z > 0.05:
            dist = max(1.0, min(25.0, -pos.z / fwd.z))
        else:
            dist = max(1.5, min(15.0, pos.length))

        self.focus = pos + fwd * dist
        offset = pos - self.focus
        self.distance = max(offset.length, 0.2)
        dir_to_cam = offset / self.distance
        self.pitch = math.asin(max(-0.999, min(0.999, dir_to_cam.z)))
        self.yaw = math.atan2(dir_to_cam.y, dir_to_cam.x)

        hfov_norm = math.radians(hfov_rad) if hfov_rad > 3.2 else hfov_rad
        if 0.05 < hfov_norm < 3.0:
            self.hfov_rad = hfov_norm

        self.save_initial()
        self.dirty = True

    def set_from_euler6(self, x: float, y: float, z: float, r: float, p: float, yaw: float, hfov: float) -> None:
        q = Euler((r, p, yaw), "XYZ").to_quaternion()
        self.set_from_gazebo_pose(Pose(x, y, z, q.x, q.y, q.z, q.w), hfov)

    def compute_camera_matrix(self) -> tuple[Matrix, Vector, Vector]:
        """Return (world_matrix, right_vec, up_vec) in Blender Z-up space (-Z forward, +Y up)."""
        cos_p = math.cos(self.pitch)
        offset = Vector(
            (
                self.distance * cos_p * math.cos(self.yaw),
                self.distance * cos_p * math.sin(self.yaw),
                self.distance * math.sin(self.pitch),
            )
        )
        cam_pos = self.focus + offset
        fwd = (self.focus - cam_pos).normalized()
        cam_z = -fwd

        world_up = Vector((0.0, 0.0, 1.0))
        cam_x = fwd.cross(world_up)
        if cam_x.length_squared < 1e-6:
            cam_x = Vector((1.0, 0.0, 0.0))
        else:
            cam_x.normalize()
        cam_y = cam_z.cross(cam_x).normalized()

        rot = Matrix(
            (
                (cam_x.x, cam_y.x, cam_z.x, 0.0),
                (cam_x.y, cam_y.y, cam_z.y, 0.0),
                (cam_x.z, cam_y.z, cam_z.z, 0.0),
                (0.0, 0.0, 0.0, 1.0),
            )
        )
        mat = Matrix.Translation(cam_pos) @ rot
        return mat, cam_x, cam_y

    def on_mouse_down(self, x: float, y: float, button: int, modifiers: int) -> None:
        self.last_mouse = (x, y)
        shift = (modifiers & 1) != 0
        if button == 1 and not shift:
            self.dragging_orbit = True
        elif button in (2, 4) or (button == 1 and shift):
            self.dragging_pan = True

    def on_mouse_up(self, button: int) -> None:
        if button == 1:
            self.dragging_orbit = False
            self.dragging_pan = False
        if button in (2, 4):
            self.dragging_pan = False

    def on_mouse_move(self, x: float, y: float, dx: float, dy: float) -> bool:
        if abs(dx) == 0.0 and abs(dy) == 0.0:
            dx = x - self.last_mouse[0]
            dy = y - self.last_mouse[1]
        self.last_mouse = (x, y)

        if self.dragging_orbit:
            sensitivity = 0.006
            self.yaw -= dx * sensitivity
            self.pitch = max(
                -math.pi * 0.5 + 0.03,
                min(math.pi * 0.5 - 0.03, self.pitch + dy * sensitivity),
            )
            self.dirty = True
            return True
        if self.dragging_pan:
            _, cam_x, cam_y = self.compute_camera_matrix()
            pan_scale = self.distance * 0.0018
            self.focus += (-cam_x * dx + cam_y * dy) * pan_scale
            self.dirty = True
            return True
        return False

    def on_wheel(self, dy: float) -> bool:
        factor = math.exp(-dy * 0.0012)
        self.distance = max(0.15, min(250.0, self.distance * factor))
        self.dirty = True
        return True

    def apply_to_blender_camera(self, cam_obj) -> None:
        mat, _, _ = self.compute_camera_matrix()
        cam_obj.matrix_world = mat
        cam_data = cam_obj.data
        cam_data.type = "PERSP"
        cam_data.lens_unit = "FOV"
        cam_data.sensor_fit = "HORIZONTAL"
        cam_data.angle = max(0.1, min(2.8, self.hfov_rad))
        cam_data.clip_start = 0.05
        cam_data.clip_end = 500.0
        self.dirty = False
