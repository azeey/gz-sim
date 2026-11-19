"""ctypes bindings to libgz_blender_bridge.dylib + binary wire protocol decoders."""

from __future__ import annotations

import ctypes
import math
import os
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np

PROTOCOL_MAGIC = 0x475A4256  # "GZBV"
PROTOCOL_VERSION = 1

# MsgType
MSG_WORLD_CONFIG = 1
MSG_TEXTURE_DATA = 2
MSG_MESH_DATA = 3
MSG_SPAWN_VISUAL = 4
MSG_DESPAWN_ENTITY = 5
MSG_LIGHT_UPSERT = 6
MSG_TRANSFORM_BATCH = 7
MSG_CLEAR_WORLD = 8
MSG_SNAPSHOT_REQUEST = 9
MSG_GUI_INPUT = 10
MSG_FRAME_READY = 11
MSG_SLOT_ACQUIRED = 12

# VisualKind
VISUAL_BOX = 0
VISUAL_SPHERE = 1
VISUAL_CYLINDER = 2
VISUAL_CAPSULE = 3
VISUAL_PLANE = 4
VISUAL_STREAMED_MESH = 5
VISUAL_GLTF_URI = 6
VISUAL_CONE = 7
VISUAL_ELLIPSOID = 8

# AlphaMode
ALPHA_OPAQUE = 0
ALPHA_MASK = 1
ALPHA_BLEND = 2

# LightType
LIGHT_DIRECTIONAL = 0
LIGHT_POINT = 1
LIGHT_SPOT = 2
LIGHT_RECT = 3
LIGHT_AREA = 3

# InputKind
INPUT_MOUSE_DOWN = 0
INPUT_MOUSE_UP = 1
INPUT_MOUSE_MOVE = 2
INPUT_WHEEL = 3
INPUT_KEY_DOWN = 4
INPUT_KEY_UP = 5
INPUT_RESIZE = 6
INPUT_SET_CAMERA_POSE = 7
INPUT_QUALITY_CONFIG = 8
INPUT_SAVE_SCREENSHOT = 9
INPUT_RESET_CAMERA = 10
INPUT_PING = 11
INPUT_SAVE_SCENE = 12

# BlenderQualityFlags (must match common/protocol.hh)
BLENDER_CYCLES = 1 << 0
BLENDER_IBL = 1 << 1
BLENDER_RT_GI = 1 << 2
BLENDER_DENOISE = 1 << 3
BLENDER_AGX = 1 << 4
BLENDER_SHADOWS = 1 << 5
BLENDER_HIGH_QUALITY = 1 << 6
BLENDER_DEFAULT = (
    BLENDER_IBL | BLENDER_RT_GI | BLENDER_DENOISE | BLENDER_AGX | BLENDER_SHADOWS
)

# Wire struct formats (compile-time parity with common/protocol.hh)
_HDR_STRUCT = struct.Struct("<IHHII")
_WORLD_CFG_STRUCT = struct.Struct("<4f4fBBHf7ffI")
_TEX_DATA_STRUCT = struct.Struct("<QIIB3xI")
_MESH_DATA_STRUCT = struct.Struct("<QIIBBBBI")
_PBR_MAT_STRUCT = struct.Struct("<4f4f4f5Q")
_SPAWN_VIS_STRUCT = struct.Struct("<QQBBBBI7f7f4fQ88s")
_DESPAWN_STRUCT = struct.Struct("<Q")
_LIGHT_STRUCT = struct.Struct("<QBBHf7f3ff4fff")
_TF_BATCH_HDR_STRUCT = struct.Struct("<d")
_TF_ITEM_STRUCT = struct.Struct("<Q7fI")
_GUI_INPUT_STRUCT = struct.Struct("<B3x7I6f7fI")

assert _HDR_STRUCT.size == 16
assert _WORLD_CFG_STRUCT.size == 76
assert _TEX_DATA_STRUCT.size == 24
assert _MESH_DATA_STRUCT.size == 24
assert _PBR_MAT_STRUCT.size == 88
assert _SPAWN_VIS_STRUCT.size == 192
assert _DESPAWN_STRUCT.size == 8
assert _LIGHT_STRUCT.size == 84
assert _TF_BATCH_HDR_STRUCT.size == 8
assert _TF_ITEM_STRUCT.size == 40
assert _GUI_INPUT_STRUCT.size == 88


@dataclass(slots=True)
class Pose:
    px: float
    py: float
    pz: float
    qx: float
    qy: float
    qz: float
    qw: float


@dataclass(slots=True)
class PbrMaterial:
    base_color: tuple[float, float, float, float]
    emissive: tuple[float, float, float, float]
    metallic: float
    roughness: float
    alpha_cutoff: float
    reflectance: float
    base_color_tex: int
    normal_tex: int
    metallic_roughness_tex: int
    occlusion_tex: int
    emissive_tex: int


@dataclass(slots=True)
class WorldConfig:
    ambient: tuple[float, float, float, float]
    background: tuple[float, float, float, float]
    has_sky: bool
    has_camera_pose: bool
    sky_intensity: float
    camera_pose: Pose
    camera_hfov: float
    env_map_path: str


@dataclass(slots=True)
class TextureData:
    texture_id: int
    width: int
    height: int
    fmt: int  # 0 = RGBA8_SRGB, 1 = RGBA8_UNORM
    pixels: np.ndarray  # (H, W, 4) uint8 view


@dataclass(slots=True)
class MeshData:
    mesh_id: int
    vertex_count: int
    index_count: int
    positions: np.ndarray  # (V, 3) float32
    normals: Optional[np.ndarray]  # (V, 3) float32
    uvs: Optional[np.ndarray]  # (V, 2) float32
    indices: np.ndarray  # (I,) uint32


@dataclass(slots=True)
class SpawnVisual:
    visual_id: int
    parent_entity_id: int
    kind: int
    cast_shadows: bool
    double_sided: bool
    alpha_mode: int
    parent_pose: Pose
    local_pose: Pose
    params: tuple[float, float, float, float]
    mesh_id: int
    material: PbrMaterial
    uri: str


@dataclass(slots=True)
class LightUpsert:
    entity_id: int
    light_type: int
    cast_shadows: bool
    intensity: float
    pose: Pose
    direction: tuple[float, float, float]
    range: float
    color: tuple[float, float, float, float]
    inner_angle: float
    outer_angle: float
    spread: float = math.pi


@dataclass(slots=True)
class GuiInput:
    kind: int
    button: int
    buttons_mask: int
    modifiers: int
    key_code: int
    width: int
    height: int
    quality_flags: int
    x: float
    y: float
    dx: float
    dy: float
    dpr: float
    camera_hfov: float
    camera_pose: Pose
    path: str


def parse_header(buf: bytes) -> Optional[tuple[int, int, memoryview]]:
    if len(buf) < 16:
        return None
    magic, version, msg_type, count, payload_bytes = _HDR_STRUCT.unpack_from(buf, 0)
    if magic != PROTOCOL_MAGIC or version != PROTOCOL_VERSION:
        return None
    if len(buf) < 16 + payload_bytes:
        return None
    return msg_type, count, memoryview(buf)[16 : 16 + payload_bytes]


def decode_world_config(payload: memoryview) -> Optional[WorldConfig]:
    if len(payload) < _WORLD_CFG_STRUCT.size:
        return None
    u = _WORLD_CFG_STRUCT.unpack_from(payload, 0)
    env_len = u[20]
    env_path = ""
    if env_len > 0 and len(payload) >= _WORLD_CFG_STRUCT.size + env_len:
        env_path = bytes(payload[_WORLD_CFG_STRUCT.size : _WORLD_CFG_STRUCT.size + env_len]).decode(
            "utf-8", errors="ignore"
        )
    return WorldConfig(
        ambient=(u[0], u[1], u[2], u[3]),
        background=(u[4], u[5], u[6], u[7]),
        has_sky=u[8] != 0,
        has_camera_pose=u[9] != 0,
        sky_intensity=u[11],
        camera_pose=Pose(u[12], u[13], u[14], u[15], u[16], u[17], u[18]),
        camera_hfov=u[19],
        env_map_path=env_path,
    )


def decode_texture_data(payload: memoryview) -> Optional[TextureData]:
    if len(payload) < _TEX_DATA_STRUCT.size:
        return None
    tex_id, w, h, fmt, data_len = _TEX_DATA_STRUCT.unpack_from(payload, 0)
    off = _TEX_DATA_STRUCT.size
    if w == 0 or h == 0 or len(payload) < off + data_len or data_len < w * h * 4:
        return None
    raw = np.frombuffer(payload[off : off + w * h * 4], dtype=np.uint8).copy()
    return TextureData(
        texture_id=tex_id,
        width=w,
        height=h,
        fmt=fmt,
        pixels=raw.reshape((h, w, 4)),
    )


def decode_mesh_data(payload: memoryview) -> Optional[MeshData]:
    if len(payload) < _MESH_DATA_STRUCT.size:
        return None
    mesh_id, vcount, icount, has_n, has_uv, has_tan, _pad, _pad2 = _MESH_DATA_STRUCT.unpack_from(
        payload, 0
    )
    if vcount == 0 or icount < 3:
        return None
    off = _MESH_DATA_STRUCT.size

    pos_bytes = vcount * 3 * 4
    if len(payload) < off + pos_bytes:
        return None
    positions = np.frombuffer(payload[off : off + pos_bytes], dtype="<f4").copy().reshape((vcount, 3))
    off += pos_bytes

    normals = None
    if has_n:
        n_bytes = vcount * 3 * 4
        if len(payload) < off + n_bytes:
            return None
        normals = np.frombuffer(payload[off : off + n_bytes], dtype="<f4").copy().reshape((vcount, 3))
        off += n_bytes

    uvs = None
    if has_uv:
        uv_bytes = vcount * 2 * 4
        if len(payload) < off + uv_bytes:
            return None
        uvs = np.frombuffer(payload[off : off + uv_bytes], dtype="<f4").copy().reshape((vcount, 2))
        off += uv_bytes

    if has_tan:
        tan_bytes = vcount * 4 * 4
        if len(payload) < off + tan_bytes:
            return None
        off += tan_bytes

    idx_bytes = icount * 4
    if len(payload) < off + idx_bytes:
        return None
    indices = np.frombuffer(payload[off : off + idx_bytes], dtype="<u4").copy()
    return MeshData(
        mesh_id=mesh_id,
        vertex_count=vcount,
        index_count=icount,
        positions=positions,
        normals=normals,
        uvs=uvs,
        indices=indices,
    )


def decode_spawn_visual(payload: memoryview) -> Optional[SpawnVisual]:
    if len(payload) < _SPAWN_VIS_STRUCT.size:
        return None
    u = _SPAWN_VIS_STRUCT.unpack_from(payload, 0)
    uri_len = u[6]
    mat_raw = u[26]
    m = _PBR_MAT_STRUCT.unpack(mat_raw)
    mat = PbrMaterial(
        base_color=(m[0], m[1], m[2], m[3]),
        emissive=(m[4], m[5], m[6], m[7]),
        metallic=m[8],
        roughness=m[9],
        alpha_cutoff=m[10],
        reflectance=m[11],
        base_color_tex=m[12],
        normal_tex=m[13],
        metallic_roughness_tex=m[14],
        occlusion_tex=m[15],
        emissive_tex=m[16],
    )
    uri = ""
    if uri_len > 0 and len(payload) >= _SPAWN_VIS_STRUCT.size + uri_len:
        uri = bytes(payload[_SPAWN_VIS_STRUCT.size : _SPAWN_VIS_STRUCT.size + uri_len]).decode(
            "utf-8", errors="ignore"
        )
    return SpawnVisual(
        visual_id=u[0],
        parent_entity_id=u[1],
        kind=u[2],
        cast_shadows=u[3] != 0,
        double_sided=u[4] != 0,
        alpha_mode=u[5],
        parent_pose=Pose(u[7], u[8], u[9], u[10], u[11], u[12], u[13]),
        local_pose=Pose(u[14], u[15], u[16], u[17], u[18], u[19], u[20]),
        params=(u[21], u[22], u[23], u[24]),
        mesh_id=u[25],
        material=mat,
        uri=uri,
    )


def decode_despawn(payload: memoryview) -> Optional[int]:
    if len(payload) < _DESPAWN_STRUCT.size:
        return None
    return _DESPAWN_STRUCT.unpack_from(payload, 0)[0]


def decode_light_upsert(payload: memoryview) -> Optional[LightUpsert]:
    if len(payload) < _LIGHT_STRUCT.size:
        return None
    u = _LIGHT_STRUCT.unpack_from(payload, 0)
    spread = (u[3] / 1000.0) if u[3] > 0 else math.pi
    return LightUpsert(
        entity_id=u[0],
        light_type=u[1],
        cast_shadows=u[2] != 0,
        intensity=u[4],
        pose=Pose(u[5], u[6], u[7], u[8], u[9], u[10], u[11]),
        direction=(u[12], u[13], u[14]),
        range=u[15],
        color=(u[16], u[17], u[18], u[19]),
        inner_angle=u[20],
        outer_angle=u[21],
        spread=spread,
    )


def decode_transform_batch(payload: memoryview, count: int) -> tuple[float, list[tuple[int, Pose]]]:
    if len(payload) < _TF_BATCH_HDR_STRUCT.size:
        return 0.0, []
    (sim_time,) = _TF_BATCH_HDR_STRUCT.unpack_from(payload, 0)
    off = _TF_BATCH_HDR_STRUCT.size
    items: list[tuple[int, Pose]] = []
    step = _TF_ITEM_STRUCT.size
    for _ in range(count):
        if off + step > len(payload):
            break
        u = _TF_ITEM_STRUCT.unpack_from(payload, off)
        off += step
        items.append((u[0], Pose(u[1], u[2], u[3], u[4], u[5], u[6], u[7])))
    return sim_time, items


def decode_gui_input(payload: memoryview) -> Optional[GuiInput]:
    if len(payload) < _GUI_INPUT_STRUCT.size:
        return None
    u = _GUI_INPUT_STRUCT.unpack_from(payload, 0)
    path_len = u[21]
    path = ""
    if path_len > 0 and len(payload) >= _GUI_INPUT_STRUCT.size + path_len:
        path = bytes(payload[_GUI_INPUT_STRUCT.size : _GUI_INPUT_STRUCT.size + path_len]).decode(
            "utf-8", errors="ignore"
        )
    return GuiInput(
        kind=u[0],
        button=u[1],
        buttons_mask=u[2],
        modifiers=u[3],
        key_code=u[4],
        width=u[5],
        height=u[6],
        quality_flags=u[7],
        x=u[8],
        y=u[9],
        dx=u[10],
        dy=u[11],
        dpr=u[12],
        camera_hfov=u[13],
        camera_pose=Pose(u[14], u[15], u[16], u[17], u[18], u[19], u[20]),
        path=path,
    )


def _find_bridge_lib() -> Path:
    env = os.environ.get("GZ_BLENDER_BRIDGE_LIB")
    if env and Path(env).exists():
        return Path(env)
    here = Path(__file__).resolve().parent
    candidates = [
        here.parent / "build" / "libgz_blender_bridge.dylib",
        here / "libgz_blender_bridge.dylib",
    ]
    for c in candidates:
        if c.exists():
            return c
    raise FileNotFoundError(
        f"Could not locate libgz_blender_bridge.dylib (checked {[str(c) for c in candidates]})"
    )


class NativeBridge:
    """Thin ctypes wrapper over libgz_blender_bridge.dylib (Zenoh SHM + Mach IOSurface)."""

    def __init__(self, session: str, endpoint: str, width: int, height: int):
        lib_path = _find_bridge_lib()
        self._lib = ctypes.CDLL(str(lib_path))
        self._setup_signatures()
        self._handle = self._lib.gz_blender_bridge_create(
            session.encode("utf-8"),
            endpoint.encode("utf-8"),
            ctypes.c_uint32(width),
            ctypes.c_uint32(height),
        )
        if not self._handle:
            raise RuntimeError(
                f"gz_blender_bridge_create failed (session={session}, endpoint={endpoint})"
            )

    def _setup_signatures(self) -> None:
        lib = self._lib
        lib.gz_blender_bridge_create.argtypes = [
            ctypes.c_char_p,
            ctypes.c_char_p,
            ctypes.c_uint32,
            ctypes.c_uint32,
        ]
        lib.gz_blender_bridge_create.restype = ctypes.c_void_p

        lib.gz_blender_bridge_destroy.argtypes = [ctypes.c_void_p]
        lib.gz_blender_bridge_destroy.restype = None

        lib.gz_blender_bridge_resize.argtypes = [ctypes.c_void_p, ctypes.c_uint32, ctypes.c_uint32]
        lib.gz_blender_bridge_resize.restype = ctypes.c_bool

        lib.gz_blender_bridge_width.argtypes = [ctypes.c_void_p]
        lib.gz_blender_bridge_width.restype = ctypes.c_uint32

        lib.gz_blender_bridge_height.argtypes = [ctypes.c_void_p]
        lib.gz_blender_bridge_height.restype = ctypes.c_uint32

        lib.gz_blender_bridge_request_snapshot.argtypes = [ctypes.c_void_p, ctypes.c_uint64]
        lib.gz_blender_bridge_request_snapshot.restype = ctypes.c_bool

        lib.gz_blender_bridge_pop_scene.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.POINTER(ctypes.c_uint8)),
            ctypes.POINTER(ctypes.c_uint32),
            ctypes.POINTER(ctypes.c_uint8),
        ]
        lib.gz_blender_bridge_pop_scene.restype = ctypes.c_bool

        lib.gz_blender_bridge_pop_input.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.POINTER(ctypes.c_uint8)),
            ctypes.POINTER(ctypes.c_uint32),
        ]
        lib.gz_blender_bridge_pop_input.restype = ctypes.c_bool

        lib.gz_blender_bridge_has_pending.argtypes = [ctypes.c_void_p]
        lib.gz_blender_bridge_has_pending.restype = ctypes.c_bool

        lib.gz_blender_bridge_present_tga_file.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_uint32,
            ctypes.c_uint32,
            ctypes.c_uint8,
            ctypes.c_uint16,
        ]
        lib.gz_blender_bridge_present_tga_file.restype = ctypes.c_bool

        lib.gz_blender_bridge_reannounce_frame.argtypes = [
            ctypes.c_void_p,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_uint32,
            ctypes.c_uint32,
            ctypes.c_uint8,
            ctypes.c_uint16,
        ]
        lib.gz_blender_bridge_reannounce_frame.restype = ctypes.c_bool

        lib.gz_blender_bridge_save_png.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
        lib.gz_blender_bridge_save_png.restype = ctypes.c_bool

    @property
    def width(self) -> int:
        return int(self._lib.gz_blender_bridge_width(self._handle))

    @property
    def height(self) -> int:
        return int(self._lib.gz_blender_bridge_height(self._handle))

    def resize(self, width: int, height: int) -> bool:
        return bool(self._lib.gz_blender_bridge_resize(self._handle, width, height))

    def request_snapshot(self, client_id: int) -> bool:
        return bool(self._lib.gz_blender_bridge_request_snapshot(self._handle, client_id))

    def has_pending(self) -> bool:
        return bool(self._lib.gz_blender_bridge_has_pending(self._handle))

    def pop_scene(self) -> Optional[tuple[bytes, bool]]:
        ptr = ctypes.POINTER(ctypes.c_uint8)()
        length = ctypes.c_uint32(0)
        is_shm = ctypes.c_uint8(0)
        if not self._lib.gz_blender_bridge_pop_scene(
            self._handle, ctypes.byref(ptr), ctypes.byref(length), ctypes.byref(is_shm)
        ):
            return None
        return ctypes.string_at(ptr, length.value), bool(is_shm.value)

    def pop_input(self) -> Optional[bytes]:
        ptr = ctypes.POINTER(ctypes.c_uint8)()
        length = ctypes.c_uint32(0)
        if not self._lib.gz_blender_bridge_pop_input(
            self._handle, ctypes.byref(ptr), ctypes.byref(length)
        ):
            return None
        return ctypes.string_at(ptr, length.value)

    def present_tga_file(
        self,
        tga_path: str,
        render_ms: float,
        fps: float,
        entity_count: int,
        quality_flags: int,
        refining: bool,
        sample_count: int,
    ) -> bool:
        return bool(
            self._lib.gz_blender_bridge_present_tga_file(
                self._handle,
                tga_path.encode("utf-8"),
                float(render_ms),
                float(fps),
                int(entity_count),
                int(quality_flags),
                1 if refining else 0,
                int(sample_count),
            )
        )

    def reannounce_frame(
        self,
        render_ms: float,
        fps: float,
        entity_count: int,
        quality_flags: int,
        refining: bool,
        sample_count: int,
    ) -> bool:
        return bool(
            self._lib.gz_blender_bridge_reannounce_frame(
                self._handle,
                float(render_ms),
                float(fps),
                int(entity_count),
                int(quality_flags),
                1 if refining else 0,
                int(sample_count),
            )
        )

    def save_png(self, png_path: str) -> bool:
        return bool(self._lib.gz_blender_bridge_save_png(self._handle, png_path.encode("utf-8")))

    def close(self) -> None:
        if getattr(self, "_handle", None):
            self._lib.gz_blender_bridge_destroy(self._handle)
            self._handle = None
