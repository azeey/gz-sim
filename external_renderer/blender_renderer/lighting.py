"""Photorealism-first Blender 5.0 lighting, physical world sky/HDRI, and AgX color management."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Optional

import bpy
from mathutils import Quaternion, Vector

from bridge import (
    BLENDER_AGX,
    BLENDER_IBL,
    BLENDER_SHADOWS,
    LIGHT_DIRECTIONAL,
    LIGHT_POINT,
    LIGHT_RECT,
    LIGHT_SPOT,
    LightUpsert,
    WorldConfig,
)


def _find_builtin_hdri(name: str) -> Optional[Path]:
    """Locate one of Blender 5.0's built-in EXR studio/world HDRIs."""
    try:
        p = Path(bpy.utils.resource_path("LOCAL")) / "datafiles" / "studiolights" / "world" / name
        if p.exists():
            return p
    except Exception:
        pass
    fallback = Path(
        f"/Applications/Blender.app/Contents/Resources/5.0/datafiles/studiolights/world/{name}"
    )
    return fallback if fallback.exists() else None


class LightingManager:
    """Manages physical Blender lights, Nishita/HDRI world shader, and AgX color management."""

    def __init__(self, scene_collection) -> None:
        self.collection = scene_collection
        self.lights: dict[int, object] = {}  # entity_id -> bpy.types.Object
        self.light_cast_shadows: dict[int, bool] = {}
        self.light_types: dict[int, int] = {}
        self.primary_sun_dir: Optional[Vector] = None
        self.world_cfg: Optional[WorldConfig] = None
        self._hdri_cache: dict[str, object] = {}

    def clear_lights(self) -> None:
        for obj in list(self.lights.values()):
            try:
                data = obj.data
                bpy.data.objects.remove(obj, do_unlink=True)
                if data and data.users == 0:
                    bpy.data.lights.remove(data)
            except ReferenceError:
                pass
        self.lights.clear()
        self.light_cast_shadows.clear()
        self.light_types.clear()
        self.primary_sun_dir = None

    def remove_light(self, entity_id: int) -> bool:
        obj = self.lights.pop(entity_id, None)
        self.light_cast_shadows.pop(entity_id, None)
        self.light_types.pop(entity_id, None)
        if obj is not None:
            try:
                data = obj.data
                bpy.data.objects.remove(obj, do_unlink=True)
                if data and data.users == 0:
                    bpy.data.lights.remove(data)
            except ReferenceError:
                pass
            return True
        return False

    def upsert_light(self, lw: LightUpsert, quality_flags: int) -> None:
        self.remove_light(lw.entity_id)

        if lw.light_type == LIGHT_DIRECTIONAL:
            bl_type = "SUN"
        elif lw.light_type == LIGHT_POINT:
            bl_type = "POINT"
        elif lw.light_type == LIGHT_RECT:
            bl_type = "AREA"
        else:
            bl_type = "SPOT"

        ldata = bpy.data.lights.new(name=f"gz_light_{lw.entity_id}", type=bl_type)
        ldata.color = (
            max(0.0, min(1.0, lw.color[0])),
            max(0.0, min(1.0, lw.color[1])),
            max(0.0, min(1.0, lw.color[2])),
        )

        # True physical inverse-square lighting with soft emitter radii (no Ogre attenuation hacks)
        use_shadows = bool(lw.cast_shadows) and bool(quality_flags & BLENDER_SHADOWS)
        ldata.use_shadow = use_shadows

        if bl_type == "SUN":
            ldata.energy = max(lw.intensity, 0.05) * 3.0
            ldata.angle = math.radians(1.8)
        elif bl_type == "POINT":
            ldata.energy = max(lw.intensity, 0.05) * 60.0
            ldata.shadow_soft_size = 0.12
        elif bl_type == "AREA":
            # Direct physical rectangular AreaLight (Watts, meters, beam spread)
            ldata.shape = "RECTANGLE"
            ldata.size = max(0.05, lw.inner_angle)
            ldata.size_y = max(0.05, lw.outer_angle)
            ldata.energy = max(0.1, lw.intensity)
            ldata.spread = max(math.radians(5.0), min(math.pi, lw.spread))
        else:
            ldata.energy = max(lw.intensity, 0.05) * 30.0
            ldata.shadow_soft_size = 0.20
            outer = max(0.08, min(math.pi - 0.02, lw.outer_angle))
            inner = max(0.02, min(outer, lw.inner_angle))
            ldata.spot_size = outer
            ldata.spot_blend = max(0.08, min(1.0, 1.0 - (inner / max(outer, 0.01))))

        lobj = bpy.data.objects.new(name=f"gz_light_{lw.entity_id}", object_data=ldata)
        self.collection.objects.link(lobj)

        # Position & orientation in direct Z-up space (Blender SUN/SPOT/AREA emit along local -Z)
        lobj.location = (lw.pose.px, lw.pose.py, lw.pose.pz)
        q = Quaternion((lw.pose.qw, lw.pose.qx, lw.pose.qy, lw.pose.qz))
        if q.magnitude > 1e-4:
            q.normalize()
        else:
            q = Quaternion((1.0, 0.0, 0.0, 0.0))

        local_dir = Vector((lw.direction[0], lw.direction[1], lw.direction[2]))
        if local_dir.length_squared < 1e-8:
            local_dir = Vector((0.0, 0.0, -1.0))
        world_dir = (q @ local_dir).normalized()

        if bl_type == "AREA":
            lobj.rotation_mode = "QUATERNION"
            lobj.rotation_quaternion = q
            # Keep area light quads invisible to camera rays so physical fixtures & sky stay unobstructed
            lobj.visible_camera = False
        elif bl_type in ("SUN", "SPOT"):
            rot_q = Vector((0.0, 0.0, -1.0)).rotation_difference(world_dir)
            lobj.rotation_mode = "QUATERNION"
            lobj.rotation_quaternion = rot_q

        if bl_type == "SUN" and self.primary_sun_dir is None:
            self.primary_sun_dir = world_dir.copy()

        self.lights[lw.entity_id] = lobj
        self.light_cast_shadows[lw.entity_id] = bool(lw.cast_shadows)
        self.light_types[lw.entity_id] = lw.light_type

    def apply_shadow_flag(self, quality_flags: int) -> None:
        shadows_on = bool(quality_flags & BLENDER_SHADOWS)
        for eid, obj in self.lights.items():
            obj.data.use_shadow = shadows_on and self.light_cast_shadows.get(eid, True)

    def apply_color_management(self, scene, quality_flags: int) -> None:
        vs = scene.view_settings
        if quality_flags & BLENDER_AGX:
            vs.view_transform = "AgX"
            vs.look = "AgX - Medium High Contrast"
        else:
            vs.view_transform = "Khronos PBR Neutral"
            vs.look = "None"
        vs.exposure = 0.0
        vs.gamma = 1.0

    def _load_hdri(self, path: Path):
        key = str(path)
        img = self._hdri_cache.get(key)
        if img is not None:
            try:
                _ = img.name
                return img
            except ReferenceError:
                pass
        img = bpy.data.images.load(key, check_existing=True)
        self._hdri_cache[key] = img
        return img

    def apply_world(self, scene, quality_flags: int, cfg: Optional[WorldConfig] = None) -> None:
        if cfg is not None:
            self.world_cfg = cfg
        wcfg = self.world_cfg

        world = scene.world
        if world is None:
            world = bpy.data.worlds.new("GazeboWorld")
            scene.world = world

        world.use_nodes = True
        if hasattr(world, "cycles"):
            world.cycles.sampling_method = "AUTOMATIC"
        nt = world.node_tree
        nt.nodes.clear()

        out_node = nt.nodes.new("ShaderNodeOutputWorld")
        out_node.location = (600, 0)

        ibl_on = bool(quality_flags & BLENDER_IBL)
        bg_col = wcfg.background if wcfg else (0.12, 0.14, 0.18, 1.0)
        has_sky = wcfg.has_sky if wcfg else False
        env_path = Path(wcfg.env_map_path) if (wcfg and wcfg.env_map_path) else None

        # 1. Explicit HDRI env map from SDF
        if env_path and env_path.exists():
            env_node = nt.nodes.new("ShaderNodeTexEnvironment")
            env_node.image = self._load_hdri(env_path)
            bg = nt.nodes.new("ShaderNodeBackground")
            raw_int = wcfg.sky_intensity if wcfg else 1.0
            env_strength = raw_int if (0.0 <= raw_int <= 10.0) else 0.35
            bg.inputs["Strength"].default_value = env_strength if ibl_on else min(0.15, env_strength)
            nt.links.new(env_node.outputs["Color"], bg.inputs["Color"])
            nt.links.new(bg.outputs["Background"], out_node.inputs["Surface"])
            return

        # 2. Physical Multiple-Scattering Atmosphere Sky for worlds with <sky> when IBL is enabled
        if ibl_on and has_sky:
            sky = nt.nodes.new("ShaderNodeTexSky")
            sky.sky_type = "MULTIPLE_SCATTERING"
            sky.sun_disc = True
            sky.sun_size = math.radians(1.2)
            sky.sun_intensity = 0.30
            sky.air_density = 1.0
            sky.aerosol_density = 1.0
            sky.ozone_density = 1.5
            # Align Nishita sun with primary directional light if present
            if self.primary_sun_dir is not None:
                to_sun = (-self.primary_sun_dir).normalized()
                elev = math.asin(max(-0.1, min(1.0, to_sun.z)))
                sky.sun_elevation = max(math.radians(8.0), elev)
                sky.sun_rotation = math.atan2(to_sun.x, to_sun.y)
            else:
                sky.sun_elevation = math.radians(38.0)
                sky.sun_rotation = math.radians(35.0)

            bg_sky = nt.nodes.new("ShaderNodeBackground")
            sky_mult = wcfg.sky_intensity if (wcfg and 0.0 <= wcfg.sky_intensity <= 10.0) else 1.0
            bg_sky.inputs["Strength"].default_value = 0.42 * sky_mult
            nt.links.new(sky.outputs["Color"], bg_sky.inputs["Color"])
            nt.links.new(bg_sky.outputs["Background"], out_node.inputs["Surface"])
            return

        # 3. Pure-interior zero-IBL mode (sky_intensity == 0): zero exterior world lighting
        #    so Cycles skips wasted world MIS rays inside enclosed halls, while keeping
        #    <background> visible to camera rays through windows/openings.
        if wcfg and not has_sky and wcfg.sky_intensity <= 1e-4:
            if hasattr(world, "cycles"):
                world.cycles.sampling_method = "NONE"
            bg_dark = nt.nodes.new("ShaderNodeBackground")
            bg_dark.location = (60, 140)
            bg_dark.inputs["Color"].default_value = (0.0, 0.0, 0.0, 1.0)
            bg_dark.inputs["Strength"].default_value = 0.0

            bg_cam = nt.nodes.new("ShaderNodeBackground")
            bg_cam.location = (60, -80)
            bg_cam.inputs["Color"].default_value = (bg_col[0], bg_col[1], bg_col[2], 1.0)
            bg_cam.inputs["Strength"].default_value = 1.0

            lp = nt.nodes.new("ShaderNodeLightPath")
            lp.location = (60, 320)
            mix = nt.nodes.new("ShaderNodeMixShader")
            mix.location = (340, 60)
            nt.links.new(lp.outputs["Is Camera Ray"], mix.inputs["Fac"])
            nt.links.new(bg_dark.outputs["Background"], mix.inputs[1])
            nt.links.new(bg_cam.outputs["Background"], mix.inputs[2])
            nt.links.new(mix.outputs["Shader"], out_node.inputs["Surface"])
            return

        # 4. Studio / Architectural HDRI for photoreal reflections + clean camera backdrop
        if ibl_on:
            num_local_lights = sum(
                1 for t in self.light_types.values() if t in (LIGHT_POINT, LIGHT_SPOT, LIGHT_RECT)
            )
            is_indoor = num_local_lights >= 4
            hdri_name = "interior.exr" if is_indoor else "studio.exr"
            hdri_path = _find_builtin_hdri(hdri_name)
            if hdri_path is not None:
                env_node = nt.nodes.new("ShaderNodeTexEnvironment")
                env_node.location = (-300, 140)
                env_node.image = self._load_hdri(hdri_path)

                bg_ibl = nt.nodes.new("ShaderNodeBackground")
                bg_ibl.location = (60, 140)
                sky_mult = wcfg.sky_intensity if (wcfg and 0.0 <= wcfg.sky_intensity <= 10.0) else 1.0
                bg_ibl.inputs["Strength"].default_value = (0.20 if is_indoor else 0.55) * sky_mult
                nt.links.new(env_node.outputs["Color"], bg_ibl.inputs["Color"])

                bg_cam = nt.nodes.new("ShaderNodeBackground")
                bg_cam.location = (60, -80)
                if is_indoor:
                    bg_cam.inputs["Color"].default_value = (0.04, 0.045, 0.055, 1.0)
                    bg_cam.inputs["Strength"].default_value = 1.0
                else:
                    # Clean neutral studio backdrop derived from SDF background
                    bg_cam.inputs["Color"].default_value = (
                        bg_col[0] * 0.45,
                        bg_col[1] * 0.45,
                        bg_col[2] * 0.45,
                        1.0,
                    )
                    bg_cam.inputs["Strength"].default_value = 1.0

                lp = nt.nodes.new("ShaderNodeLightPath")
                lp.location = (60, 320)
                mix = nt.nodes.new("ShaderNodeMixShader")
                mix.location = (340, 60)
                nt.links.new(lp.outputs["Is Camera Ray"], mix.inputs["Fac"])
                nt.links.new(bg_ibl.outputs["Background"], mix.inputs[1])
                nt.links.new(bg_cam.outputs["Background"], mix.inputs[2])
                nt.links.new(mix.outputs["Shader"], out_node.inputs["Surface"])
                return

        # 4. IBL OFF: clean low physical ambient so direct lights & shadows stay dramatic
        bg = nt.nodes.new("ShaderNodeBackground")
        bg.inputs["Color"].default_value = (
            max(0.02, bg_col[0] * 0.25),
            max(0.02, bg_col[1] * 0.25),
            max(0.02, bg_col[2] * 0.25),
            1.0,
        )
        bg.inputs["Strength"].default_value = 0.35
        nt.links.new(bg.outputs["Background"], out_node.inputs["Surface"])
