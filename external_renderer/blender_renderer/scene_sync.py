"""Gazebo ECM -> Blender 5.0 scene synchronization (primitives, cached glTF, streamed SHM meshes)."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Optional

import bmesh
import bpy
import numpy as np
from mathutils import Euler, Matrix, Quaternion, Vector

from bridge import (
    ALPHA_BLEND,
    ALPHA_MASK,
    MeshData,
    PbrMaterial,
    Pose,
    SpawnVisual,
    TextureData,
    VISUAL_BOX,
    VISUAL_CAPSULE,
    VISUAL_CONE,
    VISUAL_CYLINDER,
    VISUAL_ELLIPSOID,
    VISUAL_GLTF_URI,
    VISUAL_PLANE,
    VISUAL_SPHERE,
    VISUAL_STREAMED_MESH,
)


def _apply_pose(obj, pose: Pose, scale: tuple[float, float, float] = (1.0, 1.0, 1.0)) -> None:
    obj.location = (pose.px, pose.py, pose.pz)
    q = Quaternion((pose.qw, pose.qx, pose.qy, pose.qz))
    if q.magnitude > 1e-4:
        q.normalize()
    else:
        q = Quaternion((1.0, 0.0, 0.0, 0.0))
    obj.rotation_mode = "QUATERNION"
    obj.rotation_quaternion = q
    obj.scale = scale


def _set_smooth(mesh, smooth: bool = True) -> None:
    if len(mesh.polygons) > 0:
        mesh.polygons.foreach_set("use_smooth", [smooth] * len(mesh.polygons))


def _build_primitive_mesh(kind: int, params: tuple[float, float, float, float], name: str):
    """Build crisp, smooth-shaded Z-up Gazebo primitive geometry using bmesh."""
    mesh = bpy.data.meshes.new(name)
    bm = bmesh.new()

    p0, p1, p2, _ = params
    if kind == VISUAL_BOX:
        sx, sy, sz = max(p0, 1e-4), max(p1, 1e-4), max(p2, 1e-4)
        bmesh.ops.create_cube(bm, size=1.0)
        bmesh.ops.scale(bm, vec=(sx, sy, sz), verts=bm.verts)
        uv_layer = bm.loops.layers.uv.verify()
        for f in bm.faces:
            n = f.normal
            ax, ay, az = abs(n.x), abs(n.y), abs(n.z)
            for loop in f.loops:
                co = loop.vert.co
                if az >= ax and az >= ay:
                    u, v = (co.x / sx + 0.5), (co.y / sy + 0.5)
                elif ax >= ay:
                    u, v = (co.y / sy + 0.5), (co.z / sz + 0.5)
                else:
                    u, v = (co.x / sx + 0.5), (co.z / sz + 0.5)
                loop[uv_layer].uv = (u, v)
        bm.to_mesh(mesh)
        bm.free()
        _set_smooth(mesh, False)
        return mesh

    if kind == VISUAL_SPHERE:
        r = max(p0, 1e-4)
        bmesh.ops.create_uvsphere(bm, u_segments=48, v_segments=24, radius=r)
        bm.to_mesh(mesh)
        bm.free()
        _set_smooth(mesh, True)
        return mesh

    if kind == VISUAL_ELLIPSOID:
        rx, ry, rz = max(p0, 1e-4), max(p1, 1e-4), max(p2, 1e-4)
        bmesh.ops.create_uvsphere(bm, u_segments=48, v_segments=24, radius=1.0)
        bmesh.ops.scale(bm, vec=(rx, ry, rz), verts=bm.verts)
        bm.to_mesh(mesh)
        bm.free()
        _set_smooth(mesh, True)
        return mesh

    if kind == VISUAL_CYLINDER:
        r = max(p0, 1e-4)
        length = max(p1, 1e-4)
        bmesh.ops.create_cone(
            bm, cap_ends=True, cap_tris=False, segments=48, radius1=r, radius2=r, depth=length
        )
        # Keep side faces smooth and cap faces crisp/flat
        for f in bm.faces:
            f.smooth = abs(f.normal.z) < 0.7
        bm.to_mesh(mesh)
        bm.free()
        return mesh

    if kind == VISUAL_CONE:
        r = max(p0, 1e-4)
        length = max(p1, 1e-4)
        bmesh.ops.create_cone(
            bm, cap_ends=True, cap_tris=False, segments=48, radius1=r, radius2=0.0, depth=length
        )
        for f in bm.faces:
            f.smooth = f.normal.z > -0.7
        bm.to_mesh(mesh)
        bm.free()
        return mesh

    if kind == VISUAL_CAPSULE:
        r = max(p0, 1e-4)
        length = max(p1, 1e-4)
        half = length * 0.5
        # Build UV sphere and stretch top/bottom hemispheres apart by half-length
        bmesh.ops.create_uvsphere(bm, u_segments=48, v_segments=24, radius=r)
        for v in bm.verts:
            if v.co.z > 1e-5:
                v.co.z += half
            elif v.co.z < -1e-5:
                v.co.z -= half
        bm.to_mesh(mesh)
        bm.free()
        _set_smooth(mesh, True)
        return mesh

    if kind == VISUAL_PLANE:
        sx, sy = max(p0, 1e-2), max(p1, 1e-2)
        hx, hy = sx * 0.5, sy * 0.5
        v0 = bm.verts.new((-hx, -hy, 0.0))
        v1 = bm.verts.new((hx, -hy, 0.0))
        v2 = bm.verts.new((hx, hy, 0.0))
        v3 = bm.verts.new((-hx, hy, 0.0))
        face = bm.faces.new((v0, v1, v2, v3))
        uv_layer = bm.loops.layers.uv.verify()
        uvs = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
        for loop, uv in zip(face.loops, uvs):
            loop[uv_layer].uv = uv
        bm.to_mesh(mesh)
        bm.free()
        _set_smooth(mesh, False)
        return mesh

    bm.free()
    return mesh


def _eval_world_matrix(obj) -> Matrix:
    """Compute exact object-to-world matrix by walking parents (works even in excluded collections)."""
    chain = []
    cur = obj
    while cur is not None:
        chain.append(cur)
        cur = cur.parent
    m = Matrix.Identity(4)
    for node in reversed(chain):
        m = m @ node.matrix_local
    return m


def _tri_overlap_2d(
    t1: tuple[tuple[float, float], tuple[float, float], tuple[float, float]],
    t2: tuple[tuple[float, float], tuple[float, float], tuple[float, float]],
    eps: float = 0.003,
) -> bool:
    """Strict 2D interior triangle-triangle overlap test via Separating Axis Theorem."""
    for tri in (t1, t2):
        for k in range(3):
            x0, y0 = tri[k]
            x1, y1 = tri[(k + 1) % 3]
            dx, dy = x1 - x0, y1 - y0
            length = math.hypot(dx, dy)
            if length < 1e-7:
                return False
            nx, ny = -dy / length, dx / length
            p1 = [nx * p[0] + ny * p[1] for p in t1]
            p2 = [nx * p[0] + ny * p[1] for p in t2]
            if max(p1) <= min(p2) + eps or max(p2) <= min(p1) + eps:
                return False
    return True


def _patch_tris_2d(patch, mw: Matrix, u_ax: Vector, v_ax: Vector) -> list[tuple[tuple[float, float], tuple[float, float], tuple[float, float]]]:
    tris = []
    for f in patch["faces"]:
        pts = []
        for v in f.verts:
            vw = mw @ v.co
            pts.append((u_ax.dot(vw), v_ax.dot(vw)))
        for k in range(1, len(pts) - 1):
            tris.append((pts[0], pts[k], pts[k + 1]))
    return tris


def _patches_strictly_overlap(pi, pj, mw: Matrix, u_ax: Vector, v_ax: Vector) -> bool:
    tris_i = _patch_tris_2d(pi, mw, u_ax, v_ax)
    tris_j = _patch_tris_2d(pj, mw, u_ax, v_ax)
    for ti in tris_i:
        for tj in tris_j:
            if _tri_overlap_2d(ti, tj):
                return True
    return False


def _sanitize_coplanar_overlaps(objs: list[object], label: str) -> None:
    """Fix coplanar overlapping modular panels in imported glTF assets for path tracing.

    Game-engine environments often snap modular wall/floor panels together with partial
    coplanar overlap (e.g., two 6 m wall panels overlapping by 2.5 m within microns on
    the same plane). Rasterizers hide this in the depth buffer, whereas Cycles shoots
    shadow/bounce rays from the surface and self-shadows across the overlapping diagonal.

    This pass runs once per cached glTF import and only modifies meshes where strict
    coplanar interior overlaps (> 2 cm x 2 cm within 0.25 mm plane distance) are detected:
      - Exact duplicate coplanar patches are deleted.
      - Partially overlapping coplanar patches are graph-colored and separated by 0.8 mm
        along their inward normal (preferring standalone flat panels so corners stay put).
    """
    total_removed_faces = 0
    total_offset_patches = 0

    for obj in objs:
        if obj.type != "MESH" or obj.data is None:
            continue
        mesh = obj.data
        if len(mesh.polygons) < 2 or len(mesh.polygons) > 10000:
            continue

        mw = _eval_world_matrix(obj)
        mw3 = mw.to_3x3()
        if abs(mw3.determinant()) < 1e-12:
            continue
        inv_mw3 = mw3.inverted()
        norm_mat = inv_mw3.transposed()

        bm = bmesh.new()
        bm.from_mesh(mesh)
        bm.faces.ensure_lookup_table()
        bm.normal_update()

        # Precompute world-space face normals
        face_normals_w: list[Vector] = []
        for f in bm.faces:
            nw = norm_mat @ f.normal
            if nw.length_squared > 1e-12:
                nw.normalize()
            else:
                nw = Vector((0.0, 0.0, 1.0))
            face_normals_w.append(nw)

        # 1. Group connected coplanar faces into planar patches
        visited = [False] * len(bm.faces)
        patches = []
        for f in bm.faces:
            if visited[f.index]:
                continue
            n0 = face_normals_w[f.index]
            stack = [f]
            visited[f.index] = True
            p_faces = []
            while stack:
                cur = stack.pop()
                p_faces.append(cur)
                for e in cur.edges:
                    for nf in e.link_faces:
                        if not visited[nf.index] and face_normals_w[nf.index].dot(n0) > 0.999:
                            visited[nf.index] = True
                            stack.append(nf)

            p_verts = {v for pf in p_faces for v in pf.verts}
            p_face_set = set(p_faces)
            is_standalone = all(
                all(lf in p_face_set for lf in v.link_faces) for v in p_verts
            )
            nw_sum = Vector((0.0, 0.0, 0.0))
            for pf in p_faces:
                nw_sum += face_normals_w[pf.index]
            nw = nw_sum.normalized() if nw_sum.length_squared > 1e-12 else n0
            verts_w = [mw @ v.co for v in p_verts]
            d_w = sum(nw.dot(vw) for vw in verts_w) / max(1, len(verts_w))

            patches.append(
                {
                    "faces": p_faces,
                    "verts": p_verts,
                    "verts_w": verts_w,
                    "nw": nw,
                    "d_w": d_w,
                    "standalone": is_standalone,
                }
            )

        if len(patches) < 2:
            bm.free()
            continue

        # 2. Spatial bucket by normal & plane distance (1 mm bins)
        buckets: dict[tuple[int, int, int, int], list[int]] = {}
        for idx, p in enumerate(patches):
            nw = p["nw"]
            key = (
                int(round(nw.x * 10.0)),
                int(round(nw.y * 10.0)),
                int(round(nw.z * 10.0)),
                int(round(p["d_w"] / 0.001)),
            )
            buckets.setdefault(key, []).append(idx)

        # 3. Detect strict coplanar interior overlaps (< 0.25 mm apart, > 2 cm x 2 cm overlap)
        duplicate_patches: set[int] = set()
        conflicts: dict[int, set[int]] = {}
        checked_pairs: set[tuple[int, int]] = set()

        for (nxk, nyk, nzk, dk), idxs in buckets.items():
            cand = list(idxs)
            for neighbor_d in (dk - 1, dk + 1):
                cand.extend(buckets.get((nxk, nyk, nzk, neighbor_d), ()))
            if len(cand) < 2:
                continue

            for a_pos in range(len(idxs)):
                i = idxs[a_pos]
                if i in duplicate_patches:
                    continue
                pi = patches[i]
                ni = pi["nw"]
                ref = Vector((0.0, 0.0, 1.0)) if abs(ni.z) < 0.9 else Vector((1.0, 0.0, 0.0))
                u_ax = ni.cross(ref).normalized()
                v_ax = ni.cross(u_ax).normalized()
                ui = [u_ax.dot(vw) for vw in pi["verts_w"]]
                vi = [v_ax.dot(vw) for vw in pi["verts_w"]]
                ui_min, ui_max = min(ui), max(ui)
                vi_min, vi_max = min(vi), max(vi)
                if (ui_max - ui_min) < 0.02 or (vi_max - vi_min) < 0.02:
                    continue

                for j in cand:
                    if j <= i or j in duplicate_patches:
                        continue
                    pair = (i, j)
                    if pair in checked_pairs:
                        continue
                    checked_pairs.add(pair)

                    pj = patches[j]
                    if ni.dot(pj["nw"]) < 0.9995:
                        continue
                    if abs(pi["d_w"] - pj["d_w"]) > 0.00025:
                        continue

                    uj = [u_ax.dot(vw) for vw in pj["verts_w"]]
                    vj = [v_ax.dot(vw) for vw in pj["verts_w"]]
                    uj_min, uj_max = min(uj), max(uj)
                    vj_min, vj_max = min(vj), max(vj)

                    ov_u = min(ui_max, uj_max) - max(ui_min, uj_min)
                    ov_v = min(vi_max, vj_max) - max(vi_min, vj_min)
                    if ov_u <= 0.02 or ov_v <= 0.02:
                        continue

                    # Confirm genuine 2D triangle interior overlap (avoids false positives on
                    # UV-split rings, brackets, or L-shaped flat CAD end-caps)
                    if not _patches_strictly_overlap(pi, pj, mw, u_ax, v_ax):
                        continue

                    # Exact duplicate coplanar patch? Remove the later one cleanly.
                    same_bounds = (
                        abs(ui_min - uj_min) < 0.002
                        and abs(ui_max - uj_max) < 0.002
                        and abs(vi_min - vj_min) < 0.002
                        and abs(vi_max - vj_max) < 0.002
                        and len(pi["faces"]) == len(pj["faces"])
                    )
                    if same_bounds:
                        duplicate_patches.add(j)
                    else:
                        conflicts.setdefault(i, set()).add(j)
                        conflicts.setdefault(j, set()).add(i)

        if not duplicate_patches and not conflicts:
            bm.free()
            continue

        # 4. Graph-color overlapping patches (keep corner/non-standalone patches at layer 0)
        colors: dict[int, int] = {}
        order = sorted(
            conflicts.keys(),
            key=lambda idx: (1 if patches[idx]["standalone"] else 0, idx),
        )
        for idx in order:
            used = {colors[nb] for nb in conflicts[idx] if nb in colors}
            c = 0
            while c in used:
                c += 1
            colors[idx] = c

        # 5. Apply 0.8 mm inward-normal separation for layer > 0 patches
        for idx, layer in colors.items():
            if layer == 0 or idx in duplicate_patches:
                continue
            p = patches[idx]
            delta_w = -p["nw"] * (layer * 0.0008)
            delta_local = inv_mw3 @ delta_w
            if p["standalone"]:
                for v in p["verts"]:
                    v.co += delta_local
            else:
                p_face_set = set(p["faces"])
                boundary_edges = [
                    e
                    for f in p["faces"]
                    for e in f.edges
                    if any(lf not in p_face_set for lf in e.link_faces)
                ]
                if boundary_edges:
                    bmesh.ops.split_edges(bm, edges=list(set(boundary_edges)))
                new_verts = {v for f in p["faces"] for v in f.verts}
                for v in new_verts:
                    v.co += delta_local
            total_offset_patches += 1

        # 6. Delete exact duplicate patches
        if duplicate_patches:
            dup_faces = [f for idx in duplicate_patches for f in patches[idx]["faces"]]
            total_removed_faces += len(dup_faces)
            bmesh.ops.delete(bm, geom=dup_faces, context="FACES")

        bm.normal_update()
        bm.to_mesh(mesh)
        bm.free()
        mesh.update()

    if total_removed_faces > 0 or total_offset_patches > 0:
        print(
            f"[gz_blender] Sanitized coplanar overlaps in '{label}' "
            f"(removed {total_removed_faces} duplicate faces, offset {total_offset_patches} overlapping patches)",
            flush=True,
        )


class SceneSynchronizer:
    """Translates Gazebo ExternalRendererSystem Zenoh packets into a live Blender scene."""

    def __init__(self, scene_collection) -> None:
        self.collection = scene_collection

        # Hidden template collection for cached native glTF imports (enables VRAM-free instancing)
        self.gltf_cache_coll = bpy.data.collections.new("_GzGltfCache")
        bpy.context.scene.collection.children.link(self.gltf_cache_coll)
        layer_coll = bpy.context.view_layer.layer_collection.children.get(self.gltf_cache_coll.name)
        if layer_coll is not None:
            layer_coll.exclude = True

        self.parent_objects: dict[int, object] = {}  # parent_entity_id -> Empty
        self.visual_roots: dict[int, object] = {}  # visual_id -> Object
        self.streamed_textures: dict[int, object] = {}  # texture_id -> bpy.types.Image
        self.streamed_meshes: dict[int, object] = {}  # mesh_id -> bpy.types.Mesh
        self.gltf_collections: dict[str, object] = {}  # resolved_uri -> template Collection

        self.shm_msgs = 0
        self.raw_msgs = 0

    @property
    def entity_count(self) -> int:
        return len(self.visual_roots)

    def clear_world(self) -> None:
        # Remove all objects inside the dynamic Gazebo scene collection
        for obj in list(self.collection.objects):
            bpy.data.objects.remove(obj, do_unlink=True)
        self.parent_objects.clear()
        self.visual_roots.clear()

    def _get_or_create_parent(self, parent_entity_id: int, parent_pose: Pose):
        obj = self.parent_objects.get(parent_entity_id)
        if obj is None:
            obj = bpy.data.objects.new(name=f"gz_ent_{parent_entity_id}", object_data=None)
            self.collection.objects.link(obj)
            self.parent_objects[parent_entity_id] = obj
        _apply_pose(obj, parent_pose)
        return obj

    def ingest_texture(self, td: TextureData) -> None:
        if td.texture_id in self.streamed_textures:
            return
        img = bpy.data.images.new(
            name=f"gz_tex_{td.texture_id}",
            width=td.width,
            height=td.height,
            alpha=True,
            float_buffer=False,
        )
        # IMPORTANT: In Blender 5.0, setting colorspace_settings.name on a newly generated
        # image invalidates/zeroes its pixel buffer, so set colorspace BEFORE foreach_set!
        img.colorspace_settings.name = "sRGB" if td.fmt == 0 else "Non-Color"
        # Flip Y from top-down gz-common to bottom-up Blender convention & normalize to float32
        rgba_f32 = np.ascontiguousarray(td.pixels[::-1], dtype=np.float32) * (1.0 / 255.0)
        img.pixels.foreach_set(rgba_f32.ravel())
        img.update()
        img.pack()
        self.streamed_textures[td.texture_id] = img

    def ingest_mesh(self, md: MeshData) -> None:
        if md.mesh_id in self.streamed_meshes:
            return

        tri_count = md.index_count // 3
        if tri_count <= 0 or md.vertex_count == 0:
            return

        # Filter out any out-of-range or degenerate triangles (a==b, b==c, or a==c)
        # before passing raw loops to Blender, which can otherwise crash during mesh update/normals.
        tris = md.indices[: tri_count * 3].reshape((tri_count, 3))
        valid_mask = (
            (tris[:, 0] < md.vertex_count)
            & (tris[:, 1] < md.vertex_count)
            & (tris[:, 2] < md.vertex_count)
            & (tris[:, 0] != tris[:, 1])
            & (tris[:, 1] != tris[:, 2])
            & (tris[:, 2] != tris[:, 0])
        )
        tris = tris[valid_mask]
        tri_count = len(tris)
        if tri_count == 0:
            return
        usable_indices = np.ascontiguousarray(tris.ravel(), dtype=np.int32)

        # Sanitize NaN/Inf positions
        positions = np.nan_to_num(md.positions, nan=0.0, posinf=0.0, neginf=0.0)

        mesh = bpy.data.meshes.new(name=f"gz_mesh_{md.mesh_id}")
        mesh.vertices.add(md.vertex_count)
        mesh.vertices.foreach_set("co", np.ascontiguousarray(positions, dtype=np.float32).ravel())

        mesh.loops.add(tri_count * 3)
        mesh.loops.foreach_set("vertex_index", usable_indices)

        mesh.polygons.add(tri_count)
        loop_starts = np.arange(0, tri_count * 3, 3, dtype=np.int32)
        loop_totals = np.full(tri_count, 3, dtype=np.int32)
        mesh.polygons.foreach_set("loop_start", loop_starts)
        mesh.polygons.foreach_set("loop_total", loop_totals)
        mesh.polygons.foreach_set("use_smooth", np.ones(tri_count, dtype=bool))

        if md.uvs is not None and len(md.uvs) == md.vertex_count:
            uv_layer = mesh.uv_layers.new(name="UVMap")
            uvs_flipped = np.nan_to_num(md.uvs, nan=0.0, posinf=0.0, neginf=0.0).copy()
            uvs_flipped[:, 1] = 1.0 - uvs_flipped[:, 1]
            loop_uvs = np.ascontiguousarray(uvs_flipped[usable_indices], dtype=np.float32)
            uv_layer.data.foreach_set("uv", loop_uvs.ravel())

        mesh.validate(verbose=False, clean_customdata=False)
        mesh.update(calc_edges=True)

        if md.normals is not None and len(md.normals) == len(mesh.vertices):
            norms = np.nan_to_num(md.normals, nan=0.0, posinf=0.0, neginf=0.0).copy()
            lens = np.linalg.norm(norms, axis=1, keepdims=True)
            valid = lens[:, 0] > 1e-6
            norms[valid] /= lens[valid]
            norms[~valid] = (0.0, 0.0, 1.0)
            try:
                mesh.normals_split_custom_set_from_vertices(norms.tolist())
            except Exception:
                pass

        self.streamed_meshes[md.mesh_id] = mesh

    def _build_principled_material(
        self, name: str, mat_wire: PbrMaterial, double_sided: bool, alpha_mode: int
    ):
        mat = bpy.data.materials.new(name=name)
        mat.use_nodes = True
        mat.use_backface_culling = not double_sided

        nt = mat.node_tree
        nt.nodes.clear()

        out = nt.nodes.new("ShaderNodeOutputMaterial")
        out.location = (620, 0)
        bsdf = nt.nodes.new("ShaderNodeBsdfPrincipled")
        bsdf.location = (260, 0)
        nt.links.new(bsdf.outputs["BSDF"], out.inputs["Surface"])

        bc = mat_wire.base_color
        bsdf.inputs["Base Color"].default_value = (bc[0], bc[1], bc[2], 1.0)
        bsdf.inputs["Metallic"].default_value = max(0.0, min(1.0, mat_wire.metallic))
        bsdf.inputs["Roughness"].default_value = max(0.04, min(1.0, mat_wire.roughness))

        base_color_socket = None
        base_img = self.streamed_textures.get(mat_wire.base_color_tex) if mat_wire.base_color_tex else None
        if base_img is not None:
            tex_bc = nt.nodes.new("ShaderNodeTexImage")
            tex_bc.location = (-360, 260)
            tex_bc.image = base_img
            # Tint albedo map by SDF base_color factor if non-white
            # NOTE: ShaderNodeMix(data_type='RGBA') uses inputs[0]=Factor, inputs[6]=A(Color),
            # inputs[7]=B(Color), outputs[2]=Result(Color). Using string names would grab FLOAT sockets!
            if abs(bc[0] - 1.0) > 0.01 or abs(bc[1] - 1.0) > 0.01 or abs(bc[2] - 1.0) > 0.01:
                mix_tint = nt.nodes.new("ShaderNodeMix")
                mix_tint.data_type = "RGBA"
                mix_tint.blend_type = "MULTIPLY"
                mix_tint.location = (-80, 260)
                mix_tint.inputs[0].default_value = 1.0
                mix_tint.inputs[7].default_value = (bc[0], bc[1], bc[2], 1.0)
                nt.links.new(tex_bc.outputs["Color"], mix_tint.inputs[6])
                base_color_socket = mix_tint.outputs[2]
            else:
                base_color_socket = tex_bc.outputs["Color"]

            if alpha_mode in (ALPHA_MASK, ALPHA_BLEND):
                nt.links.new(tex_bc.outputs["Alpha"], bsdf.inputs["Alpha"])

        # Multiply baked Ambient Occlusion texture gently into Base Color if provided
        occ_img = self.streamed_textures.get(mat_wire.occlusion_tex) if mat_wire.occlusion_tex else None
        if occ_img is not None:
            tex_occ = nt.nodes.new("ShaderNodeTexImage")
            tex_occ.location = (-360, 480)
            tex_occ.image = occ_img
            mix_ao = nt.nodes.new("ShaderNodeMix")
            mix_ao.data_type = "RGBA"
            mix_ao.blend_type = "MULTIPLY"
            mix_ao.location = (60, 320)
            mix_ao.inputs[0].default_value = 0.75
            if base_color_socket is not None:
                nt.links.new(base_color_socket, mix_ao.inputs[6])
            else:
                mix_ao.inputs[6].default_value = (bc[0], bc[1], bc[2], 1.0)
            nt.links.new(tex_occ.outputs["Color"], mix_ao.inputs[7])
            base_color_socket = mix_ao.outputs[2]

        if base_color_socket is not None:
            nt.links.new(base_color_socket, bsdf.inputs["Base Color"])

        # Packed Metallic-Roughness texture (G = Roughness, B = Metallic per glTF/ExternalRendererSystem)
        mr_img = (
            self.streamed_textures.get(mat_wire.metallic_roughness_tex)
            if mat_wire.metallic_roughness_tex
            else None
        )
        if mr_img is not None:
            tex_mr = nt.nodes.new("ShaderNodeTexImage")
            tex_mr.location = (-400, -20)
            tex_mr.image = mr_img
            sep = nt.nodes.new("ShaderNodeSeparateColor")
            sep.location = (-160, -20)
            nt.links.new(tex_mr.outputs["Color"], sep.inputs["Color"])

            rough_mul = nt.nodes.new("ShaderNodeMath")
            rough_mul.operation = "MULTIPLY"
            rough_mul.use_clamp = True
            rough_mul.location = (40, -10)
            rough_mul.inputs[1].default_value = max(0.05, mat_wire.roughness)
            nt.links.new(sep.outputs["Green"], rough_mul.inputs[0])
            nt.links.new(rough_mul.outputs["Value"], bsdf.inputs["Roughness"])

            metal_mul = nt.nodes.new("ShaderNodeMath")
            metal_mul.operation = "MULTIPLY"
            metal_mul.use_clamp = True
            metal_mul.location = (40, -150)
            metal_mul.inputs[1].default_value = max(0.0, min(1.0, mat_wire.metallic))
            nt.links.new(sep.outputs["Blue"], metal_mul.inputs[0])
            nt.links.new(metal_mul.outputs["Value"], bsdf.inputs["Metallic"])

        # Normal map
        norm_img = self.streamed_textures.get(mat_wire.normal_tex) if mat_wire.normal_tex else None
        if norm_img is not None:
            tex_n = nt.nodes.new("ShaderNodeTexImage")
            tex_n.location = (-360, -320)
            tex_n.image = norm_img
            nmap = nt.nodes.new("ShaderNodeNormalMap")
            nmap.location = (-40, -320)
            nt.links.new(tex_n.outputs["Color"], nmap.inputs["Color"])
            nt.links.new(nmap.outputs["Normal"], bsdf.inputs["Normal"])

        # Emissive
        emis_img = self.streamed_textures.get(mat_wire.emissive_tex) if mat_wire.emissive_tex else None
        em = mat_wire.emissive
        has_em_color = (em[0] > 0.005 or em[1] > 0.005 or em[2] > 0.005)
        if emis_img is not None:
            tex_em = nt.nodes.new("ShaderNodeTexImage")
            tex_em.location = (-360, -560)
            tex_em.image = emis_img
            nt.links.new(tex_em.outputs["Color"], bsdf.inputs["Emission Color"])
            bsdf.inputs["Emission Strength"].default_value = 1.0
        elif has_em_color:
            bsdf.inputs["Emission Color"].default_value = (em[0], em[1], em[2], 1.0)
            bsdf.inputs["Emission Strength"].default_value = 1.0

        # Scalar transparency
        if alpha_mode == ALPHA_BLEND or bc[3] < 0.99:
            if base_img is None:
                bsdf.inputs["Alpha"].default_value = max(0.0, min(1.0, bc[3]))

        return mat

    def _get_or_import_gltf_collection(self, uri: str):
        if uri in self.gltf_collections:
            return self.gltf_collections[uri]

        path = Path(uri)
        if not path.exists():
            print(f"[gz_blender] Missing glTF file: {uri}", flush=True)
            return None

        before = set(bpy.data.objects)
        try:
            bpy.ops.import_scene.gltf(filepath=str(path), loglevel=40)
        except Exception as exc:
            print(f"[gz_blender] Failed to import glTF '{uri}': {exc}", flush=True)
            return None

        added = [obj for obj in bpy.data.objects if obj not in before]
        roots = [obj for obj in added if obj.parent is None]

        # Dedicated excluded template collection for native GPU Collection Instancing!
        # Instancing via collection creates 1 Empty per Gazebo visual even for 3,000-node .glbs.
        tmpl_coll = bpy.data.collections.new(f"_GzGltf_{len(self.gltf_collections)}_{path.stem[:24]}")
        self.gltf_cache_coll.children.link(tmpl_coll)

        # Normalize glTF Y-up -> Gazebo raw orientation once inside the template collection
        # (undoing Blender's automatic Rx(+90 deg) glTF import conversion).
        axis_fix = bpy.data.objects.new(name=f"_axis_fix_{path.stem[:20]}", object_data=None)
        axis_fix.rotation_mode = "XYZ"
        axis_fix.rotation_euler = Euler((-math.pi * 0.5, 0.0, 0.0), "XYZ")
        tmpl_coll.objects.link(axis_fix)

        for obj in added:
            for c in list(obj.users_collection):
                c.objects.unlink(obj)
            tmpl_coll.objects.link(obj)
        for r in roots:
            r.parent = axis_fix
            r.matrix_parent_inverse = Matrix.Identity(4)

        # Ensure world matrices are evaluated before measuring coplanar overlaps in meters
        bpy.context.view_layer.update()
        _sanitize_coplanar_overlaps(added, path.name)

        self.gltf_collections[uri] = tmpl_coll
        print(
            f"[gz_blender] Cached native glTF collection '{path.name}' "
            f"({len(added)} objects, {len(roots)} roots)",
            flush=True,
        )
        return tmpl_coll

    def spawn_visual(self, sv: SpawnVisual) -> None:
        # Remove any existing visual with the same visual_id
        old = self.visual_roots.pop(sv.visual_id, None)
        if old is not None:
            self._remove_subtree(old)

        parent_obj = self._get_or_create_parent(sv.parent_entity_id, sv.parent_pose)

        if sv.kind == VISUAL_GLTF_URI:
            tmpl_coll = self._get_or_import_gltf_collection(sv.uri)
            if tmpl_coll is None:
                return
            inst = bpy.data.objects.new(name=f"gz_gltf_{sv.visual_id}", object_data=None)
            inst.instance_type = "COLLECTION"
            inst.instance_collection = tmpl_coll
            inst.visible_shadow = sv.cast_shadows
            self.collection.objects.link(inst)
            inst.parent = parent_obj
            _apply_pose(inst, sv.local_pose, (sv.params[0], sv.params[1], sv.params[2]))
            self.visual_roots[sv.visual_id] = inst
            return

        if sv.kind == VISUAL_STREAMED_MESH:
            mesh = self.streamed_meshes.get(sv.mesh_id)
            if mesh is None:
                print(
                    f"[gz_blender] SpawnVisual referenced missing mesh_id={sv.mesh_id}", flush=True
                )
                return
            scale = (sv.params[0], sv.params[1], sv.params[2])
        else:
            mesh = _build_primitive_mesh(sv.kind, sv.params, f"gz_prim_{sv.visual_id}")
            scale = (1.0, 1.0, 1.0)

        mat = self._build_principled_material(
            f"gz_mat_{sv.visual_id}",
            sv.material,
            sv.double_sided or (sv.kind == VISUAL_PLANE),
            sv.alpha_mode,
        )

        # Avoid mutating shared streamed mesh materials across instances with different materials:
        # assign material on object slot (link='OBJECT')!
        obj = bpy.data.objects.new(name=f"gz_vis_{sv.visual_id}", object_data=mesh)
        self.collection.objects.link(obj)
        obj.parent = parent_obj
        _apply_pose(obj, sv.local_pose, scale)
        obj.visible_shadow = sv.cast_shadows

        if len(obj.material_slots) == 0:
            obj.data.materials.append(None)
        obj.material_slots[0].link = "OBJECT"
        obj.material_slots[0].material = mat

        self.visual_roots[sv.visual_id] = obj

    def _remove_subtree(self, root_obj) -> None:
        for child in list(root_obj.children):
            self._remove_subtree(child)
        bpy.data.objects.remove(root_obj, do_unlink=True)

    def despawn_entity(self, entity_id: int) -> bool:
        parent = self.parent_objects.pop(entity_id, None)
        if parent is None:
            return False
        # Clean visual_roots referencing children of this parent
        to_del = [vid for vid, vobj in self.visual_roots.items() if vobj.parent == parent]
        for vid in to_del:
            self.visual_roots.pop(vid, None)
        self._remove_subtree(parent)
        return True

    def apply_transforms(self, items: list[tuple[int, Pose]]) -> bool:
        moved = False
        for eid, pose in items:
            obj = self.parent_objects.get(eid)
            if obj is not None:
                _apply_pose(obj, pose)
                moved = True
        return moved
