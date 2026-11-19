use bytemuck::{Pod, Zeroable};

pub const PROTOCOL_MAGIC: u32 = 0x475A4256; // "GZBV"
pub const PROTOCOL_VERSION: u16 = 1;
pub const SWAPCHAIN_SLOTS: usize = 3;

pub const MSG_WORLD_CONFIG: u16 = 1;
pub const MSG_TEXTURE_DATA: u16 = 2;
pub const MSG_MESH_DATA: u16 = 3;
pub const MSG_SPAWN_VISUAL: u16 = 4;
pub const MSG_DESPAWN_ENTITY: u16 = 5;
pub const MSG_LIGHT_UPSERT: u16 = 6;
pub const MSG_TRANSFORM_BATCH: u16 = 7;
pub const MSG_CLEAR_WORLD: u16 = 8;
pub const MSG_SNAPSHOT_REQUEST: u16 = 9;
pub const MSG_GUI_INPUT: u16 = 10;
pub const MSG_FRAME_READY: u16 = 11;
pub const MSG_SLOT_ACQUIRED: u16 = 12;

pub const QUALITY_IBL: u32 = 1 << 0;
pub const QUALITY_SSAO: u32 = 1 << 1;
pub const QUALITY_BLOOM: u32 = 1 << 2;
pub const QUALITY_AGX: u32 = 1 << 3;
pub const QUALITY_SHADOWS: u32 = 1 << 4;
pub const QUALITY_MSAA: u32 = 1 << 5;
pub const QUALITY_DEFAULT: u32 =
    QUALITY_IBL | QUALITY_BLOOM | QUALITY_AGX | QUALITY_SHADOWS | QUALITY_MSAA;

#[repr(C, packed)]
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
pub struct MsgHeader {
    pub magic: u32,
    pub version: u16,
    pub msg_type: u16,
    pub count: u32,
    pub payload_bytes: u32,
}

#[repr(C, packed)]
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
pub struct PoseWire {
    pub px: f32,
    pub py: f32,
    pub pz: f32,
    pub qx: f32,
    pub qy: f32,
    pub qz: f32,
    pub qw: f32,
}

#[repr(C, packed)]
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
pub struct WorldConfigWire {
    pub ambient: [f32; 4],
    pub background: [f32; 4],
    pub has_sky: u8,
    pub has_camera_pose: u8,
    pub _pad: u16,
    pub sky_intensity: f32,
    pub camera_pose: PoseWire,
    pub camera_hfov: f32,
    pub env_map_len: u32,
}

#[repr(C, packed)]
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
pub struct TextureDataWire {
    pub texture_id: u64,
    pub width: u32,
    pub height: u32,
    pub format: u8,
    pub _pad: [u8; 3],
    pub data_len: u32,
}

#[repr(C, packed)]
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
pub struct MeshDataWire {
    pub mesh_id: u64,
    pub vertex_count: u32,
    pub index_count: u32,
    pub has_normals: u8,
    pub has_uvs: u8,
    pub has_tangents: u8,
    pub _pad: u8,
    pub _pad2: u32,
}

#[repr(C, packed)]
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
pub struct PbrMaterialWire {
    pub base_color: [f32; 4],
    pub emissive: [f32; 4],
    pub metallic: f32,
    pub roughness: f32,
    pub alpha_cutoff: f32,
    pub reflectance: f32,
    pub base_color_tex: u64,
    pub normal_tex: u64,
    pub metallic_roughness_tex: u64,
    pub occlusion_tex: u64,
    pub emissive_tex: u64,
}

#[repr(C, packed)]
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
pub struct SpawnVisualWire {
    pub visual_id: u64,
    pub parent_entity_id: u64,
    pub kind: u8,
    pub cast_shadows: u8,
    pub double_sided: u8,
    pub alpha_mode: u8,
    pub uri_len: u32,
    pub parent_pose: PoseWire,
    pub local_pose: PoseWire,
    pub params: [f32; 4],
    pub mesh_id: u64,
    pub material: PbrMaterialWire,
}

#[repr(C, packed)]
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
pub struct DespawnEntityWire {
    pub entity_id: u64,
}

#[repr(C, packed)]
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
pub struct LightUpsertWire {
    pub entity_id: u64,
    pub light_type: u8,
    pub cast_shadows: u8,
    pub _pad: u16,
    pub intensity: f32,
    pub pose: PoseWire,
    pub direction: [f32; 3],
    pub range: f32,
    pub color: [f32; 4],
    pub inner_angle: f32,
    pub outer_angle: f32,
}

#[repr(C, packed)]
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
pub struct TransformBatchHeaderWire {
    pub sim_time: f64,
}

#[repr(C, packed)]
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
pub struct TransformItemWire {
    pub entity_id: u64,
    pub pose: PoseWire,
    pub _pad: u32,
}

#[repr(C, packed)]
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
pub struct SnapshotRequestWire {
    pub client_id: u64,
}

#[repr(C, packed)]
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
pub struct GuiInputWire {
    pub kind: u8,
    pub _pad: [u8; 3],
    pub button: u32,
    pub buttons_mask: u32,
    pub modifiers: u32,
    pub key_code: u32,
    pub width: u32,
    pub height: u32,
    pub quality_flags: u32,
    pub x: f32,
    pub y: f32,
    pub dx: f32,
    pub dy: f32,
    pub dpr: f32,
    pub camera_hfov: f32,
    pub camera_pose: PoseWire,
    pub path_len: u32,
}

#[repr(C, packed)]
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
pub struct FrameReadyWire {
    pub generation: u32,
    pub slot: u32,
    pub width: u32,
    pub height: u32,
    pub frame_id: u64,
    pub render_ms: f32,
    pub fps: f32,
    pub entity_count: u32,
    pub quality_flags: u32,
}

#[repr(C, packed)]
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
pub struct SlotAcquiredWire {
    pub generation: u32,
    pub active_slot: u32,
}

// Compile-time size verification against common/protocol.hh
const _: () = {
    assert!(std::mem::size_of::<MsgHeader>() == 16);
    assert!(std::mem::size_of::<PoseWire>() == 28);
    assert!(std::mem::size_of::<WorldConfigWire>() == 76);
    assert!(std::mem::size_of::<TextureDataWire>() == 24);
    assert!(std::mem::size_of::<MeshDataWire>() == 24);
    assert!(std::mem::size_of::<PbrMaterialWire>() == 88);
    assert!(std::mem::size_of::<SpawnVisualWire>() == 192);
    assert!(std::mem::size_of::<DespawnEntityWire>() == 8);
    assert!(std::mem::size_of::<LightUpsertWire>() == 84);
    assert!(std::mem::size_of::<TransformBatchHeaderWire>() == 8);
    assert!(std::mem::size_of::<TransformItemWire>() == 40);
    assert!(std::mem::size_of::<SnapshotRequestWire>() == 8);
    assert!(std::mem::size_of::<GuiInputWire>() == 88);
    assert!(std::mem::size_of::<FrameReadyWire>() == 40);
    assert!(std::mem::size_of::<SlotAcquiredWire>() == 8);
};

pub fn parse_header(bytes: &[u8]) -> Option<(MsgHeader, &[u8])> {
    if bytes.len() < std::mem::size_of::<MsgHeader>() {
        return None;
    }
    let hdr: MsgHeader = bytemuck::pod_read_unaligned(&bytes[..std::mem::size_of::<MsgHeader>()]);
    if hdr.magic != PROTOCOL_MAGIC || hdr.version != PROTOCOL_VERSION {
        return None;
    }
    let payload = &bytes[std::mem::size_of::<MsgHeader>()..];
    if payload.len() < hdr.payload_bytes as usize {
        return None;
    }
    Some((hdr, &payload[..hdr.payload_bytes as usize]))
}

pub fn encode_packet<T: Pod>(msg_type: u16, count: u32, body: &T) -> Vec<u8> {
    let body_bytes = bytemuck::bytes_of(body);
    let hdr = MsgHeader {
        magic: PROTOCOL_MAGIC,
        version: PROTOCOL_VERSION,
        msg_type,
        count,
        payload_bytes: body_bytes.len() as u32,
    };
    let mut out = Vec::with_capacity(std::mem::size_of::<MsgHeader>() + body_bytes.len());
    out.extend_from_slice(bytemuck::bytes_of(&hdr));
    out.extend_from_slice(body_bytes);
    out
}

pub fn read_pod<T: Pod>(bytes: &[u8], offset: usize) -> Option<(T, usize)> {
    let end = offset.checked_add(std::mem::size_of::<T>())?;
    if end > bytes.len() {
        return None;
    }
    Some((bytemuck::pod_read_unaligned(&bytes[offset..end]), end))
}

pub fn read_f32_vec(bytes: &[u8], offset: usize, count: usize) -> Option<(Vec<f32>, usize)> {
    let byte_len = count.checked_mul(4)?;
    let end = offset.checked_add(byte_len)?;
    if end > bytes.len() {
        return None;
    }
    let slice = &bytes[offset..end];
    let mut out = vec![0.0f32; count];
    bytemuck::cast_slice_mut::<f32, u8>(&mut out).copy_from_slice(slice);
    Some((out, end))
}

pub fn read_u32_vec(bytes: &[u8], offset: usize, count: usize) -> Option<(Vec<u32>, usize)> {
    let byte_len = count.checked_mul(4)?;
    let end = offset.checked_add(byte_len)?;
    if end > bytes.len() {
        return None;
    }
    let slice = &bytes[offset..end];
    let mut out = vec![0u32; count];
    bytemuck::cast_slice_mut::<u32, u8>(&mut out).copy_from_slice(slice);
    Some((out, end))
}
