#ifndef GZ_EXTERNAL_RENDERER_PROTOCOL_HH_
#define GZ_EXTERNAL_RENDERER_PROTOCOL_HH_

#include <cstdint>
#include <cstring>
#include <string>
#include <string_view>
#include <vector>

namespace gz::ext_render
{
static constexpr uint32_t kProtocolMagic = 0x475A4256u;  // "GZBV"
static constexpr uint16_t kProtocolVersion = 1u;
static constexpr uint32_t kSwapchainSlots = 3u;

enum class MsgType : uint16_t
{
  WorldConfig = 1,
  TextureData = 2,
  MeshData = 3,
  SpawnVisual = 4,
  DespawnEntity = 5,
  LightUpsert = 6,
  TransformBatch = 7,
  ClearWorld = 8,
  SnapshotRequest = 9,
  GuiInput = 10,
  FrameReady = 11,
  SlotAcquired = 12,
};

enum class TextureFormat : uint8_t
{
  Rgba8Srgb = 0,
  Rgba8Unorm = 1,
};

enum class VisualKind : uint8_t
{
  Box = 0,
  Sphere = 1,
  Cylinder = 2,
  Capsule = 3,
  Plane = 4,
  StreamedMesh = 5,
  GltfUri = 6,
  Cone = 7,
  Ellipsoid = 8,
};

enum class AlphaMode : uint8_t
{
  Opaque = 0,
  Mask = 1,
  Blend = 2,
};

enum class LightType : uint8_t
{
  Directional = 0,
  Point = 1,
  Spot = 2,
  Rect = 3,
};

enum class RendererBackend : uint8_t
{
  Bevy = 0,
  Blender = 1,
};

enum class InputKind : uint8_t
{
  MouseDown = 0,
  MouseUp = 1,
  MouseMove = 2,
  Wheel = 3,
  KeyDown = 4,
  KeyUp = 5,
  Resize = 6,
  SetCameraPose = 7,
  QualityConfig = 8,
  SaveScreenshot = 9,
  ResetCamera = 10,
  Ping = 11,
  SaveScene = 12,
};

// Bitmask flags for Bevy rendering quality toggles
enum QualityFlags : uint32_t
{
  kQualityIbl = 1u << 0,         // Procedural atmosphere sky / IBL reflections
  kQualitySsao = 1u << 1,        // Screen-space ambient occlusion + TAA
  kQualityBloom = 1u << 2,       // HDR natural bloom
  kQualityAgx = 1u << 3,         // AgX filmic tonemapping (off = linear/Reinhard)
  kQualityShadows = 1u << 4,     // Dynamic shadow maps
  kQualityMsaa = 1u << 5,        // 4x MSAA (used when SSAO/TAA is off)
  kQualityDefault = kQualityIbl | kQualityBloom | kQualityAgx | kQualityShadows | kQualityMsaa,
};

// Bitmask flags for Blender 5.0 photorealistic rendering toggles
enum BlenderQualityFlags : uint32_t
{
  kBlenderCycles = 1u << 0,       // Cycles Metal GPU path tracing (off = EEVEE Metal)
  kBlenderIbl = 1u << 1,          // Physical Nishita sky / HDRI environment lighting
  kBlenderRtGi = 1u << 2,         // EEVEE Raytracing GI+Reflections / Cycles multi-bounce GI
  kBlenderDenoise = 1u << 3,      // OpenImageDenoise (GPU RGB+Albedo+Normal)
  kBlenderAgx = 1u << 4,          // AgX filmic color management + Medium High Contrast
  kBlenderShadows = 1u << 5,      // Physical soft shadows
  kBlenderHighQuality = 1u << 6,  // High-sample photoreal refinement budget
  kBlenderDefault = kBlenderIbl | kBlenderRtGi | kBlenderDenoise | kBlenderAgx | kBlenderShadows,
};

#pragma pack(push, 1)

struct MsgHeader
{
  uint32_t magic = kProtocolMagic;
  uint16_t version = kProtocolVersion;
  uint16_t msg_type = 0;
  uint32_t count = 0;
  uint32_t payload_bytes = 0;
};
static_assert(sizeof(MsgHeader) == 16);

struct PoseWire
{
  float px = 0.0f, py = 0.0f, pz = 0.0f;
  float qx = 0.0f, qy = 0.0f, qz = 0.0f, qw = 1.0f;
};
static_assert(sizeof(PoseWire) == 28);

struct WorldConfigWire
{
  float ambient[4] = {0.4f, 0.4f, 0.4f, 1.0f};
  float background[4] = {0.2f, 0.2f, 0.2f, 1.0f};
  uint8_t has_sky = 0;
  uint8_t has_camera_pose = 0;
  uint16_t _pad = 0;
  float sky_intensity = 1.0f;
  PoseWire camera_pose{};
  float camera_hfov = 1.0472f;
  uint32_t env_map_len = 0;
  // Followed by env_map_len UTF-8 bytes
};
static_assert(sizeof(WorldConfigWire) == 76);

struct TextureDataWire
{
  uint64_t texture_id = 0;
  uint32_t width = 0;
  uint32_t height = 0;
  uint8_t format = 0;
  uint8_t _pad[3] = {0, 0, 0};
  uint32_t data_len = 0;
  // Followed by data_len bytes (RGBA8)
};
static_assert(sizeof(TextureDataWire) == 24);

struct MeshDataWire
{
  uint64_t mesh_id = 0;
  uint32_t vertex_count = 0;
  uint32_t index_count = 0;
  uint8_t has_normals = 0;
  uint8_t has_uvs = 0;
  uint8_t has_tangents = 0;
  uint8_t _pad = 0;
  uint32_t _pad2 = 0;
  // Followed by contiguous aligned f32/u32 arrays:
  //   positions [vertex_count * 3] f32
  //   normals   [vertex_count * 3] f32 (if has_normals)
  //   uvs       [vertex_count * 2] f32 (if has_uvs)
  //   tangents  [vertex_count * 4] f32 (if has_tangents)
  //   indices   [index_count] u32
};
static_assert(sizeof(MeshDataWire) == 24);

struct PbrMaterialWire
{
  float base_color[4] = {0.8f, 0.8f, 0.8f, 1.0f};
  float emissive[4] = {0.0f, 0.0f, 0.0f, 1.0f};
  float metallic = 0.0f;
  float roughness = 0.5f;
  float alpha_cutoff = 0.5f;
  float reflectance = 0.5f;
  uint64_t base_color_tex = 0;
  uint64_t normal_tex = 0;
  uint64_t metallic_roughness_tex = 0;
  uint64_t occlusion_tex = 0;
  uint64_t emissive_tex = 0;
};
static_assert(sizeof(PbrMaterialWire) == 88);

struct SpawnVisualWire
{
  uint64_t visual_id = 0;
  uint64_t parent_entity_id = 0;
  uint8_t kind = 0;
  uint8_t cast_shadows = 1;
  uint8_t double_sided = 0;
  uint8_t alpha_mode = 0;
  uint32_t uri_len = 0;
  PoseWire parent_pose{};
  PoseWire local_pose{};
  float params[4] = {1.0f, 1.0f, 1.0f, 0.0f};
  uint64_t mesh_id = 0;
  PbrMaterialWire material{};
  // Followed by uri_len UTF-8 bytes
};
static_assert(sizeof(SpawnVisualWire) == 192);

struct DespawnEntityWire
{
  uint64_t entity_id = 0;
};
static_assert(sizeof(DespawnEntityWire) == 8);

struct LightUpsertWire
{
  uint64_t entity_id = 0;
  uint8_t light_type = 0;
  uint8_t cast_shadows = 1;
  uint16_t spread_mrad = 0;  // Area light beam spread in milliradians (0 => full 180 deg / pi)
  float intensity = 1.0f;
  PoseWire pose{};
  float direction[3] = {0.0f, 0.0f, -1.0f};
  float range = 20.0f;
  float color[4] = {1.0f, 1.0f, 1.0f, 1.0f};
  float inner_angle = 0.5f;  // For LightType::Rect: width along local X (meters)
  float outer_angle = 0.8f;  // For LightType::Rect: height along local Y (meters)
};
static_assert(sizeof(LightUpsertWire) == 84);

struct TransformBatchHeaderWire
{
  double sim_time = 0.0;
};
static_assert(sizeof(TransformBatchHeaderWire) == 8);

struct TransformItemWire
{
  uint64_t entity_id = 0;
  PoseWire pose{};
  uint32_t _pad = 0;
};
static_assert(sizeof(TransformItemWire) == 40);

struct SnapshotRequestWire
{
  uint64_t client_id = 0;
};
static_assert(sizeof(SnapshotRequestWire) == 8);

struct GuiInputWire
{
  uint8_t kind = 0;
  uint8_t _pad[3] = {0, 0, 0};
  uint32_t button = 0;
  uint32_t buttons_mask = 0;
  uint32_t modifiers = 0;
  uint32_t key_code = 0;
  uint32_t width = 0;
  uint32_t height = 0;
  uint32_t quality_flags = kQualityDefault;
  float x = 0.0f;
  float y = 0.0f;
  float dx = 0.0f;
  float dy = 0.0f;
  float dpr = 1.0f;
  float camera_hfov = 1.0472f;
  PoseWire camera_pose{};
  uint32_t path_len = 0;
  // Followed by path_len UTF-8 bytes
};
static_assert(sizeof(GuiInputWire) == 88);

struct FrameReadyWire
{
  uint32_t generation = 0;
  uint32_t slot = 0;
  uint32_t width = 0;
  uint32_t height = 0;
  uint64_t frame_id = 0;
  float render_ms = 0.0f;
  float fps = 0.0f;
  uint32_t entity_count = 0;
  uint32_t quality_flags = 0;
  uint8_t backend = 0;       // RendererBackend (0 = Bevy, 1 = Blender)
  uint8_t refining = 0;      // 1 while progressive refinement is still running
  uint16_t sample_count = 0; // Active render sample count (TAA / Cycles spp)
};
static_assert(sizeof(FrameReadyWire) == 44);

struct SlotAcquiredWire
{
  uint32_t generation = 0;
  uint32_t active_slot = 0;
};
static_assert(sizeof(SlotAcquiredWire) == 8);

#pragma pack(pop)

// Helpers to assemble framed binary packets into a flat byte buffer
class PacketBuilder
{
 public:
  void Reset(MsgType type, uint32_t count = 1)
  {
    buf_.clear();
    buf_.resize(sizeof(MsgHeader));
    MsgHeader hdr{};
    hdr.magic = kProtocolMagic;
    hdr.version = kProtocolVersion;
    hdr.msg_type = static_cast<uint16_t>(type);
    hdr.count = count;
    hdr.payload_bytes = 0;
    std::memcpy(buf_.data(), &hdr, sizeof(hdr));
  }

  template <typename T>
  void AppendPod(const T &val)
  {
    const auto offset = buf_.size();
    buf_.resize(offset + sizeof(T));
    std::memcpy(buf_.data() + offset, &val, sizeof(T));
  }

  void AppendBytes(const void *data, size_t len)
  {
    if (len == 0) return;
    const auto offset = buf_.size();
    buf_.resize(offset + len);
    std::memcpy(buf_.data() + offset, data, len);
  }

  void AppendString(std::string_view s)
  {
    AppendBytes(s.data(), s.size());
  }

  const std::vector<uint8_t> &Finish()
  {
    auto *hdr = reinterpret_cast<MsgHeader *>(buf_.data());
    hdr->payload_bytes = static_cast<uint32_t>(buf_.size() - sizeof(MsgHeader));
    return buf_;
  }

  void SetCount(uint32_t count)
  {
    auto *hdr = reinterpret_cast<MsgHeader *>(buf_.data());
    hdr->count = count;
  }

 private:
  std::vector<uint8_t> buf_;
};

// Lightweight FNV-1a 64-bit hash for deterministic mesh/texture IDs
inline uint64_t HashString64(std::string_view s)
{
  uint64_t h = 14695981039346656037ULL;
  for (unsigned char c : s)
  {
    h ^= static_cast<uint64_t>(c);
    h *= 1099511628211ULL;
  }
  return h == 0 ? 1 : h;
}

}  // namespace gz::ext_render

#endif  // GZ_EXTERNAL_RENDERER_PROTOCOL_HH_
