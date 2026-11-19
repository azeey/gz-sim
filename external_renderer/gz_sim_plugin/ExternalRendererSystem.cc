#include "ExternalRendererSystem.hh"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/common/Image.hh>
#include <gz/common/Material.hh>
#include <gz/common/Mesh.hh>
#include <gz/common/MeshManager.hh>
#include <gz/common/Pbr.hh>
#include <gz/common/SubMesh.hh>
#include <gz/common/Util.hh>
#include <gz/math/Color.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector2.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/CastShadows.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/Light.hh>
#include <gz/sim/components/Material.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Scene.hh>
#include <gz/sim/components/SourceFilePath.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/components/World.hh>

#include <sdf/Box.hh>
#include <sdf/Capsule.hh>
#include <sdf/Cone.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Ellipsoid.hh>
#include <sdf/Geometry.hh>
#include <sdf/Light.hh>
#include <sdf/Material.hh>
#include <sdf/Mesh.hh>
#include <sdf/Pbr.hh>
#include <sdf/Plane.hh>
#include <sdf/Scene.hh>
#include <sdf/Sky.hh>
#include <sdf/Sphere.hh>

#include "protocol.hh"
#include "zenoh_transport.hh"

namespace fs = std::filesystem;

namespace gz::sim::systems
{

namespace
{

std::string ToLower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

bool IsGltfFile(const std::string &path)
{
  const auto ext = ToLower(fs::path(path).extension().string());
  return ext == ".glb" || ext == ".gltf";
}

ext_render::PoseWire ToPoseWire(const math::Pose3d &p)
{
  ext_render::PoseWire w{};
  w.px = static_cast<float>(p.Pos().X());
  w.py = static_cast<float>(p.Pos().Y());
  w.pz = static_cast<float>(p.Pos().Z());
  w.qx = static_cast<float>(p.Rot().X());
  w.qy = static_cast<float>(p.Rot().Y());
  w.qz = static_cast<float>(p.Rot().Z());
  w.qw = static_cast<float>(p.Rot().W());
  return w;
}

bool PoseChanged(const math::Pose3d &a, const math::Pose3d &b)
{
  if (a.Pos().Distance(b.Pos()) > 1e-4) return true;
  const double dot = std::abs(a.Rot().X() * b.Rot().X() +
                              a.Rot().Y() * b.Rot().Y() +
                              a.Rot().Z() * b.Rot().Z() +
                              a.Rot().W() * b.Rot().W());
  return dot < 0.999999;
}

std::string FindSourceFilePath(Entity entity, const EntityComponentManager &ecm)
{
  Entity cur = entity;
  while (cur != kNullEntity)
  {
    if (auto *sfp = ecm.Component<components::SourceFilePath>(cur))
    {
      if (!sfp->Data().empty()) return sfp->Data();
    }
    auto *parent = ecm.Component<components::ParentEntity>(cur);
    if (!parent) break;
    cur = parent->Data();
  }
  return {};
}

std::string ResolveAssetPath(const std::string &uri, const std::string &sourceFilePath)
{
  if (uri.empty()) return {};
  if (fs::exists(uri)) return fs::weakly_canonical(uri).string();

  std::string full = asFullPath(uri, sourceFilePath);
  if (!full.empty() && fs::exists(full)) return fs::weakly_canonical(full).string();

  std::string found = common::findFile(full);
  if (!found.empty() && fs::exists(found)) return fs::weakly_canonical(found).string();

  found = common::findFile(uri);
  if (!found.empty() && fs::exists(found)) return fs::weakly_canonical(found).string();

  // Also try relative to source file directory directly
  if (!sourceFilePath.empty())
  {
    fs::path base = fs::path(sourceFilePath);
    if (fs::is_regular_file(base)) base = base.parent_path();
    fs::path candidate = base / uri;
    if (fs::exists(candidate)) return fs::weakly_canonical(candidate).string();
  }
  return full.empty() ? uri : full;
}

}  // namespace

class ExternalRendererSystemPrivate
{
 public:
  std::string sessionName;
  std::string zenohEndpoint;
  ext_render::ZenohNode zenoh;
  std::shared_ptr<ext_render::ZenohNode::Publisher> scenePub;

  std::atomic<bool> snapshotRequested{true};
  struct RectLightConfig
  {
    uint64_t id{0};
    math::Pose3d pose;
    float width{1.0f};
    float height{1.0f};
    float intensity{1000.0f};
    float range{30.0f};
    float color[3]{1.0f, 1.0f, 1.0f};
    float spread{0.0f};       // Radians (0 => default pi / 180 deg)
    bool castShadows{false};
  };

  bool configured{false};
  bool worldConfigSent{false};
  bool hasConfigCameraPose{false};
  math::Pose3d configCameraPose;
  float configCameraHfov{1.0472f};
  std::string configEnvMapUri;
  float configEnvMapIntensity{350.0f};
  bool hasConfigSkyIntensity{false};
  float configSkyIntensity{1.0f};
  std::vector<RectLightConfig> configRectLights;

  std::unordered_set<uint64_t> sentTextures;
  std::unordered_set<uint64_t> sentMeshes;
  std::unordered_set<Entity> sentVisuals;
  std::unordered_set<Entity> sentLights;
  std::unordered_map<Entity, math::Pose3d> lastPoses;

  std::chrono::steady_clock::time_point lastTransformSend;
  std::chrono::steady_clock::time_point lastPeriodicCheck;
  ext_render::PacketBuilder builder;

  uint64_t EnsureTextureFromFile(const std::string &rawPath,
                                 const std::string &sourceFilePath,
                                 ext_render::TextureFormat fmt)
  {
    if (rawPath.empty()) return 0;
    const std::string path = ResolveAssetPath(rawPath, sourceFilePath);
    if (path.empty() || !fs::exists(path))
    {
      return 0;
    }
    const std::string key = path + "|" + std::to_string(static_cast<int>(fmt));
    const uint64_t texId = ext_render::HashString64(key);
    if (sentTextures.count(texId)) return texId;

    common::Image img;
    if (img.Load(path, common::Image::RGBA_INT8) != 0)
    {
      gzwarn << "[ExternalRendererSystem] Failed to load texture: " << path << std::endl;
      return 0;
    }
    const uint32_t w = img.Width();
    const uint32_t h = img.Height();
    if (w == 0 || h == 0) return 0;

    const auto rawData = img.Data();
    if (rawData.size() < static_cast<size_t>(w) * h * 4)
    {
      return 0;
    }

    SendTextureBytes(texId, w, h, fmt, rawData.data(), static_cast<size_t>(w) * h * 4);
    sentTextures.insert(texId);
    return texId;
  }

  uint64_t EnsureTextureFromMemory(const std::shared_ptr<const common::Image> &imgPtr,
                                   const std::string &cacheKey,
                                   ext_render::TextureFormat fmt)
  {
    if (!imgPtr) return 0;
    const uint64_t texId = ext_render::HashString64(cacheKey);
    if (sentTextures.count(texId)) return texId;

    const uint32_t w = imgPtr->Width();
    const uint32_t h = imgPtr->Height();
    if (w == 0 || h == 0) return 0;

    const auto rgba = imgPtr->RGBAData();
    if (rgba.size() < static_cast<size_t>(w) * h * 4)
    {
      return 0;
    }
    SendTextureBytes(texId, w, h, fmt, rgba.data(), static_cast<size_t>(w) * h * 4);
    sentTextures.insert(texId);
    return texId;
  }

  // Pack separate SDF roughness_map and metalness_map into glTF/Bevy convention:
  //   R = 255, G = Roughness, B = Metalness, A = 255 (linear RGBA8_UNORM)
  uint64_t EnsureMetallicRoughnessTexture(const std::string &roughPathRaw,
                                          const std::string &metalPathRaw,
                                          const std::string &sourceFilePath,
                                          float defaultRough,
                                          float defaultMetal)
  {
    const std::string rPath = ResolveAssetPath(roughPathRaw, sourceFilePath);
    const std::string mPath = ResolveAssetPath(metalPathRaw, sourceFilePath);
    const bool hasR = !rPath.empty() && fs::exists(rPath);
    const bool hasM = !mPath.empty() && fs::exists(mPath);
    if (!hasR && !hasM) return 0;

    // If both point to the exact same file, it is likely already glTF packed (G=rough, B=metal)
    if (hasR && hasM && rPath == mPath)
    {
      return EnsureTextureFromFile(rPath, sourceFilePath, ext_render::TextureFormat::Rgba8Unorm);
    }

    const std::string key = "mr_pack|" + rPath + "|" + mPath;
    const uint64_t texId = ext_render::HashString64(key);
    if (sentTextures.count(texId)) return texId;

    common::Image rImg, mImg;
    bool loadedR = hasR && (rImg.Load(rPath, common::Image::RGBA_INT8) == 0);
    bool loadedM = hasM && (mImg.Load(mPath, common::Image::RGBA_INT8) == 0);
    if (!loadedR && !loadedM) return 0;

    uint32_t w = loadedR ? rImg.Width() : mImg.Width();
    uint32_t h = loadedR ? rImg.Height() : mImg.Height();
    if (w == 0 || h == 0) return 0;

    std::vector<unsigned char> rBuf;
    if (loadedR) rBuf = rImg.Data();

    std::vector<unsigned char> mBuf;
    if (loadedM) mBuf = mImg.Data();

    const uint32_t rw = loadedR ? rImg.Width() : 1;
    const uint32_t rh = loadedR ? rImg.Height() : 1;
    const uint32_t mw = loadedM ? mImg.Width() : 1;
    const uint32_t mh = loadedM ? mImg.Height() : 1;

    const uint8_t defR = static_cast<uint8_t>(std::clamp(defaultRough, 0.0f, 1.0f) * 255.0f);
    const uint8_t defM = static_cast<uint8_t>(std::clamp(defaultMetal, 0.0f, 1.0f) * 255.0f);

    std::vector<uint8_t> packed(static_cast<size_t>(w) * h * 4);
    for (uint32_t y = 0; y < h; ++y)
    {
      for (uint32_t x = 0; x < w; ++x)
      {
        uint8_t roughVal = defR;
        if (loadedR && !rBuf.empty())
        {
          uint32_t rx = (x * rw) / w;
          uint32_t ry = (y * rh) / h;
          const size_t idx = (static_cast<size_t>(ry) * rw + rx) * 4;
          if (idx + 1 < rBuf.size())
            roughVal = rBuf[idx + 1];
        }

        uint8_t metalVal = defM;
        if (loadedM && !mBuf.empty())
        {
          uint32_t mx = (x * mw) / w;
          uint32_t my = (y * mh) / h;
          const size_t idx = (static_cast<size_t>(my) * mw + mx) * 4;
          if (idx + 2 < mBuf.size())
            metalVal = mBuf[idx + 2];
        }

        const size_t dst = (static_cast<size_t>(y) * w + x) * 4;
        packed[dst + 0] = 255;
        packed[dst + 1] = roughVal;
        packed[dst + 2] = metalVal;
        packed[dst + 3] = 255;
      }
    }

    SendTextureBytes(texId, w, h, ext_render::TextureFormat::Rgba8Unorm, packed.data(), packed.size());
    sentTextures.insert(texId);
    return texId;
  }

  void SendTextureBytes(uint64_t texId,
                        uint32_t w,
                        uint32_t h,
                        ext_render::TextureFormat fmt,
                        const uint8_t *pixels,
                        size_t byteLen)
  {
    if (!scenePub) return;
    ext_render::TextureDataWire td{};
    td.texture_id = texId;
    td.width = w;
    td.height = h;
    td.format = static_cast<uint8_t>(fmt);
    td.data_len = static_cast<uint32_t>(byteLen);
    builder.Reset(ext_render::MsgType::TextureData, 1);
    builder.AppendPod(td);
    builder.AppendBytes(pixels, byteLen);
    scenePub->Publish(builder.Finish(), true);
  }

  uint64_t EnsureSubMesh(const std::string &meshKey, const common::SubMesh &sub)
  {
    const uint64_t meshId = ext_render::HashString64(meshKey);
    if (sentMeshes.count(meshId)) return meshId;

    const unsigned int vCount = sub.VertexCount();
    const unsigned int nCount = sub.NormalCount();
    const unsigned int uvCount = sub.TexCoordCount();
    const unsigned int rawICount = sub.IndexCount();
    if (vCount == 0) return 0;

    std::vector<float> positions;
    positions.reserve(vCount * 3);
    for (unsigned int i = 0; i < vCount; ++i)
    {
      const auto v = sub.Vertex(i);
      positions.push_back(static_cast<float>(v.X()));
      positions.push_back(static_cast<float>(v.Y()));
      positions.push_back(static_cast<float>(v.Z()));
    }

    const bool hasNormals = (nCount == vCount && nCount > 0);
    std::vector<float> normals;
    if (hasNormals)
    {
      normals.reserve(vCount * 3);
      for (unsigned int i = 0; i < vCount; ++i)
      {
        const auto n = sub.Normal(i);
        normals.push_back(static_cast<float>(n.X()));
        normals.push_back(static_cast<float>(n.Y()));
        normals.push_back(static_cast<float>(n.Z()));
      }
    }

    const bool hasUvs = (uvCount == vCount && uvCount > 0);
    std::vector<float> uvs;
    if (hasUvs)
    {
      uvs.reserve(vCount * 2);
      for (unsigned int i = 0; i < vCount; ++i)
      {
        const auto uv = sub.TexCoord(i);
        uvs.push_back(static_cast<float>(uv.X()));
        uvs.push_back(static_cast<float>(uv.Y()));
      }
    }

    // Triangulate indices according to primitive type
    std::vector<uint32_t> indices;
    const auto ptype = sub.SubMeshPrimitiveType();
    if (rawICount > 0)
    {
      if (ptype == common::SubMesh::TRIANGLES)
      {
        indices.reserve(rawICount);
        for (unsigned int i = 0; i < rawICount; ++i)
          indices.push_back(static_cast<uint32_t>(sub.Index(i)));
      }
      else if (ptype == common::SubMesh::TRISTRIPS && rawICount >= 3)
      {
        for (unsigned int i = 0; i + 2 < rawICount; ++i)
        {
          uint32_t i0 = static_cast<uint32_t>(sub.Index(i));
          uint32_t i1 = static_cast<uint32_t>(sub.Index(i + 1));
          uint32_t i2 = static_cast<uint32_t>(sub.Index(i + 2));
          if (i % 2 == 0)
          {
            indices.push_back(i0);
            indices.push_back(i1);
            indices.push_back(i2);
          }
          else
          {
            indices.push_back(i0);
            indices.push_back(i2);
            indices.push_back(i1);
          }
        }
      }
      else if (ptype == common::SubMesh::TRIFANS && rawICount >= 3)
      {
        uint32_t i0 = static_cast<uint32_t>(sub.Index(0));
        for (unsigned int i = 1; i + 1 < rawICount; ++i)
        {
          indices.push_back(i0);
          indices.push_back(static_cast<uint32_t>(sub.Index(i)));
          indices.push_back(static_cast<uint32_t>(sub.Index(i + 1)));
        }
      }
      else
      {
        // Unsupported non-triangle primitive (POINTS/LINES)
        return 0;
      }
    }
    else if (ptype == common::SubMesh::TRIANGLES)
    {
      indices.resize(vCount);
      for (unsigned int i = 0; i < vCount; ++i) indices[i] = i;
    }
    else
    {
      return 0;
    }

    if (indices.size() < 3) return 0;

    ext_render::MeshDataWire md{};
    md.mesh_id = meshId;
    md.vertex_count = vCount;
    md.index_count = static_cast<uint32_t>(indices.size());
    md.has_normals = hasNormals ? 1 : 0;
    md.has_uvs = hasUvs ? 1 : 0;
    md.has_tangents = 0;

    builder.Reset(ext_render::MsgType::MeshData, 1);
    builder.AppendPod(md);
    builder.AppendBytes(positions.data(), positions.size() * sizeof(float));
    if (hasNormals) builder.AppendBytes(normals.data(), normals.size() * sizeof(float));
    if (hasUvs) builder.AppendBytes(uvs.data(), uvs.size() * sizeof(float));
    builder.AppendBytes(indices.data(), indices.size() * sizeof(uint32_t));
    scenePub->Publish(builder.Finish(), true);

    sentMeshes.insert(meshId);
    return meshId;
  }

  ext_render::PbrMaterialWire ExtractMaterial(const common::Material *meshMat,
                                              const sdf::Material *sdfMat,
                                              const std::string &meshPath,
                                              const std::string &sourceFilePath,
                                              uint8_t &outDoubleSided,
                                              uint8_t &outAlphaMode)
  {
    ext_render::PbrMaterialWire out{};
    outDoubleSided = 0;
    outAlphaMode = static_cast<uint8_t>(ext_render::AlphaMode::Opaque);

    const std::string meshDir =
        meshPath.empty() ? sourceFilePath : fs::path(meshPath).parent_path().string();

    // 1. Base values from embedded mesh material (Assimp / Collada / OBJ)
    if (meshMat)
    {
      const auto diff = meshMat->Diffuse();
      out.base_color[0] = diff.R();
      out.base_color[1] = diff.G();
      out.base_color[2] = diff.B();
      out.base_color[3] = static_cast<float>(1.0 - meshMat->Transparency());

      const auto emis = meshMat->Emissive();
      out.emissive[0] = emis.R();
      out.emissive[1] = emis.G();
      out.emissive[2] = emis.B();
      out.emissive[3] = 1.0f;

      if (meshMat->TwoSidedEnabled()) outDoubleSided = 1;
      if (meshMat->TextureAlphaEnabled())
      {
        outAlphaMode = static_cast<uint8_t>(ext_render::AlphaMode::Mask);
        out.alpha_cutoff = static_cast<float>(meshMat->AlphaThreshold());
      }

      if (auto texImg = meshMat->TextureData())
      {
        out.base_color_tex = EnsureTextureFromMemory(
            texImg, meshPath + "|tex|" + meshMat->TextureImage(),
            ext_render::TextureFormat::Rgba8Srgb);
      }
      else if (!meshMat->TextureImage().empty())
      {
        out.base_color_tex = EnsureTextureFromFile(
            meshMat->TextureImage(), meshDir, ext_render::TextureFormat::Rgba8Srgb);
      }

      if (const auto *pbr = meshMat->PbrMaterial())
      {
        out.metallic = static_cast<float>(pbr->Metalness());
        out.roughness = static_cast<float>(pbr->Roughness());
        if (!pbr->AlbedoMap().empty())
        {
          out.base_color_tex = EnsureTextureFromFile(
              pbr->AlbedoMap(), meshDir, ext_render::TextureFormat::Rgba8Srgb);
        }
        if (!pbr->NormalMap().empty())
        {
          out.normal_tex = EnsureTextureFromFile(
              pbr->NormalMap(), meshDir, ext_render::TextureFormat::Rgba8Unorm);
        }
        else if (auto nData = pbr->NormalMapData())
        {
          out.normal_tex = EnsureTextureFromMemory(
              nData, meshPath + "|norm|" + meshMat->Name(),
              ext_render::TextureFormat::Rgba8Unorm);
        }
        if (!pbr->RoughnessMap().empty() || !pbr->MetalnessMap().empty())
        {
          out.metallic_roughness_tex = EnsureMetallicRoughnessTexture(
              pbr->RoughnessMap(), pbr->MetalnessMap(), meshDir, out.roughness, out.metallic);
        }
        if (!pbr->AmbientOcclusionMap().empty())
        {
          out.occlusion_tex = EnsureTextureFromFile(
              pbr->AmbientOcclusionMap(), meshDir, ext_render::TextureFormat::Rgba8Unorm);
        }
        if (!pbr->EmissiveMap().empty())
        {
          out.emissive_tex = EnsureTextureFromFile(
              pbr->EmissiveMap(), meshDir, ext_render::TextureFormat::Rgba8Srgb);
        }
      }
    }

    // 2. Overlay SDF <material> overrides
    if (sdfMat)
    {
      const std::string matBase =
          sdfMat->FilePath().empty() ? sourceFilePath : sdfMat->FilePath();

      const auto diff = sdfMat->Diffuse();
      out.base_color[0] = diff.R();
      out.base_color[1] = diff.G();
      out.base_color[2] = diff.B();
      out.base_color[3] = diff.A();

      const auto emis = sdfMat->Emissive();
      out.emissive[0] = emis.R();
      out.emissive[1] = emis.G();
      out.emissive[2] = emis.B();
      out.emissive[3] = emis.A();

      if (sdfMat->DoubleSided()) outDoubleSided = 1;

      if (const auto *pbr = sdfMat->PbrMaterial())
      {
        if (const auto *wf = pbr->Workflow(sdf::PbrWorkflowType::METAL))
        {
          out.metallic = static_cast<float>(wf->Metalness());
          out.roughness = static_cast<float>(wf->Roughness());

          if (!wf->AlbedoMap().empty())
          {
            uint64_t tid = EnsureTextureFromFile(
                wf->AlbedoMap(), matBase, ext_render::TextureFormat::Rgba8Srgb);
            if (tid != 0)
            {
              out.base_color_tex = tid;
              // SDF defaults diffuse to (0.8, 0.8, 0.8) if not explicitly set; keep white if albedo map drives color
              if (out.base_color[0] < 0.01f && out.base_color[1] < 0.01f && out.base_color[2] < 0.01f)
              {
                out.base_color[0] = out.base_color[1] = out.base_color[2] = 1.0f;
              }
            }
          }
          if (!wf->NormalMap().empty())
          {
            uint64_t tid = EnsureTextureFromFile(
                wf->NormalMap(), matBase, ext_render::TextureFormat::Rgba8Unorm);
            if (tid != 0) out.normal_tex = tid;
          }
          if (!wf->RoughnessMap().empty() || !wf->MetalnessMap().empty())
          {
            uint64_t tid = EnsureMetallicRoughnessTexture(
                wf->RoughnessMap(), wf->MetalnessMap(), matBase, out.roughness, out.metallic);
            if (tid != 0)
            {
              out.metallic_roughness_tex = tid;
              // Bevy multiplies factor * texture; ensure non-zero factors when maps exist
              if (!wf->RoughnessMap().empty() && out.roughness <= 0.01f) out.roughness = 1.0f;
              if (!wf->MetalnessMap().empty() && out.metallic <= 0.01f) out.metallic = 1.0f;
            }
          }
          if (!wf->AmbientOcclusionMap().empty())
          {
            uint64_t tid = EnsureTextureFromFile(
                wf->AmbientOcclusionMap(), matBase, ext_render::TextureFormat::Rgba8Unorm);
            if (tid != 0) out.occlusion_tex = tid;
          }
          if (!wf->EmissiveMap().empty())
          {
            uint64_t tid = EnsureTextureFromFile(
                wf->EmissiveMap(), matBase, ext_render::TextureFormat::Rgba8Srgb);
            if (tid != 0) out.emissive_tex = tid;
          }
        }
      }
    }

    return out;
  }

  void SendWorldConfig(const EntityComponentManager &ecm)
  {
    if (!scenePub) return;
    ext_render::WorldConfigWire cfg{};
    cfg.ambient[0] = cfg.ambient[1] = cfg.ambient[2] = 0.4f;
    cfg.ambient[3] = 1.0f;
    cfg.background[0] = cfg.background[1] = cfg.background[2] = 0.22f;
    cfg.background[3] = 1.0f;
    cfg.has_sky = 1;
    cfg.sky_intensity = 1.0f;

    std::string worldSourceFile;
    ecm.Each<components::World, components::Scene>(
        [&](const Entity &we, const components::World *, const components::Scene *sceneComp) -> bool {
          const auto &s = sceneComp->Data();
          cfg.ambient[0] = s.Ambient().R();
          cfg.ambient[1] = s.Ambient().G();
          cfg.ambient[2] = s.Ambient().B();
          cfg.ambient[3] = s.Ambient().A();
          cfg.background[0] = s.Background().R();
          cfg.background[1] = s.Background().G();
          cfg.background[2] = s.Background().B();
          cfg.background[3] = s.Background().A();
          cfg.has_sky = (s.Sky() != nullptr) ? 1 : 0;
          worldSourceFile = FindSourceFilePath(we, ecm);
          return false;
        });

    if (hasConfigCameraPose)
    {
      cfg.has_camera_pose = 1;
      cfg.camera_pose = ToPoseWire(configCameraPose);
      cfg.camera_hfov = configCameraHfov;
    }

    std::string resolvedEnvMap;
    if (!configEnvMapUri.empty())
    {
      resolvedEnvMap = ResolveAssetPath(configEnvMapUri, worldSourceFile);
      cfg.env_map_len = static_cast<uint32_t>(resolvedEnvMap.size());
      cfg.sky_intensity = configEnvMapIntensity;
    }
    else if (hasConfigSkyIntensity)
    {
      cfg.sky_intensity = configSkyIntensity;
    }

    builder.Reset(ext_render::MsgType::WorldConfig, 1);
    builder.AppendPod(cfg);
    if (!resolvedEnvMap.empty())
    {
      builder.AppendString(resolvedEnvMap);
    }
    scenePub->Publish(builder.Finish(), true);
    worldConfigSent = true;
  }

  void SendLight(Entity entity, const sdf::Light &light, const EntityComponentManager &ecm)
  {
    if (!scenePub) return;
    ext_render::LightUpsertWire lw{};
    lw.entity_id = static_cast<uint64_t>(entity);
    switch (light.Type())
    {
      case sdf::LightType::DIRECTIONAL:
        lw.light_type = static_cast<uint8_t>(ext_render::LightType::Directional);
        break;
      case sdf::LightType::POINT:
        lw.light_type = static_cast<uint8_t>(ext_render::LightType::Point);
        break;
      case sdf::LightType::SPOT:
        lw.light_type = static_cast<uint8_t>(ext_render::LightType::Spot);
        break;
      default:
        return;
    }
    lw.cast_shadows = light.CastShadows() ? 1 : 0;
    lw.intensity = static_cast<float>(light.Intensity());

    math::Pose3d wp = worldPose(entity, ecm);
    if (wp == math::Pose3d::Zero) wp = light.RawPose();
    lw.pose = ToPoseWire(wp);

    lw.direction[0] = static_cast<float>(light.Direction().X());
    lw.direction[1] = static_cast<float>(light.Direction().Y());
    lw.direction[2] = static_cast<float>(light.Direction().Z());
    lw.range = static_cast<float>(light.AttenuationRange());
    lw.color[0] = light.Diffuse().R();
    lw.color[1] = light.Diffuse().G();
    lw.color[2] = light.Diffuse().B();
    lw.color[3] = light.Diffuse().A();
    lw.inner_angle = static_cast<float>(light.SpotInnerAngle().Radian());
    lw.outer_angle = static_cast<float>(light.SpotOuterAngle().Radian());

    builder.Reset(ext_render::MsgType::LightUpsert, 1);
    builder.AppendPod(lw);
    scenePub->Publish(builder.Finish(), true);
    sentLights.insert(entity);
  }

  void SendVisual(Entity entity,
                  const sdf::Geometry &geom,
                  const sdf::Material *sdfMat,
                  const EntityComponentManager &ecm)
  {
    if (!scenePub) return;
    const math::Pose3d wp = worldPose(entity, ecm);
    lastPoses[entity] = wp;

    bool castShadows = true;
    if (auto *cs = ecm.Component<components::CastShadows>(entity))
    {
      castShadows = cs->Data();
    }
    const std::string srcFile = FindSourceFilePath(entity, ecm);

    auto publishSimple = [&](ext_render::VisualKind kind, float p0, float p1, float p2, float p3) {
      ext_render::SpawnVisualWire sv{};
      sv.visual_id = static_cast<uint64_t>(entity) << 16;
      sv.parent_entity_id = static_cast<uint64_t>(entity);
      sv.kind = static_cast<uint8_t>(kind);
      sv.cast_shadows = castShadows ? 1 : 0;
      sv.parent_pose = ToPoseWire(wp);
      sv.local_pose = ToPoseWire(math::Pose3d::Zero);
      sv.params[0] = p0;
      sv.params[1] = p1;
      sv.params[2] = p2;
      sv.params[3] = p3;
      sv.material = ExtractMaterial(nullptr, sdfMat, "", srcFile, sv.double_sided, sv.alpha_mode);

      builder.Reset(ext_render::MsgType::SpawnVisual, 1);
      builder.AppendPod(sv);
      scenePub->Publish(builder.Finish(), true);
    };

    switch (geom.Type())
    {
      case sdf::GeometryType::BOX:
        if (auto *b = geom.BoxShape())
        {
          publishSimple(ext_render::VisualKind::Box,
                        static_cast<float>(b->Size().X()),
                        static_cast<float>(b->Size().Y()),
                        static_cast<float>(b->Size().Z()), 0.0f);
        }
        break;
      case sdf::GeometryType::SPHERE:
        if (auto *s = geom.SphereShape())
        {
          publishSimple(ext_render::VisualKind::Sphere,
                        static_cast<float>(s->Radius()), 0.0f, 0.0f, 0.0f);
        }
        break;
      case sdf::GeometryType::CYLINDER:
        if (auto *c = geom.CylinderShape())
        {
          publishSimple(ext_render::VisualKind::Cylinder,
                        static_cast<float>(c->Radius()),
                        static_cast<float>(c->Length()), 0.0f, 0.0f);
        }
        break;
      case sdf::GeometryType::CAPSULE:
        if (auto *c = geom.CapsuleShape())
        {
          publishSimple(ext_render::VisualKind::Capsule,
                        static_cast<float>(c->Radius()),
                        static_cast<float>(c->Length()), 0.0f, 0.0f);
        }
        break;
      case sdf::GeometryType::PLANE:
        if (auto *p = geom.PlaneShape())
        {
          publishSimple(ext_render::VisualKind::Plane,
                        static_cast<float>(p->Size().X()),
                        static_cast<float>(p->Size().Y()), 0.0f, 0.0f);
        }
        break;
      case sdf::GeometryType::CONE:
        if (auto *c = geom.ConeShape())
        {
          publishSimple(ext_render::VisualKind::Cone,
                        static_cast<float>(c->Radius()),
                        static_cast<float>(c->Length()), 0.0f, 0.0f);
        }
        break;
      case sdf::GeometryType::ELLIPSOID:
        if (auto *el = geom.EllipsoidShape())
        {
          publishSimple(ext_render::VisualKind::Ellipsoid,
                        static_cast<float>(el->Radii().X()),
                        static_cast<float>(el->Radii().Y()),
                        static_cast<float>(el->Radii().Z()), 0.0f);
        }
        break;
      case sdf::GeometryType::MESH:
      {
        const auto *m = geom.MeshShape();
        if (!m) break;
        const std::string uri = m->Uri();
        const std::string meshBase = m->FilePath().empty() ? srcFile : m->FilePath();
        const std::string resolved = ResolveAssetPath(uri, meshBase);
        const auto sc = m->Scale();

        // Hybrid strategy: pass resolved .glb/.gltf directly to Bevy's native glTF PBR loader!
        if (IsGltfFile(resolved) && fs::exists(resolved) && m->Submesh().empty())
        {
          ext_render::SpawnVisualWire sv{};
          sv.visual_id = static_cast<uint64_t>(entity) << 16;
          sv.parent_entity_id = static_cast<uint64_t>(entity);
          sv.kind = static_cast<uint8_t>(ext_render::VisualKind::GltfUri);
          sv.cast_shadows = castShadows ? 1 : 0;
          sv.uri_len = static_cast<uint32_t>(resolved.size());
          sv.parent_pose = ToPoseWire(wp);
          sv.local_pose = ToPoseWire(math::Pose3d::Zero);
          sv.params[0] = static_cast<float>(sc.X());
          sv.params[1] = static_cast<float>(sc.Y());
          sv.params[2] = static_cast<float>(sc.Z());
          sv.params[3] = 0.0f;

          builder.Reset(ext_render::MsgType::SpawnVisual, 1);
          builder.AppendPod(sv);
          builder.AppendString(resolved);
          scenePub->Publish(builder.Finish(), true);
          break;
        }

        // Otherwise load via gz-common MeshManager (.dae, .obj, .stl, or submesh-filtered glTF)
        // and stream raw vertices/normals/UVs/indices + PBR textures over Zenoh SHM!
        auto *meshMgr = common::MeshManager::Instance();
        const common::Mesh *mesh = meshMgr->Load(resolved);
        if (!mesh)
        {
          gzwarn << "[ExternalRendererSystem] MeshManager failed to load: " << resolved << std::endl;
          break;
        }

        const std::string filterSub = m->Submesh();
        const unsigned int subCount = mesh->SubMeshCount();
        for (unsigned int i = 0; i < subCount; ++i)
        {
          auto subWeak = mesh->SubMeshByIndex(i);
          auto sub = subWeak.lock();
          if (!sub) continue;
          if (!filterSub.empty() && sub->Name() != filterSub) continue;

          const std::string meshKey = resolved + "#" + std::to_string(i) + "#" + sub->Name();
          const uint64_t meshId = EnsureSubMesh(meshKey, *sub);
          if (meshId == 0) continue;

          const common::Material *subMat = nullptr;
          if (auto matIdx = sub->GetMaterialIndex())
          {
            subMat = mesh->MaterialByIndex(*matIdx).get();
          }

          ext_render::SpawnVisualWire sv{};
          sv.visual_id = (static_cast<uint64_t>(entity) << 16) | static_cast<uint64_t>(i & 0xFFFFu);
          sv.parent_entity_id = static_cast<uint64_t>(entity);
          sv.kind = static_cast<uint8_t>(ext_render::VisualKind::StreamedMesh);
          sv.cast_shadows = castShadows ? 1 : 0;
          sv.parent_pose = ToPoseWire(wp);
          sv.local_pose = ToPoseWire(math::Pose3d::Zero);
          sv.params[0] = static_cast<float>(sc.X());
          sv.params[1] = static_cast<float>(sc.Y());
          sv.params[2] = static_cast<float>(sc.Z());
          sv.params[3] = 0.0f;
          sv.mesh_id = meshId;
          sv.material = ExtractMaterial(
              subMat, sdfMat, resolved, meshBase, sv.double_sided, sv.alpha_mode);

          builder.Reset(ext_render::MsgType::SpawnVisual, 1);
          builder.AppendPod(sv);
          scenePub->Publish(builder.Finish(), true);
        }
        break;
      }
      default:
        break;
    }

    sentVisuals.insert(entity);
  }

  void SendFullSnapshot(const EntityComponentManager &ecm)
  {
    if (!scenePub) return;
    sentTextures.clear();
    sentMeshes.clear();
    sentVisuals.clear();
    sentLights.clear();
    lastPoses.clear();

    builder.Reset(ext_render::MsgType::ClearWorld, 0);
    scenePub->Publish(builder.Finish(), false);

    SendWorldConfig(ecm);

    // Lights (standard SDF directional/point/spot)
    ecm.Each<components::Light>(
        [&](const Entity &e, const components::Light *lightComp) -> bool {
          SendLight(e, lightComp->Data(), ecm);
          return true;
        });

    // Optional rectangular area lights configured on ExternalRendererSystem (<area_light> / <rect_light>)
    for (const auto &rl : configRectLights)
    {
      ext_render::LightUpsertWire lw{};
      lw.entity_id = rl.id;
      lw.light_type = static_cast<uint8_t>(ext_render::LightType::Rect);
      lw.cast_shadows = rl.castShadows ? 1 : 0;
      lw.spread_mrad = rl.spread > 0.0f
          ? static_cast<uint16_t>(std::clamp(std::lround(rl.spread * 1000.0f), 1L, 3142L))
          : 0u;
      lw.intensity = rl.intensity;
      lw.pose = ToPoseWire(rl.pose);
      lw.direction[0] = 0.0f;
      lw.direction[1] = 0.0f;
      lw.direction[2] = -1.0f;
      lw.range = rl.range;
      lw.color[0] = rl.color[0];
      lw.color[1] = rl.color[1];
      lw.color[2] = rl.color[2];
      lw.color[3] = 1.0f;
      // For LightType::Rect, inner_angle carries width and outer_angle carries height (meters)
      lw.inner_angle = rl.width;
      lw.outer_angle = rl.height;

      builder.Reset(ext_render::MsgType::LightUpsert, 1);
      builder.AppendPod(lw);
      scenePub->Publish(builder.Finish(), true);
    }

    // Visuals
    ecm.Each<components::Visual, components::Geometry>(
        [&](const Entity &e,
            const components::Visual *,
            const components::Geometry *geomComp) -> bool {
          const auto *matComp = ecm.Component<components::Material>(e);
          const sdf::Material *sdfMat = matComp ? &matComp->Data() : nullptr;
          SendVisual(e, geomComp->Data(), sdfMat, ecm);
          return true;
        });

    gzmsg << "[ExternalRendererSystem] Sent snapshot: " << sentVisuals.size()
          << " visuals, " << sentLights.size() << " sdf lights, "
          << configRectLights.size() << " area lights, " << sentMeshes.size()
          << " streamed submeshes, " << sentTextures.size() << " streamed textures (SHM="
          << (zenoh.HasShm() ? "on" : "off") << ")" << std::endl;
  }
};

ExternalRendererSystem::ExternalRendererSystem()
    : dataPtr(std::make_unique<ExternalRendererSystemPrivate>())
{
}

ExternalRendererSystem::~ExternalRendererSystem() = default;

void ExternalRendererSystem::Configure(const Entity & /*_entity*/,
                                       const std::shared_ptr<const sdf::Element> &_sdf,
                                       EntityComponentManager & /*_ecm*/,
                                       EventManager & /*_eventMgr*/)
{
  this->dataPtr->sessionName = ext_render::DefaultSessionName();
  this->dataPtr->zenohEndpoint = ext_render::DefaultZenohEndpoint();

  if (_sdf)
  {
    if (_sdf->HasElement("session"))
    {
      this->dataPtr->sessionName = _sdf->Get<std::string>("session");
    }
    if (_sdf->HasElement("endpoint"))
    {
      this->dataPtr->zenohEndpoint = _sdf->Get<std::string>("endpoint");
    }
    if (_sdf->HasElement("camera_pose"))
    {
      this->dataPtr->configCameraPose = _sdf->Get<math::Pose3d>("camera_pose");
      this->dataPtr->hasConfigCameraPose = true;
    }
    if (_sdf->HasElement("camera_hfov"))
    {
      double hfov = _sdf->Get<double>("camera_hfov");
      if (hfov > 3.2) hfov = hfov * 3.14159265358979323846 / 180.0;
      this->dataPtr->configCameraHfov = static_cast<float>(hfov);
    }
    if (_sdf->HasElement("environment_map"))
    {
      this->dataPtr->configEnvMapUri = _sdf->Get<std::string>("environment_map");
    }
    if (_sdf->HasElement("environment_intensity"))
    {
      this->dataPtr->configEnvMapIntensity =
          static_cast<float>(_sdf->Get<double>("environment_intensity"));
    }
    if (_sdf->HasElement("sky_intensity"))
    {
      this->dataPtr->configSkyIntensity =
          static_cast<float>(_sdf->Get<double>("sky_intensity"));
      this->dataPtr->hasConfigSkyIntensity = true;
    }

    // Parse physical rectangular area lights (<area_light> defaults to cast_shadows=true;
    // <rect_light> defaults to cast_shadows=false for legacy Bevy LTC configs).
    uint64_t rectIdx = 0;
    auto parseAreaLights = [&](const std::string &tagName, bool defaultShadows) {
      for (auto elem = _sdf->FindElement(tagName); elem != nullptr;
           elem = elem->GetNextElement(tagName))
      {
        ExternalRendererSystemPrivate::RectLightConfig rl{};
        rl.id = 0xBEEF000000000000ULL | (++rectIdx);
        rl.castShadows = defaultShadows;
        if (elem->HasElement("pose"))
        {
          rl.pose = elem->Get<math::Pose3d>("pose");
        }
        if (elem->HasElement("size"))
        {
          auto sz = elem->Get<math::Vector2d>("size");
          rl.width = static_cast<float>(sz.X());
          rl.height = static_cast<float>(sz.Y());
        }
        if (elem->HasElement("width"))
        {
          rl.width = static_cast<float>(elem->Get<double>("width"));
        }
        if (elem->HasElement("height"))
        {
          rl.height = static_cast<float>(elem->Get<double>("height"));
        }
        if (elem->HasElement("power"))
        {
          rl.intensity = static_cast<float>(elem->Get<double>("power"));
        }
        else if (elem->HasElement("intensity"))
        {
          rl.intensity = static_cast<float>(elem->Get<double>("intensity"));
        }
        if (elem->HasElement("spread"))
        {
          double sp = elem->Get<double>("spread");
          if (sp > 3.2) sp = sp * 3.14159265358979323846 / 180.0;
          rl.spread = static_cast<float>(sp);
        }
        if (elem->HasElement("cast_shadows"))
        {
          rl.castShadows = elem->Get<bool>("cast_shadows");
        }
        if (elem->HasElement("range"))
        {
          rl.range = static_cast<float>(elem->Get<double>("range"));
        }
        if (elem->HasElement("color"))
        {
          auto c = elem->Get<math::Vector3d>("color");
          rl.color[0] = static_cast<float>(c.X());
          rl.color[1] = static_cast<float>(c.Y());
          rl.color[2] = static_cast<float>(c.Z());
        }
        else if (elem->HasElement("diffuse"))
        {
          auto c = elem->Get<math::Color>("diffuse");
          rl.color[0] = c.R();
          rl.color[1] = c.G();
          rl.color[2] = c.B();
        }
        this->dataPtr->configRectLights.push_back(rl);
      }
    };
    parseAreaLights("rect_light", false);
    parseAreaLights("area_light", true);
  }

  // Open Zenoh peer connecting to Bevy's listener with 128 MB POSIX SHM pool for heavy meshes/textures
  if (!this->dataPtr->zenoh.Open(this->dataPtr->zenohEndpoint, false, 128 * 1024 * 1024))
  {
    gzerr << "[ExternalRendererSystem] Could not open Zenoh session on "
          << this->dataPtr->zenohEndpoint << std::endl;
    return;
  }

  const std::string prefix = "gz_ext/" + this->dataPtr->sessionName;
  this->dataPtr->scenePub = this->dataPtr->zenoh.DeclarePublisher(prefix + "/scene/events");
  this->dataPtr->zenoh.DeclareSubscriber(
      prefix + "/scene/snapshot_req",
      [this](const uint8_t *, size_t, bool) {
        this->dataPtr->snapshotRequested.store(true, std::memory_order_release);
      });

  this->dataPtr->configured = true;
  this->dataPtr->lastTransformSend = std::chrono::steady_clock::now();
  this->dataPtr->lastPeriodicCheck = std::chrono::steady_clock::now();

  gzmsg << "[ExternalRendererSystem] Configured session='" << this->dataPtr->sessionName
        << "' endpoint='" << this->dataPtr->zenohEndpoint
        << "' SHM=" << (this->dataPtr->zenoh.HasShm() ? "enabled" : "disabled") << std::endl;
}

void ExternalRendererSystem::PostUpdate(const UpdateInfo &_info,
                                        const EntityComponentManager &_ecm)
{
  if (!this->dataPtr->configured || !this->dataPtr->scenePub) return;

  // 1. Handle late-joiner SnapshotRequest from Bevy
  if (this->dataPtr->snapshotRequested.exchange(false, std::memory_order_acq_rel))
  {
    this->dataPtr->SendFullSnapshot(_ecm);
    return;
  }

  // 2. Incrementally sync newly spawned visuals & lights
  _ecm.EachNew<components::Light>(
      [&](const Entity &e, const components::Light *lightComp) -> bool {
        this->dataPtr->SendLight(e, lightComp->Data(), _ecm);
        return true;
      });

  _ecm.EachNew<components::Visual, components::Geometry>(
      [&](const Entity &e,
          const components::Visual *,
          const components::Geometry *geomComp) -> bool {
        const auto *matComp = _ecm.Component<components::Material>(e);
        const sdf::Material *sdfMat = matComp ? &matComp->Data() : nullptr;
        this->dataPtr->SendVisual(e, geomComp->Data(), sdfMat, _ecm);
        return true;
      });

  // 3. Despawn removed visuals & lights
  _ecm.EachRemoved<components::Visual>(
      [&](const Entity &e, const components::Visual *) -> bool {
        this->dataPtr->sentVisuals.erase(e);
        this->dataPtr->lastPoses.erase(e);
        ext_render::DespawnEntityWire de{static_cast<uint64_t>(e)};
        this->dataPtr->builder.Reset(ext_render::MsgType::DespawnEntity, 1);
        this->dataPtr->builder.AppendPod(de);
        this->dataPtr->scenePub->Publish(this->dataPtr->builder.Finish(), false);
        return true;
      });

  _ecm.EachRemoved<components::Light>(
      [&](const Entity &e, const components::Light *) -> bool {
        this->dataPtr->sentLights.erase(e);
        ext_render::DespawnEntityWire de{static_cast<uint64_t>(e)};
        this->dataPtr->builder.Reset(ext_render::MsgType::DespawnEntity, 1);
        this->dataPtr->builder.AppendPod(de);
        this->dataPtr->scenePub->Publish(this->dataPtr->builder.Finish(), false);
        return true;
      });

  // 4. Stream dirty world transforms at up to ~60Hz wall-clock
  const auto now = std::chrono::steady_clock::now();
  if (now - this->dataPtr->lastTransformSend < std::chrono::milliseconds(16))
  {
    return;
  }
  this->dataPtr->lastTransformSend = now;

  std::vector<ext_render::TransformItemWire> dirty;
  for (Entity e : this->dataPtr->sentVisuals)
  {
    const math::Pose3d wp = worldPose(e, _ecm);
    auto it = this->dataPtr->lastPoses.find(e);
    if (it == this->dataPtr->lastPoses.end() || PoseChanged(wp, it->second))
    {
      this->dataPtr->lastPoses[e] = wp;
      ext_render::TransformItemWire item{};
      item.entity_id = static_cast<uint64_t>(e);
      item.pose = ToPoseWire(wp);
      dirty.push_back(item);
    }
  }

  if (!dirty.empty())
  {
    ext_render::TransformBatchHeaderWire bh{};
    bh.sim_time = std::chrono::duration<double>(_info.simTime).count();
    this->dataPtr->builder.Reset(ext_render::MsgType::TransformBatch,
                                 static_cast<uint32_t>(dirty.size()));
    this->dataPtr->builder.AppendPod(bh);
    this->dataPtr->builder.AppendBytes(dirty.data(),
                                       dirty.size() * sizeof(ext_render::TransformItemWire));
    this->dataPtr->scenePub->Publish(this->dataPtr->builder.Finish(), true);
  }
}

}  // namespace gz::sim::systems

GZ_ADD_PLUGIN(
    gz::sim::systems::ExternalRendererSystem,
    gz::sim::System,
    gz::sim::systems::ExternalRendererSystem::ISystemConfigure,
    gz::sim::systems::ExternalRendererSystem::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(
    gz::sim::systems::ExternalRendererSystem,
    "gz::sim::systems::ExternalRendererSystem")
