#ifndef GZ_EXTERNAL_RENDERER_ZENOH_TRANSPORT_HH_
#define GZ_EXTERNAL_RENDERER_ZENOH_TRANSPORT_HH_

#ifndef Z_FEATURE_SHARED_MEMORY
#define Z_FEATURE_SHARED_MEMORY
#endif
#ifndef Z_FEATURE_UNSTABLE_API
#define Z_FEATURE_UNSTABLE_API
#endif

#include <zenoh.h>

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace gz::ext_render
{

inline std::string DefaultSessionName()
{
  if (const char *env = std::getenv("GZ_EXT_RENDER_SESSION"))
  {
    if (env[0] != '\0') return std::string(env);
  }
  return "default";
}

inline std::string DefaultZenohEndpoint()
{
  if (const char *env = std::getenv("GZ_EXT_RENDER_ENDPOINT"))
  {
    if (env[0] != '\0') return std::string(env);
  }
  return "tcp/127.0.0.1:17447";
}

// Thin RAII wrapper over Zenoh 1.8 C API with POSIX Shared Memory zero-copy publishing & receiving.
class ZenohNode
{
 public:
  using SampleCallback = std::function<void(const uint8_t *data, size_t len, bool is_shm)>;

  ZenohNode() = default;
  ~ZenohNode() { Close(); }

  ZenohNode(const ZenohNode &) = delete;
  ZenohNode &operator=(const ZenohNode &) = delete;

  bool Open(const std::string &endpoint, bool listen_mode, size_t shm_pool_bytes = 64 * 1024 * 1024)
  {
    z_owned_config_t cfg;
    z_config_default(&cfg);
    zc_config_insert_json5(z_loan_mut(cfg), "mode", "\"peer\"");
    zc_config_insert_json5(z_loan_mut(cfg), "scouting/multicast/enabled", "false");
    zc_config_insert_json5(z_loan_mut(cfg), "transport/shared_memory/enabled", "true");

    const std::string ep_json = "[\"" + endpoint + "\"]";
    if (listen_mode)
    {
      zc_config_insert_json5(z_loan_mut(cfg), "listen/endpoints", ep_json.c_str());
    }
    else
    {
      zc_config_insert_json5(z_loan_mut(cfg), "connect/endpoints", ep_json.c_str());
    }

    if (z_open(&session_, z_move(cfg), nullptr) < 0)
    {
      std::cerr << "[ExtRender/Zenoh] Failed to open Zenoh session on " << endpoint << std::endl;
      return false;
    }
    open_ = true;

    if (shm_pool_bytes > 0)
    {
      z_alloc_alignment_t align = {3};  // 8-byte alignment for zero-copy casts
      if (z_memory_layout_new(&shm_layout_, shm_pool_bytes, align) == Z_OK)
      {
        if (z_posix_shm_provider_with_layout_new(&shm_provider_, z_loan(shm_layout_)) == Z_OK)
        {
          has_shm_ = true;
        }
        else
        {
          z_drop(z_move(shm_layout_));
        }
      }
    }
    return true;
  }

  void Close()
  {
    if (!open_) return;
    {
      std::lock_guard<std::mutex> lk(sub_mtx_);
      for (auto &sub : subs_)
      {
        z_drop(z_move(sub->handle));
      }
      subs_.clear();
    }
    {
      std::lock_guard<std::mutex> lk(pub_mtx_);
      for (auto &pub : pubs_)
      {
        z_drop(z_move(pub->handle));
      }
      pubs_.clear();
    }
    if (has_shm_)
    {
      z_drop(z_move(shm_provider_));
      z_drop(z_move(shm_layout_));
      has_shm_ = false;
    }
    z_drop(z_move(session_));
    open_ = false;
  }

  bool IsOpen() const { return open_; }
  bool HasShm() const { return has_shm_; }

  class Publisher
  {
   public:
    bool Publish(const uint8_t *data, size_t len, bool prefer_shm = true)
    {
      if (!node_ || !node_->open_) return false;
      std::lock_guard<std::mutex> lk(node_->shm_mtx_);
      z_owned_bytes_t payload;
      bool used_shm = false;
      if (prefer_shm && node_->has_shm_ && len > 0)
      {
        z_buf_layout_alloc_result_t res;
        z_shm_provider_alloc_gc_defrag(&res, z_loan(node_->shm_provider_), len);
        if (res.status == ZC_BUF_LAYOUT_ALLOC_STATUS_OK)
        {
          std::memcpy(z_shm_mut_data_mut(z_loan_mut(res.buf)), data, len);
          z_bytes_from_shm_mut(&payload, z_move(res.buf));
          used_shm = true;
        }
      }
      if (!used_shm)
      {
        z_bytes_copy_from_buf(&payload, data, len);
      }
      return z_publisher_put(z_loan(handle), z_move(payload), nullptr) == Z_OK;
    }

    bool Publish(const std::vector<uint8_t> &buf, bool prefer_shm = true)
    {
      return Publish(buf.data(), buf.size(), prefer_shm);
    }

   private:
    friend class ZenohNode;
    ZenohNode *node_ = nullptr;
    z_owned_publisher_t handle{};
  };

  std::shared_ptr<Publisher> DeclarePublisher(const std::string &key_expr)
  {
    if (!open_) return nullptr;
    auto pub = std::make_shared<Publisher>();
    pub->node_ = this;
    z_view_keyexpr_t ke;
    z_view_keyexpr_from_str(&ke, key_expr.c_str());
    if (z_declare_publisher(z_loan(session_), &pub->handle, z_loan(ke), nullptr) < 0)
    {
      std::cerr << "[ExtRender/Zenoh] Failed to declare publisher on " << key_expr << std::endl;
      return nullptr;
    }
    std::lock_guard<std::mutex> lk(pub_mtx_);
    pubs_.push_back(pub);
    return pub;
  }

  bool DeclareSubscriber(const std::string &key_expr, SampleCallback cb)
  {
    if (!open_) return false;
    auto sub = std::make_unique<SubState>();
    sub->cb = std::move(cb);

    z_owned_closure_sample_t closure;
    z_closure(&closure, &ZenohNode::OnSampleTrampoline, nullptr, sub.get());

    z_view_keyexpr_t ke;
    z_view_keyexpr_from_str(&ke, key_expr.c_str());
    if (z_declare_subscriber(z_loan(session_), &sub->handle, z_loan(ke), z_move(closure), nullptr) < 0)
    {
      std::cerr << "[ExtRender/Zenoh] Failed to declare subscriber on " << key_expr << std::endl;
      return false;
    }
    std::lock_guard<std::mutex> lk(sub_mtx_);
    subs_.push_back(std::move(sub));
    return true;
  }

 private:
  struct SubState
  {
    z_owned_subscriber_t handle{};
    SampleCallback cb;
  };

  static void OnSampleTrampoline(z_loaned_sample_t *sample, void *ctx)
  {
    auto *state = static_cast<SubState *>(ctx);
    if (!state || !state->cb) return;
    const z_loaned_bytes_t *payload = z_sample_payload(sample);
    const z_loaned_shm_t *shm = nullptr;
    if (z_bytes_as_loaned_shm(payload, &shm) == Z_OK && shm != nullptr)
    {
      state->cb(z_shm_data(shm), z_shm_len(shm), true);
      return;
    }
    z_owned_slice_t slice;
    if (z_bytes_to_slice(payload, &slice) == Z_OK)
    {
      const z_loaned_slice_t *loaned = z_loan(slice);
      state->cb(z_slice_data(loaned), z_slice_len(loaned), false);
      z_drop(z_move(slice));
    }
  }

  bool open_ = false;
  bool has_shm_ = false;
  z_owned_session_t session_{};
  z_owned_memory_layout_t shm_layout_{};
  z_owned_shm_provider_t shm_provider_{};
  std::mutex shm_mtx_;
  std::mutex pub_mtx_;
  std::mutex sub_mtx_;
  std::vector<std::shared_ptr<Publisher>> pubs_;
  std::vector<std::unique_ptr<SubState>> subs_;
};

}  // namespace gz::ext_render

#endif  // GZ_EXTERNAL_RENDERER_ZENOH_TRANSPORT_HH_
