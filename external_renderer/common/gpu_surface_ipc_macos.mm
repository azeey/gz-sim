#ifdef __APPLE__

#include "gpu_surface_ipc.hh"

#import <CoreFoundation/CoreFoundation.h>
#import <IOSurface/IOSurface.h>
#include <mach/mach.h>
#include <servers/bootstrap.h>

#include <cstring>
#include <iostream>

namespace gz::ext_render
{

namespace
{

constexpr mach_msg_id_t kMsgIdFetchSwapchainReq = 0x475A0001;
constexpr mach_msg_id_t kMsgIdFetchSwapchainRep = 0x475A0002;
constexpr mach_msg_id_t kMsgIdPingReq = 0x475A0003;
constexpr mach_msg_id_t kMsgIdPingRep = 0x475A0004;

struct FetchRequestMsg
{
  mach_msg_header_t header;
  uint32_t requested_generation;
  uint32_t reserved;
};

struct FetchReplyMsg
{
  mach_msg_header_t header;
  mach_msg_body_t body;
  mach_msg_port_descriptor_t ports[kSwapchainSlots];
  uint32_t generation;
  uint32_t width;
  uint32_t height;
  uint32_t pixel_format;
  uint32_t status;  // 0 = OK
  uint32_t reserved;
  mach_msg_trailer_t trailer;
};

struct PingReplyMsg
{
  mach_msg_header_t header;
  uint32_t generation;
  uint32_t status;
  mach_msg_trailer_t trailer;
};

}  // namespace

bool IsExternalRendererAlive(const std::string &service_name)
{
  mach_port_t server_port = MACH_PORT_NULL;
  kern_return_t kr = bootstrap_look_up(bootstrap_port, service_name.c_str(), &server_port);
  if (kr != KERN_SUCCESS || server_port == MACH_PORT_NULL)
  {
    return false;
  }

  mach_port_t reply_port = MACH_PORT_NULL;
  if (mach_port_allocate(mach_task_self(), MACH_PORT_RIGHT_RECEIVE, &reply_port) != KERN_SUCCESS)
  {
    mach_port_deallocate(mach_task_self(), server_port);
    return false;
  }

  FetchRequestMsg req{};
  req.header.msgh_bits = MACH_MSGH_BITS(MACH_MSG_TYPE_COPY_SEND, MACH_MSG_TYPE_MAKE_SEND_ONCE);
  req.header.msgh_size = sizeof(req);
  req.header.msgh_remote_port = server_port;
  req.header.msgh_local_port = reply_port;
  req.header.msgh_id = kMsgIdPingReq;

  bool alive = false;
  kr = mach_msg(&req.header, MACH_SEND_MSG | MACH_SEND_TIMEOUT, sizeof(req), 0,
                MACH_PORT_NULL, 250, MACH_PORT_NULL);
  if (kr == KERN_SUCCESS)
  {
    PingReplyMsg rep{};
    kr = mach_msg(&rep.header, MACH_RCV_MSG | MACH_RCV_TIMEOUT, 0, sizeof(rep),
                  reply_port, 250, MACH_PORT_NULL);
    if (kr == KERN_SUCCESS && rep.header.msgh_id == kMsgIdPingRep && rep.status == 0)
    {
      alive = true;
    }
  }

  mach_port_destruct(mach_task_self(), reply_port, 0, 0);
  mach_port_deallocate(mach_task_self(), server_port);
  return alive;
}

bool FetchExternalSwapchain(
    const std::string &service_name,
    uint32_t requested_generation,
    ImportedSwapchainInfo &out_info,
    uint32_t timeout_ms)
{
  mach_port_t server_port = MACH_PORT_NULL;
  kern_return_t kr = bootstrap_look_up(bootstrap_port, service_name.c_str(), &server_port);
  if (kr != KERN_SUCCESS || server_port == MACH_PORT_NULL)
  {
    return false;
  }

  mach_port_t reply_port = MACH_PORT_NULL;
  kr = mach_port_allocate(mach_task_self(), MACH_PORT_RIGHT_RECEIVE, &reply_port);
  if (kr != KERN_SUCCESS)
  {
    mach_port_deallocate(mach_task_self(), server_port);
    return false;
  }

  FetchRequestMsg req{};
  req.header.msgh_bits = MACH_MSGH_BITS(MACH_MSG_TYPE_COPY_SEND, MACH_MSG_TYPE_MAKE_SEND_ONCE);
  req.header.msgh_size = sizeof(req);
  req.header.msgh_remote_port = server_port;
  req.header.msgh_local_port = reply_port;
  req.header.msgh_id = kMsgIdFetchSwapchainReq;
  req.requested_generation = requested_generation;

  kr = mach_msg(&req.header, MACH_SEND_MSG | MACH_SEND_TIMEOUT, sizeof(req), 0,
                MACH_PORT_NULL, timeout_ms, MACH_PORT_NULL);
  if (kr != KERN_SUCCESS)
  {
    mach_port_destruct(mach_task_self(), reply_port, 0, 0);
    mach_port_deallocate(mach_task_self(), server_port);
    return false;
  }

  FetchReplyMsg rep{};
  kr = mach_msg(&rep.header, MACH_RCV_MSG | MACH_RCV_TIMEOUT, 0, sizeof(rep),
                reply_port, timeout_ms, MACH_PORT_NULL);
  mach_port_destruct(mach_task_self(), reply_port, 0, 0);
  mach_port_deallocate(mach_task_self(), server_port);

  if (kr != KERN_SUCCESS || rep.header.msgh_id != kMsgIdFetchSwapchainRep || rep.status != 0)
  {
    return false;
  }

  if (rep.body.msgh_descriptor_count != kSwapchainSlots)
  {
    for (uint32_t i = 0; i < rep.body.msgh_descriptor_count && i < kSwapchainSlots; ++i)
    {
      if (rep.ports[i].name != MACH_PORT_NULL)
        mach_port_deallocate(mach_task_self(), rep.ports[i].name);
    }
    return false;
  }

  ImportedSwapchainInfo imported{};
  imported.generation = rep.generation;
  imported.width = rep.width;
  imported.height = rep.height;
  imported.pixel_format = rep.pixel_format;

  bool ok = true;
  for (uint32_t i = 0; i < kSwapchainSlots; ++i)
  {
    mach_port_t port = rep.ports[i].name;
    IOSurfaceRef surf = (port != MACH_PORT_NULL) ? IOSurfaceLookupFromMachPort(port) : nullptr;
    if (port != MACH_PORT_NULL)
    {
      mach_port_deallocate(mach_task_self(), port);
    }
    if (!surf)
    {
      ok = false;
    }
    imported.native_handles[i] = static_cast<void *>(surf);
  }

  if (!ok)
  {
    ReleaseImportedSwapchain(imported);
    return false;
  }

  ReleaseImportedSwapchain(out_info);
  out_info = imported;
  return true;
}

void ReleaseImportedSwapchain(ImportedSwapchainInfo &info)
{
  for (void *&h : info.native_handles)
  {
    if (h != nullptr)
    {
      CFRelease(static_cast<IOSurfaceRef>(h));
      h = nullptr;
    }
  }
  info.generation = 0;
  info.width = 0;
  info.height = 0;
}

}  // namespace gz::ext_render

#endif  // __APPLE__
