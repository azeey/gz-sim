# Gazebo External Renderer (Zenoh SHM + Metal IOSurface + Qt 6 — Blender 5.0 & Bevy 0.19)

This directory contains an interprocess external rendering architecture for Gazebo (`gz-sim` + `gz-gui`) supporting pluggable external renderer processes over **Zenoh 1.8 POSIX Shared Memory** and **triple-buffered macOS Metal `IOSurface`** zero-copy GPU texture sharing:

* **Blender 5.0 External Renderer** (`blender_renderer/` + `blender_bridge/`): Photorealistic **Cycles Metal GPU** (path tracing + OpenImageDenoise GPU) and **EEVEE Metal** (real-time raytraced GI/reflections) with **AgX** filmic color management, adaptive interactive-to-photoreal refinement, native `.glb` collection instancing, `--show-blender` debug window mode, and live `.blend` scene export.
* **Bevy 0.19 External Renderer** (`bevy_renderer/`): Real-time Rust/`wgpu` Metal renderer.

## Architecture

```
Process A: gz sim server (C++)                 Process B: gz_blender_renderer (Blender 5.0 Metal)
┌──────────────────────────────────┐           ┌────────────────────────────────────────────┐
│ ExternalRendererSystem (ECM)     │           │ Native Bridge (libgz_blender_bridge.dylib) │
│  - Typed primitives (9 kinds)    │  Zenoh    │  - GIL-free Zenoh 1.8 SHM subscriber queue │
│  - Resolved .glb/.gltf URIs      │  POSIX    │  - Mach bootstrap 3x IOSurface server      │
│  - gz-common raw mesh + PBR tex  │── SHM ───>│                                            │
│  - 60Hz dirty TransformBatch     │           │ Blender 5.0 Scene & Photoreal Pipeline     │
│  - Late-joiner Snapshot handler  │<─ req ────│  - Direct Z-up sync + glTF Collection Inst │
└──────────────────────────────────┘           │  - Principled BSDF + packed PBR textures   │
                                               │  - EEVEE Metal (RT/GI) & Cycles Metal GPU  │
Process C: gz sim GUI (Qt 6 Metal)             │  - Adaptive interactive -> settled refine  │
┌──────────────────────────────────┐           └───────────────┬──────────────┬─────────────┘
│ ExternalRendererGui (QQuickItem) │  3x IOSurface Mach ports  │  Zenoh       │
│  - MetalTextureBridge            │<── (once per resize) ─────┘  FrameReady  │
│    QSGMetalTexture::fromNative   │<── {gen, slot, spp, ms} ─────────────────┘
│  - Adaptive Blender/Bevy HUD     │─── GuiInput + SlotAcquired ──────────────>
│  - Connect-or-auto-spawn backend │
└──────────────────────────────────┘
```

## Building

### Build the Gazebo Sim Plugin, Qt 6 Metal GUI Plugin, & Blender Native Bridge
```bash
cmake -G Ninja -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build
```
This builds:
* `build/libgz-sim-external-renderer-system.dylib` — Gazebo Sim server ECM sync plugin
* `build/libExternalRendererGui.dylib` — Qt 6 Metal zero-copy viewport & adaptive HUD plugin
* `build/libgz_blender_bridge.dylib` — Native Zenoh SHM + Mach `IOSurface` swapchain bridge loaded into Blender 5.0 via `ctypes`
* `build/test_gpu_ipc_consumer` — Cross-process `IOSurface` → `MTLTexture` verifier

### (Optional) Build the Bevy Renderer (Rust)
```bash
cargo build --release --manifest-path bevy_renderer/Cargo.toml
```

## Running Any Gazebo World with Blender 5.0

Requires Blender 5.0 installed at `/Applications/Blender.app` (or set `GZ_BLENDER_BIN`).

```bash
# Default: EEVEE Metal (interactive 4 spp -> settled 16 spp + RT/GI + AgX), switchable to Cycles live in HUD
./scripts/run_with_blender.sh shapes.sdf

# Start directly in Cycles Metal GPU path tracing (adaptive 2 spp @ 55% -> 16 spp -> 64 spp + OIDN GPU)
./scripts/run_with_blender.sh --engine cycles /Users/addisuzt/ws/rotary/scratch/gltf_comparison/damaged_helmet.sdf

# Blender-tuned warehouse (native physical Rectangular Area Lights + pure-interior Cycles/EEVEE setup)
./scripts/run_with_blender.sh --engine cycles \
  /Users/addisuzt/ws/rotary/scratch/warehouse_case_study/study/warehouse_blender.sdf -r

# Legacy Filament warehouse (glTF + streamed PBR meshes + 22 physical spotlights)
GZ_SIM_RESOURCE_PATH="/Users/addisuzt/ws/rotary/scratch/warehouse_case_study/gazebo_simple_warehouse/models" \
  ./scripts/run_with_blender.sh --engine cycles \
  /Users/addisuzt/ws/rotary/scratch/warehouse_case_study/study/warehouse_photoreal.sdf

# Debug mode: open Blender's live window alongside Gazebo's Qt GUI to inspect/tweak the synced scene
./scripts/run_with_blender.sh --show-blender shapes.sdf
```

To run the Blender renderer manually in a separate terminal (for live logs or fast Python iteration without restarting `gz sim`):
```bash
# Terminal 1
./blender_renderer/gz_blender_renderer --session default --endpoint tcp/127.0.0.1:17447 --engine cycles --verbose

# Terminal 2
GZ_EXT_RENDER_MANUAL=1 ./scripts/run_with_blender.sh shapes.sdf
```

### Viewport Controls & Blender HUD
* **Left-drag**: Orbit camera | **Right-drag / Middle-drag / Shift+Left-drag**: Pan | **Wheel**: Zoom
* **R**: Reset camera | **P**: Save lossless PNG screenshot | **B**: Save live scene as `gz_blender_scene.blend`
* **1–5** (or HUD chips): Switch **EEVEE / Cycles GPU**, toggle **IBL/Sky** (Multiple-Scattering physical sky / HDRI), **RT/GI** (EEVEE raytracing / Cycles multi-bounce GI), **Denoise** (OpenImageDenoise GPU), **AgX**, **Shadows**, or click **Interactive** / **Photoreal** / **Save .blend**.
