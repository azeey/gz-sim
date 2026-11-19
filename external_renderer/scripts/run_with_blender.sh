#!/usr/bin/env bash
set -euo pipefail

# Launch any Gazebo SDF world using the external Blender 5.0 renderer (separate process)
# displaying via zero-copy Metal IOSurface inside Gazebo's Qt 6 GUI.
#
# Usage:
#   ./scripts/run_with_blender.sh [--engine eevee|cycles] [--quality balanced|photoreal] [--show-blender] [--verbose] <world.sdf> [extra gz sim args...]
#
# Environment overrides:
#   GZ_EXT_RENDER_SESSION=default
#   GZ_EXT_RENDER_ENDPOINT=tcp/127.0.0.1:17447
#   GZ_EXT_RENDER_MANUAL=1   # Set to 1 if you already started gz_blender_renderer manually
#   GZ_BLENDER_BIN=/Applications/Blender.app/Contents/MacOS/Blender

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_ROOT="$(cd "${EXT_DIR}/../../.." && pwd)"
BUILD_DIR="${EXT_DIR}/build"
BLENDER_WRAPPER="${EXT_DIR}/blender_renderer/gz_blender_renderer"

EXTRA_RENDER_ARGS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --engine|--quality|--save-blend)
      EXTRA_RENDER_ARGS+=("$1" "$2")
      shift 2
      ;;
    --show-blender|--verbose)
      EXTRA_RENDER_ARGS+=("$1")
      shift
      ;;
    -h|--help)
      echo "Usage: $0 [--engine eevee|cycles] [--quality balanced|photoreal] [--show-blender] [--verbose] <world.sdf> [gz sim args...]"
      exit 0
      ;;
    *)
      break
      ;;
  esac
done

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 [--engine eevee|cycles] [--quality balanced|photoreal] [--show-blender] [--verbose] <world.sdf> [gz sim args...]" >&2
  exit 1
fi

WORLD_IN="$1"
shift

export GZ_IP=127.0.0.1
export GZ_EXT_RENDER_SESSION="${GZ_EXT_RENDER_SESSION:-default}"
export GZ_EXT_RENDER_ENDPOINT="${GZ_EXT_RENDER_ENDPOINT:-tcp/127.0.0.1:17447}"
export GZ_EXT_RENDER_BIN="${BLENDER_WRAPPER}"
if [[ ${#EXTRA_RENDER_ARGS[@]} -gt 0 ]]; then
  export GZ_EXT_RENDER_ARGS="${EXTRA_RENDER_ARGS[*]}"
fi
export GZ_SIM_SYSTEM_PLUGIN_PATH="${BUILD_DIR}:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_GUI_PLUGIN_PATH="${BUILD_DIR}:${GZ_GUI_PLUGIN_PATH:-}"
export GZ_SIM_SERVER_CONFIG_PATH="${EXT_DIR}/config/bevy_server.config"

# Source workspace environment if not already active
if [[ -f "${WS_ROOT}/install/setup.sh" ]]; then
  set +u
  # shellcheck disable=SC1091
  source "${WS_ROOT}/install/setup.sh"
  set -u
fi
export PATH="${WS_ROOT}/.pixi/envs/default/bin:${WS_ROOT}/install/bin:/opt/homebrew/bin:${PATH}"

# Resolve world file (allow built-in gz-sim example worlds like shapes.sdf)
WORLD_PATH="${WORLD_IN}"
if [[ ! -f "${WORLD_PATH}" && -f "${WS_ROOT}/src/gz-sim/examples/worlds/${WORLD_IN}" ]]; then
  WORLD_PATH="${WS_ROOT}/src/gz-sim/examples/worlds/${WORLD_IN}"
fi

TMP_WORLD=""
TMP_GUI=""
cleanup() {
  [[ -n "${TMP_WORLD}" && -f "${TMP_WORLD}" ]] && rm -f "${TMP_WORLD}"
  [[ -n "${TMP_GUI}" && -f "${TMP_GUI}" ]] && rm -f "${TMP_GUI}"
}
trap cleanup EXIT

GUI_CFG="${EXT_DIR}/config/bevy_gui.config"
if [[ -f "${WORLD_PATH}" ]]; then
  WORLD_ABS="$(python3 -c 'import os,sys; print(os.path.abspath(sys.argv[1]))' "${WORLD_PATH}")"
  TMP_WORLD="/tmp/gz_blender_world_$$.sdf"
  TMP_GUI="/tmp/gz_blender_gui_$$.config"

  python3 - "${WORLD_ABS}" "${TMP_WORLD}" "${GUI_CFG}" "${TMP_GUI}" <<'PY'
import os, re, sys

src_sdf, dst_sdf, src_gui, dst_gui = sys.argv[1:5]
text = open(src_sdf, "r", encoding="utf-8").read()

cam_pose = None
m = re.search(r"<camera_pose>\s*([^<]+?)\s*</camera_pose>", text)
if m:
    cam_pose = m.group(1).strip()

hfov = None
m2 = re.search(r"<horizontal_fov>\s*([^<]+?)\s*</horizontal_fov>", text)
if not m2:
    m2 = re.search(r"<camera_hfov>\s*([^<]+?)\s*</camera_hfov>", text)
if m2:
    hfov = m2.group(1).strip()

text = re.sub(r"<gui\b[^>]*>.*?</gui>", "", text, flags=re.DOTALL)

if "ExternalRendererSystem" not in text:
    cam_tag = f"\n      <camera_pose>{cam_pose}</camera_pose>" if cam_pose else ""
    fov_tag = f"\n      <camera_hfov>{hfov}</camera_hfov>" if hfov else ""
    plugin_xml = (
        '\n    <plugin filename="gz-sim-external-renderer-system" '
        'name="gz::sim::systems::ExternalRendererSystem">'
        f"{cam_tag}{fov_tag}\n    </plugin>\n"
    )
    text = re.sub(r"(<world\b[^>]*>)", r"\1" + plugin_xml, text, count=1)

open(dst_sdf, "w", encoding="utf-8").write(text)

gui_text = open(src_gui, "r", encoding="utf-8").read()
if cam_pose:
    gui_text = gui_text.replace(
        "<auto_spawn>true</auto_spawn>",
        f"<auto_spawn>true</auto_spawn>\n  <camera_pose>{cam_pose}</camera_pose>",
        1,
    )
if hfov:
    gui_text = gui_text.replace(
        "<auto_spawn>true</auto_spawn>",
        f"<auto_spawn>true</auto_spawn>\n  <horizontal_fov>{hfov}</horizontal_fov>",
        1,
    )
if os.environ.get("GZ_EXT_RENDER_MANUAL", "0") == "1":
    gui_text = gui_text.replace("<auto_spawn>true</auto_spawn>", "<auto_spawn>false</auto_spawn>")
open(dst_gui, "w", encoding="utf-8").write(gui_text)
PY

  WORLD_DIR="$(dirname "${WORLD_ABS}")"
  EXTRA_RES="${WORLD_DIR}:${WORLD_DIR}/models"
  [[ -d "${WORLD_DIR}/upgraded_models" ]] && EXTRA_RES="${WORLD_DIR}/upgraded_models:${EXTRA_RES}"
  export GZ_SIM_RESOURCE_PATH="${EXTRA_RES}:${GZ_SIM_RESOURCE_PATH:-}"
  WORLD_TO_RUN="${TMP_WORLD}"
  GUI_CFG="${TMP_GUI}"
else
  WORLD_TO_RUN="${WORLD_PATH}"
fi

exec gz sim --gui-config "${GUI_CFG}" "${WORLD_TO_RUN}" "$@"
