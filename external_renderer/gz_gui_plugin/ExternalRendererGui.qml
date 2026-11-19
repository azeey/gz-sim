import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import ExternalRenderer 1.0

Rectangle {
  id: root
  color: "#1a1d24"
  Layout.minimumWidth: 320
  Layout.minimumHeight: 240
  anchors.fill: parent

  onParentChanged: {
    if (undefined === parent)
      return;
    width = Qt.binding(function() { return parent.parent.width })
    height = Qt.binding(function() { return parent.parent.height })
  }

  ExternalViewport {
    id: viewport
    objectName: "externalViewport"
    anchors.fill: parent
  }

  // Self-styled compact HUD button (avoids Qt Material Button's oversized insets)
  component HudChip: Rectangle {
    id: chip
    property string label: ""
    property bool checkable: false
    property bool checked: false
    property bool accent: false
    signal clicked()

    implicitWidth: Math.max(44, chipText.implicitWidth + 18)
    implicitHeight: 26
    radius: 5
    color: {
      if (chipMouse.pressed) return "#2f8aa8"
      if (checkable && checked) return chipMouse.containsMouse ? "#24809e" : "#1b6b85"
      if (accent) return chipMouse.containsMouse ? "#3b4c6b" : "#2d3b55"
      return chipMouse.containsMouse ? "#343c4d" : "#252b37"
    }
    border.width: 1
    border.color: (checkable && checked) ? "#67e8f9" : "#4b5563"

    Text {
      id: chipText
      anchors.centerIn: parent
      text: chip.label
      color: (chip.checkable && chip.checked) ? "#ffffff" : "#e2e8f0"
      font.pixelSize: 11
      font.weight: (chip.checkable && chip.checked) ? Font.DemiBold : Font.Normal
      renderType: Text.NativeRendering
    }

    MouseArea {
      id: chipMouse
      anchors.fill: parent
      hoverEnabled: true
      cursorShape: Qt.PointingHandCursor
      onClicked: chip.clicked()
    }
  }

  // Waiting banner before first frame arrives
  Rectangle {
    anchors.centerIn: parent
    visible: !viewport.connected
    z: 5
    color: "#cc161922"
    border.color: "#40ffffff"
    border.width: 1
    radius: 8
    width: waitLabel.implicitWidth + 32
    height: waitLabel.implicitHeight + 20

    Text {
      id: waitLabel
      anchors.centerIn: parent
      color: "#e6edf3"
      font.pixelSize: 13
      text: viewport.statusText
    }
  }

  // Collapsible HUD + Renderer Controls overlay
  property bool hudExpanded: true
  property string toastText: ""

  Timer {
    id: toastTimer
    interval: 2500
    onTriggered: root.toastText = ""
  }

  Rectangle {
    id: hudPanel
    z: 10
    anchors.top: parent.top
    anchors.right: parent.right
    anchors.margins: 10
    color: "#d9141820"
    border.color: "#4dffffff"
    border.width: 1
    radius: 8
    implicitWidth: hudCol.implicitWidth + 20
    implicitHeight: hudCol.implicitHeight + 16
    width: implicitWidth
    height: implicitHeight

    // Prevent clicks/drags on the HUD panel from orbiting the 3D viewport underneath
    MouseArea {
      anchors.fill: parent
      acceptedButtons: Qt.AllButtons
      onWheel: function(wheel) { wheel.accepted = true }
    }

    ColumnLayout {
      id: hudCol
      anchors.fill: parent
      anchors.margins: 8
      spacing: 7

      RowLayout {
        Layout.fillWidth: true
        spacing: 10

        Text {
          Layout.fillWidth: true
          color: "#9ef0ff"
          font.bold: true
          font.pixelSize: 12
          renderType: Text.NativeRendering
          text: {
            if (!viewport.connected)
              return viewport.statusText
            if (viewport.isBlender) {
              var modeTag = (viewport.qualityFlags & 1) ? "Cycles" : "EEVEE"
              var sppTag = viewport.sampleCount > 0 ? (viewport.sampleCount + " spp") : ""
              var refTag = viewport.refining ? " (refining...)" : ""
              return modeTag + " | " + viewport.renderMs.toFixed(0) + " ms | "
                     + sppTag + refTag + " | "
                     + viewport.viewportWidth + "x" + viewport.viewportHeight
                     + " | " + viewport.entityCount + " vis"
            }
            return viewport.fps.toFixed(0) + " FPS | " + viewport.renderMs.toFixed(1) + " ms | "
                   + viewport.viewportWidth + "x" + viewport.viewportHeight
                   + " | " + viewport.entityCount + " vis"
          }
        }

        HudChip {
          implicitWidth: 26
          implicitHeight: 22
          label: root.hudExpanded ? "−" : "+"
          onClicked: root.hudExpanded = !root.hudExpanded
        }
      }

      ColumnLayout {
        visible: root.hudExpanded
        spacing: 6

        Text {
          color: "#a8b3c7"
          font.pixelSize: 10
          renderType: Text.NativeRendering
          text: viewport.isBlender
                ? "Blender 5.0 Metal IOSurface (3x) + Zenoh SHM  |  LMB: Orbit  RMB/Shift: Pan  Wheel: Zoom"
                : "Metal IOSurface (3x) + Zenoh SHM  |  LMB: Orbit  RMB/Shift: Pan  Wheel: Zoom"
        }

        // --- Blender 5.0 Photorealism Controls ---
        ColumnLayout {
          visible: viewport.isBlender
          spacing: 6

          RowLayout {
            spacing: 5
            HudChip {
              label: "EEVEE"
              checkable: true
              checked: (viewport.qualityFlags & 1) === 0
              onClicked: viewport.setBlenderEngine(false)
            }
            HudChip {
              label: "Cycles GPU"
              checkable: true
              checked: (viewport.qualityFlags & 1) !== 0
              onClicked: viewport.setBlenderEngine(true)
            }
            HudChip {
              label: "IBL/Sky"
              checkable: true
              checked: (viewport.qualityFlags & 2) !== 0
              onClicked: viewport.toggleQualityFlag(2)
            }
            HudChip {
              label: "RT/GI"
              checkable: true
              checked: (viewport.qualityFlags & 4) !== 0
              onClicked: viewport.toggleQualityFlag(4)
            }
            HudChip {
              label: "Denoise"
              checkable: true
              checked: (viewport.qualityFlags & 8) !== 0
              onClicked: viewport.toggleQualityFlag(8)
            }
            HudChip {
              label: "AgX"
              checkable: true
              checked: (viewport.qualityFlags & 16) !== 0
              onClicked: viewport.toggleQualityFlag(16)
            }
            HudChip {
              label: "Shadows"
              checkable: true
              checked: (viewport.qualityFlags & 32) !== 0
              onClicked: viewport.toggleQualityFlag(32)
            }
          }

          RowLayout {
            spacing: 5
            HudChip {
              label: "Interactive"
              accent: true
              onClicked: viewport.setQualityPreset("interactive")
            }
            HudChip {
              label: "Photoreal"
              accent: true
              onClicked: viewport.setQualityPreset("photoreal")
            }
            HudChip {
              label: "Reset Cam"
              accent: true
              onClicked: viewport.resetCamera()
            }
            HudChip {
              label: "Screenshot"
              accent: true
              onClicked: {
                viewport.saveScreenshot()
                root.toastText = "Saved PNG screenshot to working directory"
                toastTimer.restart()
              }
            }
            HudChip {
              label: "Save .blend"
              accent: true
              onClicked: {
                viewport.saveScene()
                root.toastText = "Saved live scene to gz_blender_scene.blend"
                toastTimer.restart()
              }
            }
          }
        }

        // --- Bevy 0.19 Controls ---
        ColumnLayout {
          visible: !viewport.isBlender
          spacing: 6

          RowLayout {
            spacing: 5
            HudChip {
              label: "IBL"
              checkable: true
              checked: (viewport.qualityFlags & 1) !== 0
              onClicked: viewport.toggleQualityFlag(1)
            }
            HudChip {
              label: "SSAO+TAA"
              checkable: true
              checked: (viewport.qualityFlags & 2) !== 0
              onClicked: viewport.toggleQualityFlag(2)
            }
            HudChip {
              label: "Bloom"
              checkable: true
              checked: (viewport.qualityFlags & 4) !== 0
              onClicked: viewport.toggleQualityFlag(4)
            }
            HudChip {
              label: "AgX"
              checkable: true
              checked: (viewport.qualityFlags & 8) !== 0
              onClicked: viewport.toggleQualityFlag(8)
            }
            HudChip {
              label: "Shadows"
              checkable: true
              checked: (viewport.qualityFlags & 16) !== 0
              onClicked: viewport.toggleQualityFlag(16)
            }
          }

          RowLayout {
            spacing: 5
            HudChip {
              label: "Bevy PBR"
              accent: true
              onClicked: viewport.setQualityPreset("pbr")
            }
            HudChip {
              label: "Reset Cam"
              accent: true
              onClicked: viewport.resetCamera()
            }
            HudChip {
              label: "Screenshot"
              accent: true
              onClicked: {
                viewport.saveScreenshot()
                root.toastText = "Saved PNG screenshot to working directory"
                toastTimer.restart()
              }
            }
          }
        }

        Text {
          visible: root.toastText.length > 0
          color: "#86efac"
          font.pixelSize: 10
          renderType: Text.NativeRendering
          text: root.toastText
        }
      }
    }
  }
}
