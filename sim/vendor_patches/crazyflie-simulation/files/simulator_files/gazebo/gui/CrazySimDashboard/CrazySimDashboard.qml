import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Rectangle {
  id: root
  width: 300
  height: Math.max(120, content.implicitHeight + 28)
  radius: 8
  color: "#15191fcc"
  border.color: "#4e9cff"
  border.width: 1

  ColumnLayout {
    id: content
    anchors.fill: parent
    anchors.margins: 12
    spacing: 8

    RowLayout {
      Layout.fillWidth: true
      spacing: 8

      Rectangle {
        width: 8
        height: 8
        radius: 4
        color: "#38d973"
      }

      Text {
        Layout.fillWidth: true
        text: "CrazySim"
        color: "#f5f7fb"
        font.pixelSize: 14
        font.bold: true
      }
    }

    Rectangle {
      Layout.fillWidth: true
      height: 1
      color: "#ffffff22"
    }

    Text {
      Layout.fillWidth: true
      text: Dashboard ? Dashboard.text : "Dashboard not connected"
      color: "#d7deea"
      font.family: "monospace"
      font.pixelSize: 12
      lineHeight: 1.12
      wrapMode: Text.Wrap
    }

    Text {
      Layout.fillWidth: true
      text: Dashboard ? Dashboard.topic : ""
      color: "#8fa2bd"
      font.pixelSize: 10
      elide: Text.ElideRight
    }
  }
}
