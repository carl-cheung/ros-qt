import QtQuick 2.12
import QtCharts 2.3
import QtQuick.Window 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import QtQuick.Controls.Material 2.12
import QtQuick.VirtualKeyboard 2.4

import Ros.Communication 1.0

Window {
	id: window
	visible: true
	width: 640
	height: 480
	title: qsTr("ROS UI")
	Material.theme: Material.Dark
	Material.accent: Material.Cyan
	Component.onCompleted: ROS
	Flickable{
		id:flickable
		anchors { top: parent.top; left: parent.left; right: parent.right; bottom: inputPanel.top }
		boundsBehavior: Flickable.StopAtBounds
		contentWidth: window.width
		contentHeight: flow.height
		clip: true
		function scrollTo(y){
			scrollAnimation.to = y
			scrollAnimation.start()
		}
		NumberAnimation{
			id:scrollAnimation
			target: flickable
			property: "contentY"
			duration: 250
		}
		Flow{
			id: flow
			width: window.width
			//QML CONTROLS GO HERE IN THIS FLOW

		}
	}
	InputPanel {
		id: inputPanel
		z: 99
		x: 0
		y: window.height
		width: window.width

		onActiveChanged: {
			var globalCoordinates = activeFocusItem.mapToItem(flickable.parent,0,0)
			var yFromBottom = inputPanel.parent.height - globalCoordinates.y
			if (inputPanel.height+50 > yFromBottom && active){
				flickable.scrollTo(flickable.contentY + (inputPanel.height-yFromBottom)+50)
			}
		}

		states: State {
			name: "visible"
			when: inputPanel.active
			PropertyChanges {
				target: inputPanel
				y: window.height - inputPanel.height
			}
		}
		transitions: Transition {
			from: ""
			to: "visible"
			reversible: true
			ParallelAnimation {
				NumberAnimation {
					properties: "y"
					duration: 250
					easing.type: Easing.InOutQuad
				}
			}
		}
	}
}
