/******************************************** ros.qml ********************************************
Overview:
	This singleton object will be the same throughout qml and can be used to synchronise
	our QML application with ROS Subscribers and Pubishers.
	Settings can also be stored using QtLabs Settings so all values can be synced throughout.
************************************************************************************************/

pragma Singleton
import QtQuick 2.9
import Ros.Communication 1.0
import Qt.labs.settings 1.0

Item {
	//Publishers

	//Subscribers
}