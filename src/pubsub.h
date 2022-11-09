/********************************************* pubsub.h *********************************************
Overview:
	Setups the variables, functions, slots and signals needed for a small ROS node

Functions/Slots:
	PubSub(): Pubsub object constructor
	~PubSub(): Pubsub Obejct deconstructor
	ros_spin(): does a rosspin() to fetch subscribed topic values
	setRosTimerIntervalSlot(): Will set ros timer interval for rosspin
	run(): runs the Qthread initially

Description:
	This PubSub class is subclassed as a QThread so that it can run in the background simultaneously
	to the UI.
********************************************************************************************************/

#pragma once
#include <QObject>
#include <QThread>
#include <QTimer>
#include <ros/ros.h>

class PubSub : public QThread {
	Q_OBJECT

public:
	ros::NodeHandle n;

	PubSub ();
	~PubSub ();

private slots:
	void ros_spin();
	void setRosTimerIntervalSlot(int interval){rosTimer->setInterval(interval);};
	void startRosTimerSlot() { rosTimer->start(); };

private:
	void run();
	QTimer* rosTimer = new QTimer;

signals:
	void setRosTimerInterval(int interval);
	void startRosTimer();
};
