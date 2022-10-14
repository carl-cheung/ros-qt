#include <QtWidgets/QApplication>
#include "pubsub.h"

PubSub::PubSub () { }

PubSub::~PubSub() { ros::shutdown(); quit(); wait(); }

void PubSub::run() {

	connect(rosTimer, SIGNAL(timeout()),this,SLOT(ros_spin()));
	connect(this, SIGNAL(startRosTimer()),this,SLOT(startRosTimerSlot()));
	connect(this, SIGNAL(setRosTimerInterval(int)),this,SLOT(setRosTimerIntervalSlot(int)));
	emit setRosTimerInterval(100);
	emit startRosTimer();

	exec();
	return;
}

void PubSub::ros_spin() {
	if (!ros::ok()){ QApplication::quit(); }
	ros::spinOnce();
	return;
}