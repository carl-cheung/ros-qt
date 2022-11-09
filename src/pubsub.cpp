/********************************************* pubsub.h *********************************************
Overview:
	Setups the variables, functions, slots and signals needed for a small ROS node
	
Functions/Slots:
	PubSub(): Pubsub object constructor
	~PubSub(): Pubsub Obejct deconstructor
	run(): Sets up the timer, connecting the timeout to the correct slot and interval.
	ros_spin(): does a ros::spinOnce() to fetch subscribed topic values

Description:
	Contrsuctor does nothing currently
	Deconstructor shutsdown ros and quits.
	Run sets the timer timeout slots, interval and starts it. It then returns to pool with exec()
	ros_spin gets the values that it is subscribed to.

********************************************************************************************************/
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