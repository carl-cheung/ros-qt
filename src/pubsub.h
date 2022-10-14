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
