/********************************************* publisher.hpp *********************************************
Overview:
	Definitions of all QObject types to be registered as a qml component.
	All objects will be passed with 3 major Q_Properties

QObject Properties:
	topic: The topic you want to publish the value to. Passed as a string
	value: The value the topic will be publishing. Needs to match value type
	republish: The rate the value will republish at in milliseconds. Setting to 0 will disable republish

Value Types:
	StringPub: string -> std_msg/String
	BoolPub:   bool   -> std_msg/Bool
	FloatPub:  float  -> std_msg/Float
	Int16Pub:  int16  -> std_msg/Int16

Description:
	First the object is needs to be passed to QML using qmlregistertype. The object will then be exposed
	to qml. When using in qml you set the topic as a a string "my/topic" it will then advertise that 
	topic to the ROS network.
	The value can then be written to in qml, by linking it to a control. Whenever the "value" changes,
	in qml, "setValue" will trigger, changing the value and publishing using function pub().
	Pub() will then publish the value to ROS
	If republish is set, then there will be a singleshot timer will trigger at every republish interval
********************************************************************************************************/

#include <QtQml/qqml.h>
#include <QTimer>
#include <ros/ros.h>

#include <std_msgs/String.h>
class StringPub : public QObject {
	Q_OBJECT
	Q_PROPERTY (QString topic MEMBER topic WRITE setTopic NOTIFY topicChanged);
	Q_PROPERTY (QString value MEMBER value WRITE setValue);
	Q_PROPERTY (int republish MEMBER republish WRITE setRepublish);
//	QML_ELEMENT

	QString topic;
	QString value;
	QTimer timer;
	int republish;
	ros::Publisher p;

	public:
	StringPub (): topic (""), value (""), republish (0) {};
	void setTopic (const QString& t) {
		topic = t;
		ros::NodeHandle h;
		p = h.advertise <std_msgs::String> (topic.toStdString (), 1, true);
		emit topicChanged (topic);
	}
	void setValue (const QString& v) { value = v; pub (); }
	void setRepublish (int v) {
		republish = v;
		if(republish>0){timer.singleShot(republish,this, &StringPub::pub);}
	}
	void pub () {
		if (topic.isEmpty ()) return;
		std_msgs::String s;
		s.data = value.toStdString ();
		p.publish (s);
		if(republish>0){timer.singleShot(republish,this, &StringPub::pub);}
	}
	signals:
	void topicChanged (const QString &value);
};

#include <std_msgs/Bool.h>
class BoolPub : public QObject {
	Q_OBJECT
	Q_PROPERTY (QString topic MEMBER topic WRITE setTopic NOTIFY topicChanged);
	Q_PROPERTY (bool value MEMBER value WRITE setValue);
	Q_PROPERTY (int republish MEMBER republish WRITE setRepublish);
//	QML_ELEMENT

	QString topic;
	bool value;
	QTimer timer;
	int republish;
	ros::Publisher p;

	public:
	BoolPub (): topic (""), value (false), republish (0) {};
	void setTopic (const QString& t) {
		topic = t;
		ros::NodeHandle h;
		p = h.advertise <std_msgs::Bool> (topic.toStdString (), 1, true);
		emit topicChanged (topic);
	}
	void setValue (const bool v) { value = v; pub (); }
	void setRepublish (int v) {
		republish = v;
		if(republish>0){timer.singleShot(republish,this, &BoolPub::pub);}
	}
	void pub () {
		if (topic.isEmpty ()) return;
		std_msgs::Bool x;
		x.data = value;
		p.publish (x);
		if(republish>0){timer.singleShot(republish,this, &BoolPub::pub);}
	}
	signals:
	void topicChanged (const QString &value);
};

#include <std_msgs/Float32.h>
class FloatPub : public QObject {
	Q_OBJECT
	Q_PROPERTY (QString topic MEMBER topic WRITE setTopic NOTIFY topicChanged);
	Q_PROPERTY (float value MEMBER value WRITE setValue);
	Q_PROPERTY (int republish MEMBER republish WRITE setRepublish);
//	QML_ELEMENT

	QString topic;
	float value;
	QTimer timer;
	int republish;
	ros::Publisher p;

	public:
	FloatPub (): topic (""), value (NAN), republish (0) {};
	void setTopic (const QString& t) {
		topic = t;
		ros::NodeHandle h;
		p = h.advertise <std_msgs::Float32> (topic.toStdString (), 1, true);
		emit topicChanged (topic);
	}
	void setValue (const float v) { value = v; pub (); }
	void setRepublish (int v) {
		republish = v;
		if(republish>0){timer.singleShot(republish,this,&FloatPub::pub);}
	}
	void pub () {
		if (topic.isEmpty ()) return;
		std_msgs::Float32 x;
		x.data = value;
		p.publish (x);
		if(republish>0){timer.singleShot(republish,this,&FloatPub::pub);}
	}
	signals:
	void topicChanged (const QString &value);
};

#include <std_msgs/Int16.h>
class Int16Pub : public QObject {
	Q_OBJECT
	Q_PROPERTY (QString topic MEMBER topic WRITE setTopic NOTIFY topicChanged);
	Q_PROPERTY (int value MEMBER value WRITE setValue);
	Q_PROPERTY (int republish MEMBER republish WRITE setRepublish);
//	QML_ELEMENT

	QString topic;
	int16_t value;
	QTimer timer;
	int republish;
	ros::Publisher p;

	public:
	Int16Pub (): topic (""), value (0), republish (0) {};
	void setTopic (const QString& t) {
		topic = t;
		ros::NodeHandle h;
		p = h.advertise <std_msgs::Int16> (topic.toStdString (), 1, true);
		emit topicChanged (topic);
	};
	void setValue (const int v) { value = v; pub (); }
	void setRepublish (int v) {
		republish = v;
		if(republish>0){timer.singleShot(republish,this, &Int16Pub::pub);}
	}
	void pub () {
		if (topic.isEmpty ()) return;
		std_msgs::Int16 x;
		x.data = value;
		p.publish (x);
		if(republish>0){timer.singleShot(republish,this, &Int16Pub::pub);}
	}
	signals:
	void topicChanged (const QString &value);
};
