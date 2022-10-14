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
		connect (&timer, &QTimer::timeout, this, &StringPub::pub);
	}
	void pub () {
		if (topic.isEmpty ()) return;
		std_msgs::String s;
		s.data = value.toStdString ();
		p.publish (s);
		if (republish)
			timer.start (republish);
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
		connect (&timer, &QTimer::timeout, this, &BoolPub::pub);
	}
	void pub () {
		if (topic.isEmpty ()) return;
		std_msgs::Bool x;
		x.data = value;
		p.publish (x);
		if (republish)
			timer.start (republish);
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
		connect (&timer, &QTimer::timeout, this, &FloatPub::pub);
		republish = v;
	}
	void pub () {
		if (topic.isEmpty ()) return;
		std_msgs::Float32 x;
		x.data = value;
		p.publish (x);
		if (republish)
			timer.start (republish);
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
		connect (&timer, &QTimer::timeout, this, &Int16Pub::pub);
	}
	void pub () {
		if (topic.isEmpty ()) return;
		std_msgs::Int16 x;
		x.data = value;
		p.publish (x);
		if (republish)
			timer.start (republish);
	}
	signals:
	void topicChanged (const QString &value);
};
