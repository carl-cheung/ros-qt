#include <QtQml/qqml.h>
#include <QTimer>
#include <ros/ros.h>

#define inhibit_period 2100

#include <std_msgs/String.h>
class StringSub : public QObject {
	Q_OBJECT
	Q_PROPERTY (QString topic READ topic WRITE setTopic);
	Q_PROPERTY (QString value READ value NOTIFY valueChanged);
	Q_PROPERTY (QString active READ active NOTIFY activeChanged);
//	QML_ELEMENT

	QString t, v;
	QTimer timer;
	bool _active;

	ros::Subscriber s;
	void callback (const std_msgs::String::ConstPtr &msg) {
		QString s = QString::fromStdString (msg->data);
		if (s != v) {
			v = s;
			emit valueChanged (v);
		}
		if (!_active) {
			_active = true;
			emit activeChanged (_active);
		}
		QMetaObject::invokeMethod (&timer, "start", Q_ARG (int, 2000));
	};

	public:
	QString topic () const { return t; };
	void setTopic (const QString &topic) {
		ros::NodeHandle h;
		t = topic;
		v = "";
		try {
			s = h.subscribe <std_msgs::String> (t.toStdString (), 1, &StringSub::callback, this);
		} catch (ros::ConflictingSubscriptionException& e) {
			std::cerr << "Failed to subscribe to " << topic.toStdString ()
			<< " " << e.what() << std::endl;
		}
		connect (&timer, &QTimer::timeout, this, &StringSub::expire);
		timer.start (2000);
		emit valueChanged (v);
	};
	QString value () const { return v; };
	bool active () const { return _active; };

	signals:
	void valueChanged (const QString &value);
	void activeChanged (bool active);
	public slots:
	void expire () { _active= false; emit activeChanged (_active); }
};


#include <std_msgs/Float32.h>
class FloatSub : public QObject {
	Q_OBJECT
	Q_PROPERTY (QString topic READ topic WRITE setTopic);
	Q_PROPERTY (float value READ value NOTIFY valueChanged);
	Q_PROPERTY (bool active READ active NOTIFY activeChanged);
//	QML_ELEMENT

	QString t;
	float v;
	QTimer timer, inhibit_timer;
	bool _active;

	ros::Subscriber s;
	void callback (const std_msgs::Float32::ConstPtr &msg) {
		const float f = msg->data;

		if (!inhibit_timer.isActive ()
			&& ((isfinite (v) && isfinite (f) && f != v) || isfinite (v) != isfinite (f)))
		{
			v = f;
			emit valueChanged (v);
		}
		if (!_active) {
			_active = true;
			emit activeChanged (_active);
		}
		QMetaObject::invokeMethod (&timer, "start", Q_ARG (int, 2000));
	};

	public:
	QString topic () const { return t; };
	void setTopic (const QString &topic) {
		ros::NodeHandle h;
		t = topic;
		v = NAN;
		inhibit_timer.setSingleShot (true);
		try {
			s = h.subscribe <std_msgs::Float32> (t.toStdString (), 1, &FloatSub::callback, this);
		} catch (ros::ConflictingSubscriptionException& e) {
			std::cerr << "Failed to subscribe to " << topic.toStdString ()
			<< " " << e.what() << std::endl;
		}
		connect (&timer, &QTimer::timeout, this, &FloatSub::expire);
		timer.start (2000);
		emit valueChanged (v);
	};
	float value () const { return v; };
	bool active () const { return _active; };

	signals:
	void valueChanged (float value);
	void activeChanged (bool active);
	public slots:
	void expire () { _active= false; emit activeChanged (_active); }
	void inhibit () { inhibit_timer.start (inhibit_period); };
};

#include <QVector>
#include <std_msgs/Float32MultiArray.h>
class FloatArraySub : public QObject {
	Q_OBJECT
	Q_PROPERTY (QString topic READ topic WRITE setTopic);
	Q_PROPERTY (QVector<qreal> value MEMBER v NOTIFY valueChanged);
	Q_PROPERTY (bool active READ active NOTIFY activeChanged);
//	QML_ELEMENT

	QString t;
	QVector<qreal> v;
	QTimer timer;
	bool _active;

	ros::Subscriber s;
	void callback (const std_msgs::Float32MultiArray::ConstPtr &msg) {
	  const size_t size = msg->layout.dim[0].size;
		v.resize (size);
		for (size_t i = 0; i < size; ++i)
			if (msg->data[i] != v[i]) {
				v[i] = msg->data[i];
				emit valueChanged (v);
			}
		if (!_active) {
			_active = true;
			emit activeChanged (_active);
		}
		QMetaObject::invokeMethod (&timer, "start", Q_ARG (int, 2000));
	};

	public:
	FloatArraySub (): t(), v(), timer(), _active (false) {}
	QString topic () const { return t; };
	void setTopic (const QString &topic) {
		ros::NodeHandle h;
		t = topic;
		v.fill (0.0);
		try {
			s = h.subscribe <std_msgs::Float32MultiArray> (t.toStdString (), 1, &FloatArraySub::callback, this);
		} catch (ros::ConflictingSubscriptionException& e) {
			std::cerr << "Failed to subscribe to " << topic.toStdString ()
			<< " " << e.what() << std::endl;
		}
		connect (&timer, &QTimer::timeout, this, &FloatArraySub::expire);
		timer.start (2000);
		emit valueChanged (v);
	};
	bool active () const { return _active; }

	signals:
	void valueChanged (QVector<qreal> value);
	void activeChanged (bool active);

	public slots:
	void expire () { _active= false; emit activeChanged (_active); }
};


#include <std_msgs/Int16.h>
class Int16Sub : public QObject {
	Q_OBJECT
	Q_PROPERTY (QString topic READ topic WRITE setTopic);
	Q_PROPERTY (int value READ value NOTIFY valueChanged);
	Q_PROPERTY (bool active READ active NOTIFY activeChanged);
//	QML_ELEMENT

	QString t;
	int v;
	QTimer timer, inhibit_timer;
	bool _active;

	ros::Subscriber s;
	void callback (const std_msgs::Int16::ConstPtr &msg) {
		const int i = msg->data;
		if (i != v && !inhibit_timer.isActive ()) {
			v = i;
			emit valueChanged (v);
		}
		if (!_active) {
			_active = true;
			emit activeChanged (_active);
		}
		QMetaObject::invokeMethod (&timer, "start", Q_ARG (int, 2000));
	};

	public:
	QString topic () const { return t; };
	void setTopic (const QString &topic) {
		ros::NodeHandle h;
		t = topic;
		v = 0;
		inhibit_timer.setSingleShot (true);
		try {
			s = h.subscribe <std_msgs::Int16> (t.toStdString (), 1, &Int16Sub::callback, this);
		} catch (ros::ConflictingSubscriptionException& e) {
			std::cerr << "Failed to subscribe to " << topic.toStdString ()
			<< " " << e.what() << std::endl;
		}
		connect (&timer, &QTimer::timeout, this, &Int16Sub::expire);
		timer.start (2000);
		emit valueChanged (v);
	};
	int value () const { return v; };
	bool active () const { return _active; };

	signals:
	void valueChanged (int value);
	void activeChanged (bool active);
	public slots:
	void expire () { _active= false; emit activeChanged (_active); }
	void inhibit () { inhibit_timer.start (inhibit_period); };
};

#include <std_msgs/Bool.h>
class BoolSub : public QObject {
	Q_OBJECT
	Q_PROPERTY (QString topic READ topic WRITE setTopic);
	Q_PROPERTY (bool value READ value NOTIFY valueChanged);
	Q_PROPERTY (bool active READ active NOTIFY activeChanged);
//	QML_ELEMENT

	QString t;
	bool v;
	QTimer timer;
	bool _active;

	ros::Subscriber s;
	void callback (const std_msgs::Bool::ConstPtr &msg) {
		const bool b = msg->data;
		if (b != v) {
			v = b;
			emit valueChanged (v);
		}
		if (!_active) {
			_active = true;
			emit activeChanged (_active);
		}
		QMetaObject::invokeMethod (&timer, "start", Q_ARG (int, 2000));
	};

	public:
	QString topic () const { return t; };
	void setTopic (const QString &topic) {
		ros::NodeHandle h;
		t = topic;
		v = false;
		try {
			s = h.subscribe <std_msgs::Bool> (t.toStdString (), 1, &BoolSub::callback, this);
		} catch (ros::ConflictingSubscriptionException& e) {
			std::cerr << "Failed to subscribe to " << topic.toStdString ()
			<< " " << e.what() << std::endl;
		}
		connect (&timer, &QTimer::timeout, this, &BoolSub::expire);
		timer.start (2000);
		emit valueChanged (v);
	};
	bool value () const { return v; };
	bool active () const { return _active; };

	signals:
	void valueChanged (bool value);
	void activeChanged (bool active);
	public slots:
	void expire () { _active= false; emit activeChanged (_active); }
};
