#include <QtWidgets/QApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QDebug>
#include "pubsub.h"
#include "publisher.hpp"
#include "subscriber.hpp"

int main(int argc, char *argv[])
{
	qputenv("QT_IM_MODULE", QByteArray("qtvirtualkeyboard"));

	ros::init (argc, argv, "rosuinode");

	QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);

	QApplication app(argc, argv);

	QQmlApplicationEngine engine;
	qmlRegisterType<StringPub>("Ros.Communication", 1, 0, "StringPub");
	qmlRegisterType<FloatPub>("Ros.Communication", 1, 0, "FloatPub");
	qmlRegisterType<Int16Pub>("Ros.Communication", 1, 0, "Int16Pub");
	qmlRegisterType<BoolPub>("Ros.Communication", 1, 0, "BoolPub");

	qmlRegisterType<StringSub>("Ros.Communication", 1, 0, "StringSub");
	qmlRegisterType<FloatSub>("Ros.Communication", 1, 0, "FloatSub");
	qmlRegisterType<FloatArraySub>("Ros.Communication", 1, 0, "FloatArraySub");
	qmlRegisterType<Int16Sub>("Ros.Communication", 1, 0, "Int16Sub");
	qmlRegisterType<BoolSub>("Ros.Communication", 1, 0, "BoolSub");

	qmlRegisterSingletonType( QUrl("qrc:qml/ros.qml"), "Ros.Communication", 1, 0, "ROS" );

	const QUrl url(QStringLiteral("qrc:qml/main.qml"));
	QObject::connect(&engine, &QQmlApplicationEngine::objectCreated,
									 &app, [url](QObject *obj, const QUrl &objUrl) {
		if (!obj && url == objUrl)
			QCoreApplication::exit(-1);
	}, Qt::QueuedConnection);
	engine.load(url);

	PubSub pubsub;
	pubsub.start();	

	return app.exec();;
}
