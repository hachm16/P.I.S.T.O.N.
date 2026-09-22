#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>

#include "PistonBackend.h"

int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);

    PistonBackend pistonBackend;

    QQmlApplicationEngine engine;

    engine.rootContext()->setContextProperty(
        "pistonBackend",
        &pistonBackend);

    QObject::connect(
        &engine,
        &QQmlApplicationEngine::objectCreationFailed,
        &app,
        []()
        {
            QCoreApplication::exit(-1);
        },
        Qt::QueuedConnection);

    engine.loadFromModule(
        "PISTON_GUI_V1",
        "Main");

    return app.exec();
}
