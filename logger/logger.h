#ifndef LOGGER_H
#define LOGGER_H

#include <QObject>
#include <QFile>
#include <QTextStream>
#include <QTime>
#include <QDebug>
#include "protocol_echolot/PA500.h"

class Logger:  public QObject
{
    Q_OBJECT
public:
    Logger(QObject *parent = nullptr);
    QFile fileEcho;
public slots:
    void log_Echo(float d);
};

#endif // LOGGER_H
