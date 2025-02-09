#ifndef LOGGER_H
#define LOGGER_H

#include <QObject>
#include <QFile>
#include <QTextStream>
#include <QTime>
#include <QDebug>
#include "protocol_echolot/PA500.h"
#include "protocol_GPS/nmea0183.h"
#include "protocol_gans_ZIMA/protocolzima.h"
#include "coordSSP/coordssp.h"

class Logger:  public QObject
{
    Q_OBJECT
public:
    Logger(QObject *parent = nullptr);
    QFile fileEcho;
protected:
    EcholotStr echoLog;
    NMEA::GPS gpsLog;
    ZimaData zimaLog;
    SSPdata sspLog;
    void logWrite();
public slots:
    void log_Echo(EcholotStr *echo);
    void log_GPS (NMEA::GPS *gps);
    void log_Zima(ZimaData *zima);
    void log_SSP(SSPdata *ssp);
};

#endif // LOGGER_H
