#include "logger.h"

Logger::Logger(QObject *parent)
{
    QString fileName = QString("log-")+QSysInfo::machineHostName()+QString("-")+QDate::currentDate().toString("yy-MM-dd")+QString("-") \
            +QTime::currentTime().toString("hh-mm-ss")+".txt";
    qDebug()<<fileName;
    fileEcho.setFileName(fileName);

    if (fileEcho.open(QIODevice::ReadWrite | QIODevice::Text))
    {
//        qDebug()<<"fileGPS is opened";
        QTextStream stream (&fileEcho);
        stream << "timeUSV; " \
               << "Echolot; Echolot; "\
               << "GPS.gga; GPS.gga; GPS.gga; GPS.gga; GPS.gga; "  \
               << "GPS.gga; GPS.gga; GPS.gga; GPS.gga; GPS.gga; "  \
               << "GPS.gga; GPS.gga; GPS.gga; GPS.gga; GPS.gga; "  \
               << "GPS.gll; GPS.gll; GPS.gll; GPS.gll; GPS.gll; GPS.gll; GPS.gll; GPS.gll; "  \
               << "GPS.psat; GPS.psat; GPS.psat; GPS.psat; GPS.psat; GPS.psat; " \
               << "zimaLog.pzmae; zimaLog.pzmae; zimaLog.pzmae; zimaLog.pzmae; zimaLog.pzmae; zimaLog.pzmae; zimaLog.pzmae; zimaLog.pzmae; zimaLog.pzmae; zimaLog.pzmae; " \
               << "zimaLog.pzmaf; zimaLog.pzmaf; zimaLog.pzmaf; zimaLog.pzmaf; zimaLog.pzmaf; " \
               << "zimaLog.pzmag; zimaLog.pzmag; zimaLog.pzmag; " \
               << "zimaLog.pzma0; zimaLog.pzma0; " \

               << "\n";
        stream << "timeUSV; " \
               << "counter; "<< "distanse, m;"\
               << "time, hh:mm:ss; latitude; latHemisphere; longitude; lonHemisphere; " \
               << "quality; satellitesUsed; hdop; altitude; altitudeUnit; " \
               << "geoidHeight; geoidUnit; dgpsAge; dgpsStationId; count; " \
               << "lat; NS; long_; EW; time; status; posMode; count; " \
               << "psat_time; psat_yaw; psat_pitch; psat_roll; psat_dataType; psat_count; " \
               << "pzmae_TargetID; pzmae_RequestID; pzmae_dFlag; pzmae_Azimuth; pzmae_Distance; pzmae_DataValue; pzmae_SNR; pzmae_DPL; pzmae_count_answer; pzmae_count_request; " \
               << "pzmafTemperature; pzmafDepth; pzmafisAHRSEnabled; pzmafTRX_State; pzmafcount; " \
               << "pzmagRoll; pzmagPitch; pzmagcount; " \
               << "pzma0Error_code; pzma0count_request; " \
               << "counter; latitudeReper; longitudeReper; X; Y; yawMagn; yawIner; yaw; pitch; roll; X_accel; Y_accel; Z_accel; X_rate; Y_rate; Z_rate; X_magn; Y_magn; Z_magn; quat[0]; quat[1]; quat[2]; quat[3]; VMA1; VMA2; VMA3; VMA4; flag_yaw; flag_pitch; flag_roll; flag_march; flag_depth; flag_lag; modeReal; yawSet; latitudePerpose; longitudePerpose; " \

               <<"\n";
    }
    else
    {
        qDebug()<< fileEcho.errorString() << " " << fileEcho.error();
    }
}

void Logger::logWrite()
{
//    qDebug() << "logWrite";
        if (fileEcho.isOpen())
        {
            QTextStream stream (&fileEcho);
            stream << QTime::currentTime().toString("hh-mm-ss") << "; " \
                   << echoLog.counter << "; " << QString::number(echoLog.depth, 'f', 9)  << "; "\

                   << gpsLog.gga.time.toString("hh:mm:ss") << "; " \
                   << QString::number(gpsLog.gga.latitude, 'f', 16) << "; " \
                   << gpsLog.gga.latHemisphere << "; " \
                   << QString::number(gpsLog.gga.longitude, 'f', 16) << "; " \
                   << gpsLog.gga.lonHemisphere << "; " \
                   << gpsLog.gga.quality << "; " \
                   << gpsLog.gga.satellitesUsed << "; " \
                   << gpsLog.gga.hdop << "; " \
                   << gpsLog.gga.altitude << "; " \
                   << gpsLog.gga.altitudeUnit << "; " \
                   << gpsLog.gga.geoidHeight << "; " \
                   << gpsLog.gga.geoidUnit << "; " \
                   << gpsLog.gga.dgpsAge << "; " \
                   << gpsLog.gga.dgpsStationId << "; " \
                   << gpsLog.gga.count << "; " \

                   << QString::number(gpsLog.gll.lat, 'f', 16)  << "; " \
                   << gpsLog.gll.NS << "; " \
                   << QString::number(gpsLog.gll.long_, 'f', 16)  << "; " \
                   << gpsLog.gll.EW << "; " \
                   << gpsLog.gll.time.toString("hh:mm:ss") << "; " \
                   << gpsLog.gll.status << "; " \
                   << gpsLog.gll.posMode << "; " \
                   << gpsLog.gll.count << "; " \

                   << gpsLog.psat.time.toString("hh:mm:ss")  << "; " \
                   << gpsLog.psat.yaw  << "; " \
                   << gpsLog.psat.pitch  << "; " \
                   << gpsLog.psat.roll  << "; " \
                   << gpsLog.psat.dataType  << "; " \
                   << gpsLog.psat.count  << "; " \

                   << zimaLog.pzmae.TargetID  << "; " \
                   << zimaLog.pzmae.RequestID  << "; " \
                   << zimaLog.pzmae.dFlag  << "; " \
                   << zimaLog.pzmae.Azimuth  << "; " \
                   << QString::number(zimaLog.pzmae.Distance, 'f', 9)   << "; " \
                   << zimaLog.pzmae.DataValue  << "; " \
                   << zimaLog.pzmae.SNR  << "; " \
                   << zimaLog.pzmae.DPL  << "; " \
                   << zimaLog.pzmae.count_answer  << "; " \
                   << zimaLog.count_request  << "; " \

                   << zimaLog.pzmaf.Temperature  << "; " \
                   << zimaLog.pzmaf.Depth  << "; " \
                   << zimaLog.pzmaf.isAHRSEnabled  << "; " \
                   << zimaLog.pzmaf.TRX_State  << "; " \
                   << zimaLog.pzmaf.count  << "; " \

                   << zimaLog.pzmag.Roll  << "; " \
                   << zimaLog.pzmag.Pitch  << "; " \
                   << zimaLog.pzmag.count  << "; " \

                   << zimaLog.pzma0.Error_code  << "; " \
                   << zimaLog.pzma0.count_request  << "; " \

                   << sspLog.counter << "; " \
                   << QString::number(sspLog.latitudeReper, 'f', 16)  << "; " \
                   << QString::number(sspLog.longitudeReper, 'f', 16)<< "; " \
                   << sspLog.X << "; " \
                   << sspLog.Y << "; " \
                   << QString::number(sspLog.yawMagn, 'f', 6) << "; " \
                   << QString::number(sspLog.yawIner, 'f', 6) << "; " \
                   << QString::number(sspLog.yaw, 'f', 6) << "; " \
                   << sspLog.pitch << "; " \
                   << sspLog.roll << "; " \
                   << sspLog.X_accel << "; " \
                   << sspLog.Y_accel << "; " \
                   << sspLog.Z_accel << "; " \
                   << sspLog.X_rate << "; " \
                   << sspLog.Y_rate << "; " \
                   << sspLog.Z_rate << "; " \
                   << sspLog.X_magn << "; " \
                   << sspLog.Y_magn << "; " \
                   << sspLog.Z_magn << "; " \
                   << sspLog.quat[0] << "; " \
                   << sspLog.quat[1]  << "; " \
                   << sspLog.quat[2]  << "; " \
                   << sspLog.quat[3]  << "; " \
                   << sspLog.VMA1 << "; " \
                   << sspLog.VMA2 << "; " \
                   << sspLog.VMA3 << "; " \
                   << sspLog.VMA4 << "; " \
                   << sspLog.flag_yaw << "; " \
                   << sspLog.flag_pitch << "; " \
                   << sspLog.flag_roll << "; " \
                   << sspLog.flag_march << "; " \
                   << sspLog.flag_depth << "; " \
                   << sspLog.flag_lag << "; " \
                   << sspLog.modeReal << "; " \
                   << sspLog.yawSet << "; " \
                   << QString::number(sspLog.latitudePerpose, 'f', 16) << "; " \
                   << QString::number(sspLog.longitudePerpose, 'f', 16) << "; " \

                   <<"\n";
            }
}

void Logger::log_Echo(EcholotStr *echo)
{

    echoLog = *echo;
    qDebug() << "echoLog.depth;" << echoLog.depth;
    qDebug() <<"echoLog.counter;" << echoLog.counter;
    logWrite();
}

void Logger::log_GPS(NMEA::GPS *gps)
{
//    qDebug() << "log_GPS(NMEA::GPS gps";
    gpsLog = *gps;
//    qDebug() << gpsLog.gga.time;

    logWrite();
}

void Logger::log_Zima(ZimaData *zima)
{
    qDebug() << "log_ZIMA";
    zimaLog = *zima;
    logWrite();
}

void Logger::log_SSP(SSPdata *ssp)
{
    sspLog = *ssp;
    logWrite();
}

