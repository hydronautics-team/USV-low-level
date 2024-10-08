#include "logger.h"

Logger::Logger(QObject *parent)
{
    QString fileName = QString("log-Echolot")+QSysInfo::machineHostName()+QString("-")+QDate::currentDate().toString("yy-MM-dd")+QString("-") \
            +QTime::currentTime().toString("hh-mm-ss")+".txt";
    qDebug()<<fileName;
    fileEcho.setFileName(fileName);

    if (fileEcho.open(QIODevice::ReadWrite | QIODevice::Text))
    {
        qDebug()<<"fileGPS is opened";
        QTextStream stream (&fileEcho);
        stream << "distanse, m"\
               <<"\n";
    }
    else
    {
        qDebug()<< fileEcho.errorString() << " " << fileEcho.error();
    }
}

void Logger::log_Echo(float d)
{
    if (fileEcho.isOpen())
    {
        QTextStream stream (&fileEcho);
        stream << d\
               <<"\n";
    }
}

