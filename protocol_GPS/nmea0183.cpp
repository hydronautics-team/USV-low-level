#include "nmea0183.h"

/// Baudrate: 115200 bit/s
/// GPGGA - коордиинаты
/// PSAT - углы ориентации

namespace NMEA {

NMEA0183::NMEA0183(QString portName, int baudRate, QObject *parent)
{
    gps_port.setBaudRate(baudRate);
    gps_port.setPortName(portName);
    gps_port.open(QIODevice::ReadOnly);
    if (gps_port.isOpen()){
        qDebug()<<" port was opened";
    }
    else {
        qDebug()<<" error open port "<< gps_port.errorString();
    }

    {
        QString fileName = QString("log-")+QSysInfo::machineHostName()+QString("-")+QDate::currentDate().toString("yy-MM-dd")+QString("-") \
                           +QTime::currentTime().toString("hh-mm-ss")+".txt";
        qDebug()<<fileName;
        fileGPS.setFileName(fileName);

        if (fileGPS.open(QIODevice::ReadWrite | QIODevice::Text))
        {
            qDebug()<<"fileGPS is opened";
        }
        else
        {
            qDebug()<< fileGPS.errorString() << " " << fileGPS.error();
        }

    connect (&gps_port, &QSerialPort::readyRead, this, &NMEA0183::readData);
}
}

void NMEA0183::readData()
{
    QTextStream stream (&fileGPS);
    gps_buffer.append(gps_port.readAll());
    stream << gps_buffer;
    // gps_buffer.clear();

    parseBuffer();
    emit newMessageDetected(gps);
}

TitleNMEA stringToTitle(const QByteArray tit)
{
    if (tit == "GPRMC") return GPRMC;
    if (tit == "GPVTG") return GPVTG;
    if (tit == "GPTXT") return GPTXT;
    if (tit == "GPGGA") return GPGGA;
    if (tit == "GPGSA") return GPGSA;
    if (tit == "GPGSV") return GPGSV;
    if (tit == "GPGLL") return GPGLL;
    if (tit == "GNRMC") return GNRMC;
    if (tit == "GNVTG") return GNVTG;
    if (tit == "GNGSA") return GNGSA;
    if (tit == "GNGGA") return GNGGA;
    if (tit == "GPGGA") return GNGGA;
    if (tit == "GPZDA") return GPZDA;
    if (tit == "GNZDA") return GNZDA;
    if (tit == "GLZDA") return GLZDA;
    if (tit == "HDT") return HDT;
    if (tit == "ROT") return ROT;
    if (tit == "PSAT") return PSAT;
    if (tit == "PRDCU") return PRDCU;
    if (tit == "GLGSV") return GLGSV;
    if (tit == "GLGSA") return GLGSA;
    if (tit == "GLGGA") return GLGGA;
    if (tit == "GLRMC") return GLRMC;
    if (tit == "GLVTG") return GLVTG;
    if (tit == "GNGLL") return GNGLL;

    return UNKNOWN;
}



void NMEA0183::parseBuffer()
{
    if (gps_buffer.size () < 6)
        return;//определяем что буфер не пустой
    int index =gps_buffer.indexOf("$"); //поиск индекса $
    if ((index>0))
        gps_buffer.remove(0, index-1);
    index =gps_buffer.indexOf("$"); //поиск индекса $
//    qDebug () << "index$:   " << index;
    if (index == -1)
    {
        // Не найдено сообщение
        if(test_message) qDebug() << "Нет сообщения в буфере";
        return;
    }
    int count = gps_buffer.count(36);
    if(test_message) qDebug() << "count $"<< count;
    while (count !=0)
    {
        count = count-1;
        int index =gps_buffer.indexOf("$"); //поиск индекса $
        uint crc_in =gps_buffer.indexOf("*"); //поиск * после которой идет контрольная сумма
        uint end = crc_in + 5; //последний символ посылки
        if (end >= gps_buffer.size()) return;
        if ((index == -1) or (crc_in == -1))
        {
            // Не найдено сообщение
            if(test_message) qDebug() << "Нет сообщения в буфере";
            return;
        }
        else
        {
            QByteArray title = gps_buffer.mid (index+1, 5);
            if(test_message) qDebug() << "findTitle";
            findTitleNMEA(index, crc_in, end, title);
        }
        gps_buffer.remove(0, end);

        if(test_message) qDebug() << "gps_buffer.size:   " << gps_buffer.size();
    }
}

void NMEA0183::findTitleNMEA(qint8 index, qint8 crc_in, uint end, QByteArray title)
{
    TitleNMEA titleEnum = stringToTitle(title);
    if (titleEnum == 27)
    {
        QByteArray title4 = title.mid(0,4);
        if (title4 == "PSAT") titleEnum = PSAT;
    }
    QByteArray msg;
    int crc_real = 0;

    if (crc_in < index)
    {
        gps_buffer.remove(0, index - 1);
        index = gps_buffer.indexOf("$");
        crc_in = gps_buffer.indexOf("*");
    }
    // Обрезаем сообщение для расчета CRC, убираем символы после '*'
    msg = gps_buffer.mid(index + 1, crc_in - index - 1).trimmed(); // Убираем '\r\n'
    crc_real = crc_real_method(gps_buffer, crc_in);

    if (crc_real == crc(msg))  // Сравниваем реальные и рассчитанные CRC
    {
        switch (titleEnum) {
        case GPRMC:
            parseGPRMC(msg);
            break;
        case GPVTG:
            parseGPVTG(msg);
            break;
        case GPTXT:
            parseGPTXT(msg);
            break;
        case GPGGA:
            parseGPGGA(msg);
            break;
        case GPGSA:
            parseGPGSA(msg);
            break;
        case GPGSV:
            parseGPGSV(msg);
            break;
        case GPGLL:
            parseGPGLL(msg);
        case GNGLL:
            parseGPGLL(msg);
            break;
        case GNRMC:
            parseGNRMC(msg);
            break;
        case GNVTG:
            parseGNVTG(msg);
            break;
        case GNGSA:
            parseGNGSA(msg);
            break;
        case GNGGA:
            parseGPGGA(msg);
            break;
        case GPZDA:
            parseGPZDA(msg);
            break;
        case GNZDA:
            parseGNZDA(msg);
            break;
        case GLZDA:
            parseGLZDA(msg);
            break;
        case HDT:
            parseHDT(msg);
            break;
        case ROT:
            parseROT(msg);
            break;
        case PSAT:
            parsePSAT(msg);
            break;
        case PRDCU:
            parsePRDCU(msg);
            break;
        case GLGSV:
            parseGLGSV(msg);
            break;
        case GLGSA:
            parseGLGSA(msg);
            break;
        case GLGGA:
            parseGLGGA(msg);
            break;
        case GLRMC:
            parseGLRMC(msg);
            break;
        case GLVTG:
            parseGLVTG(msg);
            break;
        case UNKNOWN:
            if(test_message) qDebug() << "Unknown or unsupported NMEA title:" << title;
            break;
        }
    }
    else
    {
        if(test_message) qDebug() << "error crc";
    }

    gps_buffer.remove(0, end);
    if(test_message) qDebug() << title << "parsed.";
}


int NMEA0183::crc(QByteArray tmp)
{
    int crc_ = 0;
    for (int i = 0; i <= tmp.size(); i++)  // Исключаем $ и *<checksum>
    {
        if (i<tmp.size())
        crc_ ^= tmp[i];
    }
    return crc_;
}


int NMEA0183::crc_real_method(QByteArray gps_buffer, uint crc_in)
{
    int crc_real = 0;

    // Получение первого символа контрольной суммы
    char first_char = gps_buffer[crc_in + 1];
    if (first_char >= '0' && first_char <= '9') {
        crc_real += (first_char - '0') * 16;  // Цифры '0'-'9'
    } else if (first_char >= 'A' && first_char <= 'F') {
        crc_real += (first_char - 'A' + 10) * 16;  // Символы 'A'-'F'
    }

    // Получение второго символа контрольной суммы
    char second_char = gps_buffer[crc_in + 2];
    if (second_char >= '0' && second_char <= '9') {
        crc_real += (second_char - '0');  // Цифры '0'-'9'
    } else if (second_char >= 'A' && second_char <= 'F') {
        crc_real += (second_char - 'A' + 10);  // Символы 'A'-'F'
    }

    return crc_real;
}


void NMEA0183::parseGPRMC(QByteArray msg)
{
    int index =msg.indexOf(44);//ищем первую запятую
    msg.remove(0, index+1);
    index =msg.indexOf(44);//ищем запятую
    if (index == 0)
    {
        msg.remove(0, index+1);
    }
    if (index > 0)
    {
        gps.rms.time = QTime::fromString((msg.mid(0, index-1)), "hhmmss.z");
        msg.remove(0, index+1);
    }
    index =msg.indexOf(44);//ищем запятую
    if (index == 0)
    {
        gps.rms.status = 0;
        msg.remove(0, index+1);
    }
    if (index > 0)
    {
        gps.rms.status = (char)msg[0];
        msg.remove(0, index+1);
    }
    index =msg.indexOf(44);//ищем запятую
    if (index == 0)
    {
        gps.rms.lat = 0;
        msg.remove(0, index+1);
    }
    if (index > 0)
    {
        gps.rms.lat = atof(msg.mid(0, index-1));
        msg.remove(0, index+1);
    }
    index =msg.indexOf(44);//ищем запятую
    if (index == 0)
    {
        gps.rms.NS = 0;
        msg.remove(0, index+1);
    }
    if (index > 0)
    {
        gps.rms.NS = (char)msg[0];
        msg.remove(0, index+1);
    }
    index =msg.indexOf(44);//ищем запятую
    if (index == 0)
    {
        gps.rms.long_ = 0;
        msg.remove(0, index+1);
    }
    if (index > 0)
    {
        gps.rms.long_ = atof(msg.mid(0, index-1));
        msg.remove(0, index+1);
    }
    index =msg.indexOf(44);//ищем запятую
    if (index == 0)
    {
        gps.rms.EW = 0;
        msg.remove(0, index+1);
    }
    if (index > 0)
    {
        gps.rms.EW = (char)msg[0];
        msg.remove(0, index+1);
    }
    index =msg.indexOf(44);//ищем запятую
    if (index == 0)
    {
        gps.rms.spd = 0;
        msg.remove(0, index+1);
    }
    if (index > 0)
    {
        gps.rms.spd = atof(msg.mid(0, index-1));
        msg.remove(0, index+1);
    }
    index =msg.indexOf(44);//ищем запятую
    if (index == 0)
    {
        gps.rms.cog = 0;
        msg.remove(0, index+1);
    }
    if (index > 0)
    {
        gps.rms.cog = atof(msg.mid(0, index-1));
        msg.remove(0, index+1);
    }
    index =msg.indexOf(44);//ищем запятую
//    qDebug() << msg;
    if (index == 0)
    {
        msg.remove(0, index+1);
    }
    if (index > 0)
    {
//        qDebug() << msg.mid(0, index);
        gps.rms.date = QDate::fromString((msg.mid(0, index)), "ddMMyy");
        msg.remove(0, index+1);
//        qDebug() << gps.rms.date.toString();
    }
    index =msg.indexOf(44);//ищем запятую
    if (index == 0)
    {
        gps.rms.mv = 0;
        msg.remove(0, index+1);
    }
    if (index > 0)
    {
        gps.rms.mv = atof(msg.mid(0, index-1));
        msg.remove(0, index+1);
    }
    index =msg.indexOf(44);//ищем запятую
    if (index == 0)
    {
        gps.rms.mvEW = 0;
        msg.remove(0, index+1);
    }
    if (index > 0)
    {
        gps.rms.mvEW = (char)msg[0];
        msg.remove(0, index+1);
    }
    index =msg.indexOf(44);//ищем запятую
    if (index == 0)
    {
        gps.rms.posMode = 0;
        msg.remove(0, index+1);
    }
    if (index > 0)
    {
        gps.rms.posMode = (char)msg[0];
        msg.remove(0, index+1);
    }
    gps.rms.counter +=1;
    if(test_message)qDebug() << "gps.rms.time:    " << gps.rms.time.toString("hhmmss.z");
    if(test_message)qDebug() << "gps.rms.status:  " << gps.rms.status;
    if(test_message)qDebug() << "gps.rms.lat:     " << gps.rms.lat;
    if(test_message)qDebug() << "gps.rms.NS:      " << gps.rms.NS;
    if(test_message)qDebug() << "gps.rms.long:    " << gps.rms.long_;
    if(test_message)qDebug() << "gps.rms.EW:      " << gps.rms.EW;
    if(test_message)qDebug() << "gps.rms.spd:     " << gps.rms.spd;
    if(test_message)qDebug() << "gps.rms.cog:     " << gps.rms.cog;
    if(test_message)qDebug() << "gps.rms.date:    " << gps.rms.date.toString("dd.MM.yy");
    if(test_message)qDebug() << "gps.rms.mv:      " << gps.rms.mv;
    if(test_message)qDebug() << "gps.rms.mvEW:    " << gps.rms.mvEW;
    if(test_message)qDebug() << "gps.rms.posMode: " << gps.rms.posMode;
    if(test_message)qDebug() << "gps.rms.counter: " << gps.rms.counter;

}

void NMEA0183::parseGLRMC(QByteArray msg)
{

}

void NMEA0183::parseGNVTG(QByteArray msg)
{

}

void NMEA0183::parseGPVTG(QByteArray msg)
{
    int index = msg.indexOf(',');
    gps.vtg.trackMadeGood = atof(msg.mid(0, index));
    msg.remove(0, index + 1);

    index = msg.indexOf(',');
    gps.vtg.trackMadeGoodReference = msg.mid(0, 1);
    msg.remove(0, index + 1);

    index = msg.indexOf(',');
    gps.vtg.speedKnots = atof(msg.mid(0, index));
    msg.remove(0, index + 1);

    index = msg.indexOf(',');
    gps.vtg.speedKnotsUnit = msg.mid(0, 1);
    msg.remove(0, index + 1);

    index = msg.indexOf(',');
    gps.vtg.speedKph = atof(msg.mid(0, index));
    msg.remove(0, index + 1);

    index = msg.indexOf(',');
    gps.vtg.speedKphUnit = msg.mid(0, 1);
    msg.remove(0, index + 1);

    index = msg.indexOf(',');
    gps.vtg.posMode = msg.mid(0, 1);

    if(test_message) qDebug() << "GPVTG Parsed";
}

void NMEA0183::parseGLVTG(QByteArray msg)
{

}

void NMEA0183::parseGPTXT(QByteArray msg)
{
    // GPTXT сообщение может содержать текстовую информацию
    gps.txt.id = atoi(msg.mid(0, 2));
    msg.remove(0, 3);  // Удаляем ID и запятую

    int index = msg.indexOf(',');
    gps.txt.text = msg.mid(0, index);
    if(test_message)qDebug() << "GPTXT Parsed";
}


void NMEA0183::parseGPGGA(QByteArray msg)
{
    // Пример сообщения: $GPGGA,123519.00,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
    if(test_message)qDebug() << msg;
    int index = msg.indexOf(',');
    msg.remove(0, index + 1);

    // Время UTC
    index = msg.indexOf(',');
    gps.gga.time = QTime::fromString(msg.mid(0, index), "hhmmss.z");
    msg.remove(0, index + 1);
    if(test_message)qDebug() << "gps.gga.time: " << gps.gga.time;

    // Широта
    index = msg.indexOf(',');
    gps.gga.latitude = msg.mid(0, index).toDouble();
    msg.remove(0, index + 1);
    if(test_message) qDebug() << "gps.gga.latitude: " << gps.gga.latitude;

    // Полушарие широты (N/S)
    index = msg.indexOf(',');
    gps.gga.latHemisphere = msg.mid(0, index);
    msg.remove(0, index + 1);
    if(test_message) qDebug() << "gps.gga.latHemisphere: " << gps.gga.latHemisphere;

    // Долгота
    index = msg.indexOf(',');
    gps.gga.longitude = msg.mid(0, index).toDouble();
    msg.remove(0, index + 1);
    if(test_message)qDebug() << "gps.gga.longitude: " << gps.gga.longitude;

    // Полушарие долготы (E/W)
    index = msg.indexOf(',');
    gps.gga.lonHemisphere = msg.mid(0, index);
    msg.remove(0, index + 1);
    if(test_message)qDebug() << "gps.gga.lonHemisphere: " << gps.gga.lonHemisphere;

    // Индикатор качества
    index = msg.indexOf(',');
    gps.gga.quality = msg.mid(0, index).toInt();
    msg.remove(0, index + 1);
    if(test_message)qDebug() << "gps.gga.quality: " << gps.gga.quality;

    // Количество спутников
    index = msg.indexOf(',');
    gps.gga.satellitesUsed = msg.mid(0, index).toInt();
    msg.remove(0, index + 1);
    if(test_message)qDebug() << "gps.gga.satellitesUsed: " << gps.gga.satellitesUsed;

    // Величина горизонтального геометрического фактора (HDOP)
    index = msg.indexOf(',');
    gps.gga.hdop = msg.mid(0, index).toDouble();
    msg.remove(0, index + 1);
    if(test_message)qDebug() << "gps.gga.hdop: " << gps.gga.hdop;

    // Высота антенны над уровнем моря
    index = msg.indexOf(',');
    gps.gga.altitude = msg.mid(0, index).toDouble();
    msg.remove(0, index + 1);
    if(test_message)qDebug() << "gps.gga.altitude: " << gps.gga.altitude;

    // Единица измерения высоты (метры)
    index = msg.indexOf(',');
    gps.gga.altitudeUnit = msg.mid(0, index);
    msg.remove(0, index + 1);
    if(test_message)qDebug() << "gps.gga.altitudeUnit: " << gps.gga.altitudeUnit;

    // Превышение геоида над эллипсоидом WGS84
    index = msg.indexOf(',');
    gps.gga.geoidHeight = msg.mid(0, index).toDouble();
    msg.remove(0, index + 1);
    if(test_message)qDebug() << "gps.gga.geoidHeight: " << gps.gga.geoidHeight;

    // Единица измерения геоида (метры)
    index = msg.indexOf(',');
    gps.gga.geoidUnit = msg.mid(0, index);
    msg.remove(0, index + 1);
    if(test_message)qDebug() << "gps.gga.geoidUnit: " << gps.gga.geoidUnit;

    // Возраст дифференциальной поправки (если есть)
    index = msg.indexOf(',');
    if (index > 0)
    {
        gps.gga.dgpsAge = msg.mid(0, index).toDouble();
        msg.remove(0, index + 1);
    }
    if(test_message)qDebug() << "gps.gga.dgpsAge: " << gps.gga.dgpsAge;

    // Идентификатор дифференциальной станции (если есть)
    index = msg.indexOf('*');
    if (index > 0)
    {
        gps.gga.dgpsStationId = msg.mid(0, index).toInt();
        msg.remove(0, index + 1);
    }
    if(test_message)qDebug() << "gps.gga.dgpsStationId: " << gps.gga.dgpsStationId;

    if(test_message)qDebug() << "GPGGA Parsed";
}


void NMEA0183::parseGLGGA(QByteArray msg)
{

}

void NMEA0183::parseGNRMC(QByteArray msg)
{

}

void NMEA0183::parseGPGSA(QByteArray msg)
{
    // Парсинг GSA сообщений
    int index = msg.indexOf(',');
    gps.gsa.mode = msg.mid(0, 1);
    msg.remove(0, index + 1);

    index = msg.indexOf(',');
    gps.gsa.fixType = atoi(msg.mid(0, index));
    msg.remove(0, index + 1);

    for (int i = 0; i < 12; i++) {
        index = msg.indexOf(',');
        gps.gsa.satellites[i] = atoi(msg.mid(0, index));
        msg.remove(0, index + 1);
    }

    index = msg.indexOf(',');
    gps.gsa.pdop = atof(msg.mid(0, index));
    msg.remove(0, index + 1);

    index = msg.indexOf(',');
    gps.gsa.hdop = atof(msg.mid(0, index));
    msg.remove(0, index + 1);

    index = msg.indexOf(',');
    gps.gsa.vdop = atof(msg.mid(0, index));

    if(test_message)qDebug() << "GPGSA Parsed";
}

void NMEA0183::parseGLGSA(QByteArray msg)
{

}

void NMEA0183::parsePGRMT(QByteArray msg)
{

}

void NMEA0183::parseGPZDA(QByteArray msg)
{
    int index = msg.indexOf(',');
    gps.zda.time = QTime::fromString(msg.mid(0, index), "hhmmss.z");
    msg.remove(0, index + 1);
    if(test_message)qDebug() << "gps.zda.time: " << gps.zda.time;
    index = msg.indexOf(',');
    gps.zda.day = atoi(msg.mid(0, index));
    msg.remove(0, index + 1);
    if(test_message)qDebug() << "gps.zda.day: " << gps.zda.day;
    index = msg.indexOf(',');
    gps.zda.month = atoi(msg.mid(0, index));
    msg.remove(0, index + 1);
    if(test_message)qDebug() << "gps.zda.month: " << gps.zda.month;
    index = msg.indexOf(',');
    gps.zda.year = atoi(msg.mid(0, index));
    msg.remove(0, index + 1);
    if(test_message)qDebug() << "gps.zda.year: " << gps.zda.year;
    index = msg.indexOf(',');
    gps.zda.localZoneHours = atoi(msg.mid(0, index));
    msg.remove(0, index + 1);
    if(test_message)qDebug() << "gps.zda.localZoneHours: " << gps.zda.localZoneHours;
    index = msg.indexOf(',');
    gps.zda.localZoneMinutes = atoi(msg.mid(0, index));
    if(test_message)qDebug() << "gps.zda.localZoneMinutes: " << gps.zda.localZoneMinutes;
    if(test_message)qDebug() << "GPZDA Parsed";
}

void NMEA0183::parseGNZDA(QByteArray msg)
{

}

void NMEA0183::parseGLZDA(QByteArray msg)
{

}

void NMEA0183::parseHDT(QByteArray msg)
{
    int index = msg.indexOf(',');
    gps.hdt.heading = atof(msg.mid(0, index));
    msg.remove(0, index + 1);

    gps.hdt.trueIndicator = msg.mid(0, 1);
    if(test_message)qDebug() << "HDT Parsed";
}

void NMEA0183::parseROT(QByteArray msg)
{
    int index = msg.indexOf(',');
    gps.rot.rateOfTurn = atof(msg.mid(0, index));
    msg.remove(0, index + 1);

    gps.rot.status = msg.mid(0, 1);
    if(test_message)qDebug() << "ROT Parsed";
}

void NMEA0183::parsePSAT(QByteArray msg)
{
    // Пример сообщения: $PSAT,HPR,123519.00,182.7,0.5,-1.3,N*6D
    if(test_message)qDebug() << msg;
    int index = msg.indexOf(',');
    msg.remove(0, index + 1);
    if(test_message)qDebug() << msg;
    index = msg.indexOf(',');
    msg.remove(0, index + 1);
    if(test_message)qDebug() << msg;
    index = msg.indexOf(',');
    gps.psat.time = QTime::fromString(msg.mid(0, index), "hhmmss.z");
    msg.remove(0, index + 1);
   if(test_message) qDebug() << "gps.psat.time: " << gps.psat.time;

    index = msg.indexOf(',');
    gps.psat.yaw = msg.mid(0, index).toDouble();
    msg.remove(0, index + 1);
    if(test_message)qDebug() << "gps.psat.yaw: " << gps.psat.yaw;

    index = msg.indexOf(',');
    gps.psat.pitch = msg.mid(0, index).toDouble();
    msg.remove(0, index + 1);
    if(test_message)qDebug() << "gps.psat.pitch: " << gps.psat.pitch;

    index = msg.indexOf(',');
    gps.psat.roll = msg.mid(0, index).toDouble();
    msg.remove(0, index + 1);
    if(test_message)qDebug() << "gps.psat.roll: " << gps.psat.roll;

    index = msg.indexOf('*');
    gps.psat.dataType = msg.mid(0, index);
    msg.remove(0, index + 1);
    if(test_message)qDebug() << "gps.psat.dataType: " << gps.psat.dataType;

    if(test_message) qDebug() << "PSAT Parsed";
}



void NMEA0183::parsePRDCU(QByteArray msg)
{

}

void NMEA0183::parseGPGSV(QByteArray msg)
{

}

void NMEA0183::parseGLGSV(QByteArray msg)
{

}

void NMEA0183::parseGNGSA(QByteArray msg)
{

}

void NMEA0183::parseGPGLL(QByteArray msg)
{
    int index =msg.indexOf(44);//ищем первую запятую
//    qDebug() << "msg: " << msg;
//    qDebug() << "index первая запятая: " << index;
    msg.remove(0, index+1);
    index =msg.indexOf(44);//ищем запятую
//    qDebug() << "index 2 запятая: " << index;
//    qDebug() << "msg: " << msg;
        if (index == 0)
        {
            gps.gll.lat = 0;
            msg.remove(0, index+1);
        }
        if (index > 0)
        {
            gps.gll.lat = atof(msg.mid(0, index-1));
            msg.remove(0, index+1);
        }
//        qDebug() << "gll.lat: " << gll.lat;

    index =msg.indexOf(44);//ищем запятую
//    qDebug() << "index 3 запятая: " << index;
//    qDebug() << "msg: " << msg;
        if (index == 0)
        {
            gps.gll.NS = 0;
            msg.remove(0, index+1);
        }
        if (index > 0)
        {
            gps.gll.NS = (char)msg[0];
            msg.remove(0, index+1);
        }
//        qDebug() << "gll.NS: " << gll.NS;

        index =msg.indexOf(44);//ищем запятую
//        qDebug() << "index 4 запятая: " << index;
//        qDebug() << "msg: " << msg;
            if (index == 0)
            {
                gps.gll.long_ = 0;
                msg.remove(0, index+1);
            }
            if (index > 0)
            {
                gps.gll.long_ = atof(msg.mid(0, index-1));
                msg.remove(0, index+1);
            }
//            qDebug() << "gll.long_: " << gll.long_;

        index =msg.indexOf(44);//ищем запятую
//        qDebug() << "index 5 запятая: " << index;
//        qDebug() << "msg: " << msg;
            if (index == 0)
            {
                gps.gll.EW = 0;
                msg.remove(0, index+1);
            }
            if (index > 0)
            {
                gps.gll.EW = (char)msg[0];
                msg.remove(0, index+1);
            }
//            qDebug() << "gll.EW: " << gll.EW;

        index =msg.indexOf(44);//ищем запятую
//        qDebug() << "index 6 запятая: " << index;
//        qDebug() << "msg: " << msg;
            if (index == 0)
            {
          //      gps.gll.time = 0;
                msg.remove(0, index+1);
            }
            if (index > 0)
            {
               // gps.gll.time = atof(msg.mid(0, index-1));
//                gps.gll.time_h = atoi(msg.mid(0, 2));
//                gps.gll.time_m = atoi(msg.mid(2, 2));
//                gps.gll.time_s = atoi(msg.mid(4, 2));
//                gps.gll.time_ms = atoi(msg.mid(7, 2));
                gps.gll.time = QTime::fromString((msg.mid(0, index-1)), "hhmmss.z");
                msg.remove(0, index+1);
            }
//            qDebug() << "gll.time: " << gll.time;

        index =msg.indexOf(44);//ищем запятую
//        qDebug() << "index 7 запятая: " << index;
//        qDebug() << "msg: " << msg;
            if (index == 0)
            {
                gps.gll.status = 0;
                msg.remove(0, index+1);
            }
            if (index > 0)
            {
                gps.gll.status = (char)msg[0];
                msg.remove(0, index+1);
            }
//            qDebug() << "gll.status: " << gll.status;
//        qDebug() << "index 8 запятая: " << index;
//        qDebug() << "msg: " << msg;
            if (index == 0)
            {
                gps.gll.posMode = 0;
                msg.remove(0, index+1);
            }
            if (index > 0)
            {
                gps.gll.posMode = (char)msg[0];
                msg.remove(0, index+1);
            }
//            qDebug() << "gll.posMode: " << gll.posMode;
        if(test_message)qDebug() << "gps.gll.lat:       " << gps.gll.lat;
        if(test_message)qDebug() << "gps.gll.NS:        " << gps.gll.NS;
        if(test_message)qDebug() << "gps.gll.long_:     " << gps.gll.long_;
        if(test_message)qDebug() << "gps.gll.EW:        " << gps.gll.EW;
//        qDebug() << "gps.gll.time:      " << gps.gll.time_h << ":" << gps.gll.time_m << ":" \
//                 << gps.gll.time_s << ":" << gps.gll.time_ms;
        if(test_message)qDebug() << "gps.gll.time:      " << gps.gll.time;
        if(test_message)qDebug() << "gps.gll.status:    " << gps.gll.status;
        if(test_message)qDebug() << "gps.gll.posMode:   " << gps.gll.posMode;
}

} //end namespace NMEA
