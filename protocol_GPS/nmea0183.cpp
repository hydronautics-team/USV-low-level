#include "nmea0183.h"

/// Baudrate: 115200 bit/s
/// GPGGA - коордиинаты
/// PSAT - углы ориентации

namespace NMEA {

NMEA0183::NMEA0183(QString portName, int baudRate, QObject *parent)
{
    qDebug() << "I start";
    gps_port.setBaudRate(baudRate);
    gps_port.setPortName(portName);
    gps_port.open(QIODevice::ReadOnly);
    if (gps_port.isOpen()){
        qDebug()<<" port was opened";
    }
    else {
        qDebug()<<" error open port "<< gps_port.errorString();
    }
    qDebug() << "объявление";
    gps = new GPS;
    connect (&gps_port, &QSerialPort::readyRead, this, &NMEA0183::readData);

}

void NMEA0183::readData()
{
    gps_buffer.append(gps_port.readAll());
//    qDebug() << gps_buffer;
    // gps_buffer.clear();
    parseBuffer();
}

TitleNMEA stringToTitle(const QByteArray &tit)
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
    int index = gps_buffer.indexOf("$"); //поиск индекса $
    if ((index>0))
        gps_buffer.remove(0, index); //удаляем оставшийся мусор в посылке
    index = gps_buffer.indexOf("$"); //поиск индекса $
//    qDebug() << "parseBuffer index $" << index;
    if (index == -1)
    {
        // Не найдено сообщение
        if(test_message) qDebug() << "Нет сообщения в буфере";
        return;
    }
//    qDebug() << "parseBuffer start" << gps_buffer;
    int count = gps_buffer.count("$");
    if(test_message) qDebug() << "count $"<< count;
    while (count !=0)
    {
        count = count-1;
        qint8 index =gps_buffer.indexOf("$"); //поиск индекса $
        qint8 crc_in =gps_buffer.indexOf("*", index); //поиск * после которой идет контрольная сумма,index - после какого символа начинаем искать
        qint8 end = crc_in + 5; //последний символ посылки
//        qDebug() << "gps_buffer.mid(0,end):" <<gps_buffer.mid(0,end);
        if (end > gps_buffer.size()) return;
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

void NMEA0183::findTitleNMEA(qint8 &index, qint8 &crc_in, qint8 &end, QByteArray &title)
{
    TitleNMEA titleEnum = stringToTitle(title);
    if (titleEnum == 27)
    {
        QByteArray title4 = title.mid(0,4);
        if (title4 == "PSAT") titleEnum = PSAT;
    }
    QByteArray msg;
    int crc_real = 0;

//    if (crc_in < index)
//    {
//        gps_buffer.remove(0, index - 1);
//        index = gps_buffer.indexOf("$");
//        crc_in = gps_buffer.indexOf("*");
//    }
    // Обрезаем сообщение для расчета CRC, убираем символы после '*'
    msg = gps_buffer.mid(index + 1, crc_in - index - 1).trimmed(); // Убираем '\r\n'
    crc_real = crc_real_method(gps_buffer, crc_in);

    if (crc_real == crc(msg))  // Сравниваем реальные и рассчитанные CRC
    {
        switch (titleEnum) {
        case GPRMC:
//            parseGPRMC(msg);
            break;
        case GPVTG:
//            parseGPVTG(msg);
            break;
        case GPTXT:
//            parseGPTXT(msg);
            break;
        case GPGGA:
            parseGPGGA(msg);
            break;
        case GPGSA:
//            parseGPGSA(msg);
            break;
        case GPGSV:
//            parseGPGSV(msg);
            break;
        case GPGLL:
            parseGPGLL(msg);
        case GNGLL:
            parseGPGLL(msg);
            break;
        case GNRMC:
//            parseGNRMC(msg);
            break;
        case GNVTG:
//            parseGNVTG(msg);
            break;
        case GNGSA:
//            parseGNGSA(msg);
            break;
        case GNGGA:
            parseGPGGA(msg);
            break;
        case GPZDA:
//            parseGPZDA(msg);
            break;
        case GNZDA:
//            parseGNZDA(msg);
            break;
        case GLZDA:
//            parseGLZDA(msg);
            break;
        case HDT:
//            parseHDT(msg);
            break;
        case ROT:
//            parseROT(msg);
            break;
        case PSAT:
            parsePSAT(msg);
            break;
        case PRDCU:
//            parsePRDCU(msg);
            break;
        case GLGSV:
//            parseGLGSV(msg);
            break;
        case GLGSA:
//            parseGLGSA(msg);
            break;
        case GLGGA:
            parseGPGGA(msg);
            break;
        case GLRMC:
//            parseGLRMC(msg);
            break;
        case GLVTG:
//            parseGLVTG(msg);
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
//    int index =msg.indexOf(44);//ищем первую запятую
//    msg.remove(0, index+1);
//    index =msg.indexOf(44);//ищем запятую
//    if (index == 0)
//    {
//        msg.remove(0, index+1);
//    }
//    if (index > 0)
//    {
//        gps->rms.time = QTime::fromString((msg.mid(0, index-1)), "hhmmss.z");
//        msg.remove(0, index+1);
//    }
//    index =msg.indexOf(44);//ищем запятую
//    if (index == 0)
//    {
//        gps->rms.status = 0;
//        msg.remove(0, index+1);
//    }
//    if (index > 0)
//    {
//        gps->rms.status = (char)msg[0];
//        msg.remove(0, index+1);
//    }
//    index =msg.indexOf(44);//ищем запятую
//    if (index == 0)
//    {
//        gps->rms.lat = 0;
//        msg.remove(0, index+1);
//    }
//    if (index > 0)
//    {
//        gps->rms.lat = atof(msg.mid(0, index-1));
//        msg.remove(0, index+1);
//    }
//    index =msg.indexOf(44);//ищем запятую
//    if (index == 0)
//    {
//        gps->rms.NS = 0;
//        msg.remove(0, index+1);
//    }
//    if (index > 0)
//    {
//        gps->rms.NS = (char)msg[0];
//        msg.remove(0, index+1);
//    }
//    index =msg.indexOf(44);//ищем запятую
//    if (index == 0)
//    {
//        gps->rms.long_ = 0;
//        msg.remove(0, index+1);
//    }
//    if (index > 0)
//    {
//        gps->rms.long_ = atof(msg.mid(0, index-1));
//        msg.remove(0, index+1);
//    }
//    index =msg.indexOf(44);//ищем запятую
//    if (index == 0)
//    {
//        gps->rms.EW = 0;
//        msg.remove(0, index+1);
//    }
//    if (index > 0)
//    {
//        gps->rms.EW = (char)msg[0];
//        msg.remove(0, index+1);
//    }
//    index =msg.indexOf(44);//ищем запятую
//    if (index == 0)
//    {
//        gps->rms.spd = 0;
//        msg.remove(0, index+1);
//    }
//    if (index > 0)
//    {
//        gps->rms.spd = atof(msg.mid(0, index-1));
//        msg.remove(0, index+1);
//    }
//    index =msg.indexOf(44);//ищем запятую
//    if (index == 0)
//    {
//        gps->rms.cog = 0;
//        msg.remove(0, index+1);
//    }
//    if (index > 0)
//    {
//        gps->rms.cog = atof(msg.mid(0, index-1));
//        msg.remove(0, index+1);
//    }
//    index =msg.indexOf(44);//ищем запятую
////    qDebug() << msg;
//    if (index == 0)
//    {
//        msg.remove(0, index+1);
//    }
//    if (index > 0)
//    {
////        qDebug() << msg.mid(0, index);
//        gps->rms.date = QDate::fromString((msg.mid(0, index)), "ddMMyy");
//        msg.remove(0, index+1);
////        qDebug() << gps->rms.date.toString();
//    }
//    index =msg.indexOf(44);//ищем запятую
//    if (index == 0)
//    {
//        gps->rms.mv = 0;
//        msg.remove(0, index+1);
//    }
//    if (index > 0)
//    {
//        gps->rms.mv = atof(msg.mid(0, index-1));
//        msg.remove(0, index+1);
//    }
//    index =msg.indexOf(44);//ищем запятую
//    if (index == 0)
//    {
//        gps->rms.mvEW = 0;
//        msg.remove(0, index+1);
//    }
//    if (index > 0)
//    {
//        gps->rms.mvEW = (char)msg[0];
//        msg.remove(0, index+1);
//    }
//    index =msg.indexOf(44);//ищем запятую
//    if (index == 0)
//    {
//        gps->rms.posMode = 0;
//        msg.remove(0, index+1);
//    }
//    if (index > 0)
//    {
//        gps->rms.posMode = (char)msg[0];
//        msg.remove(0, index+1);
//    }
//    gps->rms.counter +=1;
//    if(test_message)qDebug() << "gps->rms.time:    " << gps->rms.time.toString("hhmmss.z");
//    if(test_message)qDebug() << "gps->rms.status:  " << gps->rms.status;
//    if(test_message)qDebug() << "gps->rms.lat:     " << gps->rms.lat;
//    if(test_message)qDebug() << "gps->rms.NS:      " << gps->rms.NS;
//    if(test_message)qDebug() << "gps->rms.long:    " << gps->rms.long_;
//    if(test_message)qDebug() << "gps->rms.EW:      " << gps->rms.EW;
//    if(test_message)qDebug() << "gps->rms.spd:     " << gps->rms.spd;
//    if(test_message)qDebug() << "gps->rms.cog:     " << gps->rms.cog;
//    if(test_message)qDebug() << "gps->rms.date:    " << gps->rms.date.toString("dd.MM.yy");
//    if(test_message)qDebug() << "gps->rms.mv:      " << gps->rms.mv;
//    if(test_message)qDebug() << "gps->rms.mvEW:    " << gps->rms.mvEW;
//    if(test_message)qDebug() << "gps->rms.posMode: " << gps->rms.posMode;
//    if(test_message)qDebug() << "gps->rms.counter: " << gps->rms.counter;

}

void NMEA0183::parseGLRMC(QByteArray msg)
{

}

void NMEA0183::parseGNVTG(QByteArray msg)
{

}

void NMEA0183::parseGPVTG(QByteArray msg)
{
//    int index = msg.indexOf(',');
//    gps->vtg.trackMadeGood = atof(msg.mid(0, index));
//    msg.remove(0, index + 1);

//    index = msg.indexOf(',');
//    gps->vtg.trackMadeGoodReference = msg.mid(0, 1);
//    msg.remove(0, index + 1);

//    index = msg.indexOf(',');
//    gps->vtg.speedKnots = atof(msg.mid(0, index));
//    msg.remove(0, index + 1);

//    index = msg.indexOf(',');
//    gps->vtg.speedKnotsUnit = msg.mid(0, 1);
//    msg.remove(0, index + 1);

//    index = msg.indexOf(',');
//    gps->vtg.speedKph = atof(msg.mid(0, index));
//    msg.remove(0, index + 1);

//    index = msg.indexOf(',');
//    gps->vtg.speedKphUnit = msg.mid(0, 1);
//    msg.remove(0, index + 1);

//    index = msg.indexOf(',');
//    gps->vtg.posMode = msg.mid(0, 1);

//    if(test_message) qDebug() << "GPVTG Parsed";
}

void NMEA0183::parseGLVTG(QByteArray msg)
{

}

void NMEA0183::parseGPTXT(QByteArray msg)
{
//    // GPTXT сообщение может содержать текстовую информацию
//    gps->txt.id = atoi(msg.mid(0, 2));
//    msg.remove(0, 3);  // Удаляем ID и запятую

//    int index = msg.indexOf(',');
//    gps->txt.text = msg.mid(0, index);
//    if(test_message)qDebug() << "GPTXT Parsed";
}


void NMEA0183::parseGPGGA(QByteArray &msg)
{
    // Пример сообщения: $GPGGA,123519.00,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
    if(test_message)qDebug() << msg;
    QList list = msg.split(',');
    gps->gga.time = QTime::fromString(list[1], "hhmmss.z");
//    gps->gga.time = list[1].toDouble();
    gps->gga.latitude       = list[2].toDouble();
    if (list[3].size()>0) gps->gga.latHemisphere  = QString::fromUtf8(list[3]);
    gps->gga.longitude      = list[4].toDouble();
    if (list[5].size()>0) gps->gga.lonHemisphere  = QString::fromUtf8(list[5]);
    gps->gga.quality        = list[6].toInt();
    gps->gga.satellitesUsed = list[7].toInt();
    gps->gga.hdop           = list[8].toDouble();
    gps->gga.altitude       = list[9].toDouble();
    if (list[10].size()>0) gps->gga.altitudeUnit   = QString::fromUtf8(list[10]);
    gps->gga.geoidHeight    = list[11].toDouble();
    if (list[12].size()>0) gps->gga.geoidUnit      = QString::fromUtf8(list[12]);
    gps->gga.dgpsAge        = list[13].toDouble();
    gps->gga.dgpsStationId  = list[14].toInt();

    qDebug() << "gps->gga.time" <<      gps->gga.time  ;
    qDebug() << "gps->gga.latitude" <<  QString::number(gps->gga.latitude ,'f', 16);
    qDebug() << "gps->gga.longitude" << QString::number(gps->gga.longitude,'f', 16);
    if(test_message) qDebug() << "GPGGA Parsed";
}

void NMEA0183::parseGNRMC(QByteArray msg)
{

}

void NMEA0183::parseGPGSA(QByteArray msg)
{
//    // Парсинг GSA сообщений
//    int index = msg.indexOf(',');
//    gps->gsa.mode = msg.mid(0, 1);
//    msg.remove(0, index + 1);

//    index = msg.indexOf(',');
//    gps->gsa.fixType = atoi(msg.mid(0, index));
//    msg.remove(0, index + 1);

//    for (int i = 0; i < 12; i++) {
//        index = msg.indexOf(',');
//        gps->gsa.satellites[i] = atoi(msg.mid(0, index));
//        msg.remove(0, index + 1);
//    }

//    index = msg.indexOf(',');
//    gps->gsa.pdop = atof(msg.mid(0, index));
//    msg.remove(0, index + 1);

//    index = msg.indexOf(',');
//    gps->gsa.hdop = atof(msg.mid(0, index));
//    msg.remove(0, index + 1);

//    index = msg.indexOf(',');
//    gps->gsa.vdop = atof(msg.mid(0, index));

//    if(test_message)qDebug() << "GPGSA Parsed";
}

void NMEA0183::parseGLGSA(QByteArray msg)
{

}

void NMEA0183::parsePGRMT(QByteArray msg)
{

}

void NMEA0183::parseGPZDA(QByteArray msg)
{
//    int index = msg.indexOf(',');
//    gps->zda.time = QTime::fromString(msg.mid(0, index), "hhmmss.z");
//    msg.remove(0, index + 1);
//    if(test_message)qDebug() << "gps->zda.time: " << gps->zda.time;
//    index = msg.indexOf(',');
//    gps->zda.day = atoi(msg.mid(0, index));
//    msg.remove(0, index + 1);
//    if(test_message)qDebug() << "gps->zda.day: " << gps->zda.day;
//    index = msg.indexOf(',');
//    gps->zda.month = atoi(msg.mid(0, index));
//    msg.remove(0, index + 1);
//    if(test_message)qDebug() << "gps->zda.month: " << gps->zda.month;
//    index = msg.indexOf(',');
//    gps->zda.year = atoi(msg.mid(0, index));
//    msg.remove(0, index + 1);
//    if(test_message)qDebug() << "gps->zda.year: " << gps->zda.year;
//    index = msg.indexOf(',');
//    gps->zda.localZoneHours = atoi(msg.mid(0, index));
//    msg.remove(0, index + 1);
//    if(test_message)qDebug() << "gps->zda.localZoneHours: " << gps->zda.localZoneHours;
//    index = msg.indexOf(',');
//    gps->zda.localZoneMinutes = atoi(msg.mid(0, index));
//    if(test_message)qDebug() << "gps->zda.localZoneMinutes: " << gps->zda.localZoneMinutes;
//    if(test_message)qDebug() << "GPZDA Parsed";
}

void NMEA0183::parseGNZDA(QByteArray msg)
{

}

void NMEA0183::parseGLZDA(QByteArray msg)
{

}

void NMEA0183::parseHDT(QByteArray msg)
{
//    int index = msg.indexOf(',');
//    gps->hdt.heading = atof(msg.mid(0, index));
//    msg.remove(0, index + 1);

//    gps->hdt.trueIndicator = msg.mid(0, 1);
//    if(test_message)qDebug() << "HDT Parsed";
}

void NMEA0183::parseROT(QByteArray msg)
{
//    int index = msg.indexOf(',');
//    gps->rot.rateOfTurn = atof(msg.mid(0, index));
//    msg.remove(0, index + 1);

//    gps->rot.status = msg.mid(0, 1);
//    if(test_message)qDebug() << "ROT Parsed";
}

void NMEA0183::parsePSAT(QByteArray &msg)
{
    // Пример сообщения: $PSAT,HPR,123519.00,182.7,0.5,-1.3,N*6D
//    qDebug() << msg;
    QList list = msg.split(',');
    gps->psat.time = QTime::fromString(list[2], "hhmmss.z");
    gps->psat.yaw   = list[3].toDouble();
    gps->psat.pitch = list[4].toDouble();
    gps->psat.roll  = list[5].toDouble();
//    qDebug() << "gps->psat.yaw   ==" << gps->psat.yaw   ;
//    qDebug() << "gps->psat.pitch ==" << gps->psat.pitch ;
//    qDebug() << "gps->psat.roll  ==" << gps->psat.roll  ;
    if (list[6].size()>0) gps->psat.dataType = QString::fromUtf8(list[6]);
//    if(test_message) qDebug() << "PSAT Parsed";
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

void NMEA0183::parseGPGLL(QByteArray &msg)
{
    if(test_message)qDebug() << msg;
    QList list = msg.split(',');
    gps->gll.lat = list[1].toDouble();
    if (list[2].size()>0) gps->gll.NS = QString::fromUtf8(list[2]);
    gps->gll.long_ = list[3].toDouble();
    if (list[4].size()>0) gps->gll.EW = QString::fromUtf8(list[4]);
    gps->gll.time = QTime::fromString(list[5], "hhmmss.z");
    if (list[6].size()>0) gps->gll.status = QString::fromUtf8(list[6]);
    if (list[7].size()>0) gps->gll.posMode = QString::fromUtf8(list[7]);

    if(test_message)qDebug() << "gps->gll.lat:       " << gps->gll.lat;
    if(test_message)qDebug() << "gps->gll.NS:        " << gps->gll.NS;
    if(test_message)qDebug() << "gps->gll.long_:     " << gps->gll.long_;
    if(test_message)qDebug() << "gps->gll.EW:        " << gps->gll.EW;
//      qDebug() << "gps->gll.time:      " << gps->gll.time_h << ":" << gps->gll.time_m << ":" \
//               << gps->gll.time_s << ":" << gps->gll.time_ms;
    if(test_message)qDebug() << "gps->gll.time:      " << gps->gll.time;
    if(test_message)qDebug() << "gps->gll.status:    " << gps->gll.status;
    if(test_message)qDebug() << "gps->gll.posMode:   " << gps->gll.posMode;
}

} //end namespace NMEA
