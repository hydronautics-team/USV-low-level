#ifndef NMEA0183_H
#define NMEA0183_H

#include <QObject>
#include <QString>
#include <QSerialPort>
#include <QDebug>
#include <QString>
#include <QTimer>
#include <QTime>
#include <QDate>
#include <QFile>
#include <QList>



namespace NMEA {

#pragma pack(push,1)

enum TitleNMEA
{
    GNGGA = 1,  ///<  GNSS fix data
    GPGGA = 2,      ///<  GPS fix data
    GLGGA = 3,      ///<  GLONASS fix data
    GNRMC = 4,      ///<  Recomended minimum specific
    GPRMC = 5,      ///<  Recomended minimum specific GPS-only
    GLRMC = 6,      ///<  Recomended minimum specific GLONASS-only
    GNVTG = 7,      ///<  Track made good and ground speed
    GPVTG = 8,      ///<  Track made good and ground speed
    GLVTG = 9,      ///<  Track made good and ground speed
    GPGSV = 10,      ///<  GPS satellites in view
    GLGSV = 11,      ///<  GLONASS satellites in view
    GNGSA = 12,      ///<  GNS DOP and active satelites
    GPGSA = 13,      ///<  GPS DOP and active satelites
    GLGSA = 14,      ///<  GPS DOP and active satelites
    HDT   = 15,        ///<  Истинный курс
    ROT   = 16,        ///<  Рысканье по курсу
    PGRMT = 17,      ///<  Receiver info
    GPZDA = 18,      ///<  Время
    GNZDA = 19,      ///<  Время
    GLZDA = 20,      ///<  Время
    PSAT  = 21,       ///<  Крен, дифферент
    PRDCU = 22,       ///<  Крен, дифферент
    GPTXT = 23,
    GPGLL = 24,
    GNGLL = 25,
    GAGSV = 26,
    UNKNOWN = 27
};

struct GLL
{
    double lat = 0;
    QString NS = 0;
    double long_ = 0;
    QString EW = 0;
    QTime time;
    QString status = 0;
    QString posMode = 0;
};

struct stPSAT
{
    QTime time;        // Время UTC
    double yaw;        // Курс (рысканье)
    double pitch;      // Килевая качка
    double roll;       // Бортовая качка
    QString dataType;  // Тип данных (N - курс от GPS, G - гиро курс)
};

struct RMS
{
    QTime time;
    char status = 0;
    float lat = 0;
    char NS = 0;
    float long_ = 0;
    char EW = 0;
    float spd = 0;
    float cog = 0;
    QDate date;
    float mv;
    char mvEW;
    char posMode;
    int counter = 0;
};

struct GGA
{
    QTime time;           // UTC Время обсервации
    double latitude = 0;      // Широта
    QString latHemisphere;// Полушарие (N/S)
    double longitude;     // Долгота
    QString lonHemisphere;// Полушарие (E/W)
    int quality;          // Индикатор качества обсервации
    int satellitesUsed;   // Количество спутников
    double hdop;          // Величина горизонтального геометрического фактора (HDOP)
    double altitude;      // Высота антенны над уровнем моря (геоидом)
    QString altitudeUnit; // Единица измерения высоты (м)
    double geoidHeight;   // Превышение геоида над эллипсоидом WGS84
    QString geoidUnit;    // Единица измерения превышения геоида (м)
    double dgpsAge = 0;       // Возраст дифференциальной поправки
    int dgpsStationId = 0;    // Идентификатор ККС
};

struct RMC {
    QTime time;          // Время GPS (UTC)
    QString status;      // Статус (A = активный, V = неактивный)
    double lat;          // Широта
    QString NS;          // Север/Юг
    double lon;          // Долгота
    QString EW;          // Восток/Запад
    double speedKnots;   // Скорость в узлах
    double course;       // Направление
    QDate date;          // Дата
    double magneticVariation; // Магнитное отклонение
    QString magneticEW;  // Восток/Запад магнитного отклонения
    QString posMode;     // Режим определения позиции
};

struct VTG {
    double trackMadeGood;      // Направление (по истинному северу)
    QString trackMadeGoodReference; // Ссылка на направление (T = истинный север)
    double speedKnots;         // Скорость в узлах
    QString speedKnotsUnit;    // Единица измерения скорости (N = узлы)
    double speedKph;           // Скорость в км/ч
    QString speedKphUnit;      // Единица измерения скорости (K = км/ч)
    QString posMode;           // Режим определения позиции
};

struct GSA {
    QString mode;          // Автономный/ручной режим (M = ручной, A = автоматический)
    int fixType;           // Тип фиксации (1 = нет фиксации, 2 = 2D фиксация, 3 = 3D фиксация)
    int satellites[12];    // Список PRN спутников, используемых в фиксации
    double pdop;           // PDOP (позиционная точность)
    double hdop;           // HDOP (горизонтальная точность)
    double vdop;           // VDOP (вертикальная точность)
};

struct ZDA {
    QTime time;            // Время GPS (UTC)
    int day;               // День
    int month;             // Месяц
    int year;              // Год
    int localZoneHours;    // Часовой пояс (часы)
    int localZoneMinutes;  // Часовой пояс (минуты)
};

struct stHDT {
    double heading;        // Направление по истинному северу
    QString trueIndicator; // Идентификатор истинного севера (T)
};

struct stROT {
    double rateOfTurn;     // Скорость изменения курса
    QString status;        // Статус (A = активный, V = неактивный)
};

struct TXT {
    int id;                // Идентификатор сообщения
    QString text;          // Текст сообщения
};

struct GPSData {
    GGA gga;
    RMC rmc;
    VTG vtg;
    GSA gsa;
    ZDA zda;
    stHDT hdt;
    stROT rot;
    TXT txt;
};


struct GPS
{
    GLL gll;
//    RMS rms;
    GGA gga;
//    RMC rmc;
//    VTG vtg;
//    GSA gsa;
//    ZDA zda;
//    stHDT hdt;
//    stROT rot;
//    TXT txt;
    stPSAT psat;
};

#pragma pack(pop)

class NMEA0183 : public QObject
{
    Q_OBJECT
public:
    explicit NMEA0183(QString portName, int baudRate = 115200, QObject *parent = 0);
    void readData();
    GPS *gps;
protected:
    QFile fileGPS;
    void findTitleNMEA(qint8 &index, qint8 &crc_in, qint8 &end, QByteArray &title); //поиск заголовка
    QSerialPort gps_port;
    QByteArray gps_buffer;
    void parseBuffer();
    void parseGNGGA(QByteArray msg);
    void parseGPGGA(QByteArray &msg);
    void parseGNRMC(QByteArray msg);
    void parseGPRMC(QByteArray msg);
    void parseGLRMC(QByteArray msg);
    void parseGNVTG(QByteArray msg);
    void parseGPVTG(QByteArray msg);
    void parseGLVTG(QByteArray msg);
    void parseGPGSV(QByteArray msg);
    void parseGLGSV(QByteArray msg);
    void parseGNGSA(QByteArray msg);
    void parseGPGSA(QByteArray msg);
    void parseGLGSA(QByteArray msg);
    void parsePGRMT(QByteArray msg);
    void parseGPZDA(QByteArray msg);
    void parseGNZDA(QByteArray msg);
    void parseGLZDA(QByteArray msg);
    void parseHDT(QByteArray msg);
    void parseROT(QByteArray msg);
    void parsePSAT(QByteArray &msg);
    void parsePRDCU(QByteArray msg);
    void parseGPTXT(QByteArray msg);
    void parseGPGLL(QByteArray &msg);


    int crc (QByteArray msg);
    int crc_real_method(QByteArray gps_buffer, uint crc_in);
    bool test_message = false;
    QTimer timer;


};

} //end namespace NMEA

#endif // NMEA0183_H
