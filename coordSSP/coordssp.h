#ifndef COORDSSP_H
#define COORDSSP_H

#include <QObject>
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <cmath>
#include <QDebug>

// Структура для хранения GPS координат (широта и долгота)
struct GPSPoint {
    double latitude  = 0;   // В градусах
    double longitude = 0;  // В градусах
};

// Структура для хранения декартовых координат UTM
struct UTMPoint {
    int zone        = 0;          // Зона UTM
    bool north      = 0;        // Полушарие: true для северного, false для южного
    double easting  = 0;    // В метрах
    double northing = 0;   // В метрах
};

struct SSPdata {
    uint16_t counter = 0;
    double latitudeReper  = 0;   // В градусах
    double longitudeReper = 0;  // В градусах
    float X  = 0;
    float Y = 0;
    float yawMagn = 0;
    float yawIner = 0;
    float yaw = 0;
    float pitch = 0;
    float roll = 0;
    float X_accel   = 0;
    float Y_accel   = 0;
    float Z_accel   = 0;

    float X_rate    = 0;
    float Y_rate    = 0;
    float Z_rate    = 0;

    float X_magn    = 0;
    float Y_magn    = 0;
    float Z_magn    = 0;

    float quat [4];
    float VMA1     = 0;
    float VMA2     = 0;
    float VMA3     = 0;
    float VMA4     = 0;
    quint8 flag_yaw = 1;
    quint8 flag_pitch = 1;
    quint8 flag_roll = 1;
    quint8 flag_march = 1;
    quint8 flag_depth = 1;
    quint8 flag_lag = 1;
    quint8 modeReal = 0;///<текущий режим
    float yawSet = 0;
    double latitudePerpose  = 0;   // В градусах
    double longitudePerpose = 0;  // В градусах

};

class CoordSSP : public QObject
{
    Q_OBJECT
public:
    explicit CoordSSP(QObject *parent = nullptr);
    // Функция для преобразования GPS координат в UTM относительно репера (начальной точки)
    bool gpsToUTM(const GPSPoint& gps, UTMPoint& utm);


    // Метод для установки реперной точки
    void setReferencePoint(const GPSPoint& gps);

    // Метод для расчета текущей координаты в локальной системе отсчета (СКО)
    void getLocalCoordinates(const GPSPoint& currentGPS, double& dx, double& dy);

private:
    GPSPoint reper; //координаты репера
    // Реперная точка
    UTMPoint referenceUTM;


    // Преобразование градусов в радианы
    double degreesToRadians(double degrees);
signals:
};

#endif // COORDSSP_H
