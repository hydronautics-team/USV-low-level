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
