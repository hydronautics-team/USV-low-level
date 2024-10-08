#include "coordssp.h"

CoordSSP::CoordSSP(QObject *parent)
    : QObject{parent}
{}

bool CoordSSP::gpsToUTM(const GPSPoint &gps, UTMPoint &utm)
{
    try {
        double easting, northing;
        int zone;
        bool north;

        // Используем GeographicLib для преобразования
        GeographicLib::UTMUPS::Forward(
            gps.latitude,
            gps.longitude,
            zone,
            north,
            easting,
            northing
            );

        utm.zone = zone;
        utm.north = north;
        utm.easting = easting;
        utm.northing = northing;

        return true;
    }
    catch (const std::exception& e) {
        qDebug() << "Ошибка преобразования GPS в UTM:" << e.what();
        return false;
    }
}

void CoordSSP::setReferencePoint(const GPSPoint &gps)
{
    if (gpsToUTM(gps, referenceUTM)) {
        qDebug() << "Реперная точка установлена:"
                 << "Zone:" << referenceUTM.zone
                 << (referenceUTM.north ? "N" : "S")
                 << "Easting:" << referenceUTM.easting
                 << "Northing:" << referenceUTM.northing;
    }
    else {
        qDebug() << "Ошибка установки реперной точки.";
    }
}

void CoordSSP::getLocalCoordinates(const GPSPoint &currentGPS, double &dx, double &dy)
{
    UTMPoint currentUTM;

    if (gpsToUTM(currentGPS, currentUTM)) {
        // Проверяем, находятся ли текущая точка и репер в одной зоне и полушарии
        if (currentUTM.zone == referenceUTM.zone && currentUTM.north == referenceUTM.north) {
            // Рассчитываем смещение от репера
            dx = currentUTM.easting - referenceUTM.easting;
            dy = currentUTM.northing - referenceUTM.northing;

            qDebug() << "Текущие локальные координаты: dx =" << dx << "м, dy =" << dy << "м";
        }
        else {
            qDebug() << "Текущая точка находится в другой зоне или полушарии.";
            dx = 0;
            dy = 0;
        }
    }
    else {
        qDebug() << "Ошибка преобразования текущих GPS-координат в UTM.";
        dx = 0;
        dy = 0;
    }
}

double CoordSSP::degreesToRadians(double degrees)
{
    return degrees * M_PI / 180.0;
}
