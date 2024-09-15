#ifndef PROTOCOL_H
#define PROTOCOL_H

#include "stdint.h"
#include <QDebug>
#include <QTime>

#pragma pack(push,1)

enum class e_CSMode : quint8
{ //режим работы
    MODE_MANUAL = 0, //ручной
    MODE_AUTOMATED, //автоматизированный
    MODE_AUTOMATIC, //автоматический
    MODE_GROUP //групповой
};

enum class power_Mode : quint8
{ //режим работы
    MODE_2 = 0, //включены вычислитель, wifi, uwb
    MODE_3, //включены вычислитель, wifi, uwb, гидроакустика
    MODE_4, //включены вычислитель, wifi, uwb, гидроакустика, ВМА
    MODE_5 //выключить вычислитель на 5 секунд и включить обратно
};

enum class mission_Control : quint8
{ //команды управления миссией
    MODE_IDLE = 0, //ожидание
    MODE_START, //отправка запроса на выполнение миссии
    MODE_CANCEL, //отмена выполнения миссии
    MODE_STOP, //пауза, остановить временно
    MODE_COMPLETE //завершить миссию
};

enum class mission_Status : quint8
{ //состояние миссии
    MODE_IDLE = 0, //ожидание
    MODE_ERROR, //ошибка инициализации миссии
    MODE_RUNNING, //миссия запущена и выполняется
    MODE_STOPPED, //миссия приостановлена, на паузе
    MODE_PERFOMED, //миссия завершена
};

//структура данных, которая передается из Северова в Пульт
//тут описаны данные, которые Пульт принимает от Северова

struct ControlData
{ //данные, которые идут с пульта в СУ
    float yaw       = 0;
    float pitch     = 0;
    float roll      = 0;
    float march     = 0;
    float depth     = 0;
    float lag       = 0;
};

struct ControlVMA
{ //данные, которые идут на каждый ВМА
    float VMA1     = 0;
    float VMA2     = 0;
    float VMA3     = 0;
    float VMA4     = 0;
};

struct ControlContoursFlags
{ //флаги замыкания контуров (если 1, то замкнуты, 0 - разомкнуты)
    quint8 yaw = 1;
    quint8 pitch = 1;
    quint8 roll = 1;
    quint8 march = 1;
    quint8 depth = 1;
    quint8 lag = 1;
};


struct AUVCurrentData
{
    //!!тут все текущие параметры АНПА
    quint8 modeReal = 0;//текущий режим
    ControlContoursFlags controlReal;//текущее состояние контуров, чтобы проверить что сигнал с пульта был обработан
    quint8 modeAUV_Real = 0;//текущий выбор модель/реальный НПА
    ControlData ControlDataReal;//текущие курс, дифферент, крен, значения с имушки по сути
    ControlVMA signalVMA_real;//управление на ВМА (текущие управляющие сигнлы на движители)
};


struct HeaderSwap
{
    int senderID=0;
    int receiverID=0;
    int msgSize=0;
};

struct DataAH127C
{ //структура данных с датчика бесплатформенной системы ориентации
    float yaw       = 0; //курс градусы +/- 180
    float pitch     = 0; //...
    float roll      = 0;

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
};

struct FlagAH127C_bort
{
    quint8 startCalibration = 0;
    quint8 endCalibration = 0;
};

struct FlagAH127C_pult
{
    quint8 initCalibration = 0;
    quint8 saveCalibration = 0;
};

struct DataGANS
{ //структура данных с ГАНСА

    double azimuth     = 0; // Горизонтальный угол на ответчик, град.
    double distance    = 0; // Дистанция до ответчика, м
    double dataValue   = 0; // Значение запрошенного параметра

    double temperature      = 0; // Температура воды, °С
    double depth            = 0; // Глубина базовой станции от поверхности, м

    double roll_GANS  = 0; // Крен, °. 0 - вертикальное положение, 0..+90 - поворот на правый борт, 0..-90 - поворот на левый борт
    double pitch_GANS = 0; // Дифферент, °. 0 - вертикальное положение, 0..+90 - крен на нос, 0..-90 - крен на корму
};

struct GPS_angular
{
    QTime time_UTC;    // Время UTC
    double yaw;        // Курс (рысканье)
    double pitch;      // Килевая качка
    double roll;       // Бортовая качка
    QString dataType;  // Тип данных (N - курс от GPS, G - гиро курс)
};

struct GPS_coordinate
{
    QTime time;           // UTC Время обсервации
    double latitude;      // Широта
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

struct Diagnostic {
    quint16 voltage_bat;
    quint16 voltage_5v;
    quint16 voltage_12v;
    quint16 voltage_4;
    quint16 current_1;
    quint16 current_2;
    quint16 current_3;
    quint16 current_4;
    quint8 PMW1;
    quint8 PMW2;
    quint8 PMW3;
    quint8 PMW4;
};

struct ToPult
{
    ToPult(int auvID=0)
    {
        headerSwap.senderID = auvID;
        headerSwap.receiverID = 0;
        headerSwap.msgSize = sizeof (ToPult);
    }

    HeaderSwap headerSwap;
    AUVCurrentData auvData;// данные о текущих параметрах
    DataAH127C dataAH127C;// данные с БСО
    FlagAH127C_bort flagAH127C_bort;
    DataGANS dataGANS;
    GPS_angular angularGPS;
    GPS_coordinate coordinateGPS;
    Diagnostic diagnostics;
    quint8 ID_mission = 0;
    mission_Status missionStatus = mission_Status::MODE_IDLE;
    uint checksum;
};

//структура данных, которая передается из пульта в АНПА
struct FromPult
{
    ControlData controlData; //данные, которые идут с пульта при замыканиии контуров
    e_CSMode cSMode = e_CSMode::MODE_MANUAL; //режим работы
    ControlContoursFlags controlContoursFlags; //флаги замыкания контуров (если больше 0, то замкнуты
    quint8 modeAUV_selection = 1;//текущий выбор модель/реальный НПА
    power_Mode pMode = power_Mode::MODE_2; //режим работы системы питания, структура с желаемыми параметрами системы питания
    FlagAH127C_pult flagAH127C_pult;
    quint8 ID_mission_AUV = 0;
    mission_Control missionControl = mission_Control::MODE_IDLE;
    uint checksum;

};

#pragma pack (pop)


#endif // PROTOCOL_H
