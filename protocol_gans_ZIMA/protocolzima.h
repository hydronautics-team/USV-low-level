#ifndef PROTOCOLZIMA_H
#define PROTOCOLZIMA_H

#include <QObject>
#include <QSerialPort>
#include <QTimer>
#include <QDebug>
#include <QString>

/// Cсылка на документацию:
/// https://docs.unavlab.com/documentation/RU/Zima/Zima_Protocol_Specification_ru.html

#pragma pack(push,1)

enum TitleZIMA
{
    PZMAE,
    PZMAF,
    PZMAG,
    PZMA0,
    UNKNOWN
};

struct stPZMAE // Ответ удаленного ответчика.
{
    qint8 TargetID     = 0; // Адрес запрошенного ответчика
    qint16 RequestID   = 0; // Идентификатор запроса (см. 3.5.)
    qint8  dFlag       = 0; // Зарезервированно
    double Azimuth     = 0; // Горизонтальный угол на ответчик, град.
    double Distance    = 0; // Дистанция до ответчика, м
    double DataValue   = 0; // Значение запрошенного параметра
    double SNR         = 0; // Соотношение сигнал-шум, дБ
    double DPL         = 0; // Допплеровское смещение, Гц
};

struct stPZMAF ///< Cостояние системы
{
    double Temperature      = 0; // Температура воды, °С
    double Depth            = 0; // Глубина базовой станции от поверхности, м
    double isAHRSEnabled    = 0; // Состояние AHRS
    double TRX_State        = 0; //	Состояние приемопередатчика
};

struct stPZMAG ///< Показания встроенного инклинометра*
{
    double Roll  = 0; // Крен, °. 0 - вертикальное положение, 0..+90 - поворот на правый борт, 0..-90 - поворот на левый борт
    double Pitch = 0; // Дифферент, °. 0 - вертикальное положение, 0..+90 - крен на нос, 0..-90 - крен на корму
};

struct stPZMA0 ///< Сообщение IC_D2H_ACK - реакция устройства на поступивший от управляющей системы запрос.
{
    int Error_code = 0; ///< Код ошибки
        // ‘0’	NO_ERROR	Запрос принят
        // ‘1’	INVALID_SYNTAX	Ошибка синтаксиса
        // ‘2’	UNSUPPORTED	Команда не поддерживается
        // ‘3’	TRANSMITTER_BUSY	Передатчик занят
        // ‘4’	ARGUMENT_OUT_OF_RANGE	Аргумент/параметр вне диапазона допустимых значений
        // ‘5’	INVALID_OPERATION	Невозможно выполнить операцию в данный момент
        // ‘6’	UNKNOWN_FIELD_ID	Неизвестное/неподдерживаемое поле
        // ‘7’	VALUE_UNAVAILIBLE	Запрошенное значение недоступно
        // ‘8’	RECEIVER_BUSY	Приемник занят
        // ‘9’	WAKE_UP	Управление энергосберегающим режимом. Ответчик высылает сообщение об ошибке с данным параметром сразу после пробуждения
        // ‘10’	STAND_BY	Управление энергосберегающим режимом. Ответчик высылает сообщение об ошибке с данным параметром перед уходом в спящий режим
};

struct ZimaData  ///< Данные, которые поступают от девайса в управляющую систему
{
    stPZMAE pzmae; // Ответ удаленного ответчика.
    stPZMAF pzmaf; // Cостояние системы
    stPZMAG pzmag; // Показания встроенного инклинометра*
    stPZMA0 pzma0; // Сообщение IC_D2H_ACK - реакция устройства на поступивший от управляющей системы запрос.
    //следующих сообщений нет у Анны Сергеевны
    // PZMA3 pzma3; // Значение настроечного поля.
    // PZMA6 pzma6; // Значение локального параметра.
    // PZMAA pzmaa; // Навигационные данные (ответчик).
    // PZMAB pzmab; // Запрос базовой станции (ответчик).
    // PZMAD pzmad; // Таймаут удаленного ответчика.
    // PZMAI pzmai; // Информация об устройстве. PZMA!

};

#pragma pack(pop)

class ProtocolZIMA : public QObject
{
    Q_OBJECT

public:
    explicit ProtocolZIMA(QString portName, int baudRate = 9600,
                          QObject *parent = nullptr);
    ZimaData data; // Данные от ГАНС

    private:
    void findTitle(qint8 index, qint8 crc_in, uint end, QByteArray title); //поиск заголовка
    void parseBuffer();
    QSerialPort zima;
    QByteArray zima_buffer;
    int crc (QByteArray msg);
    int crc_real(qint8 crc_in);
    bool test_messege = true; ///< Вывод сообщений отладки
    //Команды для
    void parsePZMAE(QByteArray msg); // Ответ удаленного ответчика.
    void parsePZMAF(QByteArray msg); // Cостояние системы
    void parsePZMAG(QByteArray msg); // Показания встроенного инклинометра*
    void parsePZMA0(QByteArray msg); // Сообщение IC_D2H_ACK - реакция устройства на поступивший от управляющей системы запрос.

    QTimer* timer_send; //таймер для отправки данных на ГАНС

private slots:
    void sendSoundSlot();
};

#endif // PROTOCOLZIMA_H
