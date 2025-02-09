#ifndef PA500_H
#define PA500_H

#include <QSerialPort>
#include <QDebug>
#include <QObject>

struct EcholotStr
{
    float depth = 0;
    uint16_t counter = 0;
};

class PA500 : public QObject
{
    Q_OBJECT

public:
    explicit PA500(const QString &portName = "/dev/ttyUSB1", int baudrate = 9600);
    ~PA500();
    void readData(); // обработка данных из посылки эхолота

    EcholotStr echo;

signals:
    void sendDistance(EcholotStr *echo); // сигнал для передачи данных

private:
    QSerialPort *ser_PA500;
    QString PA500_port_name;
    int PA500_baudrate;
};

#endif // PA500_H
