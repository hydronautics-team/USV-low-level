#ifndef PA500_H
#define PA500_H

#include <QSerialPort>
#include <QDebug>
#include <QObject>

class PA500 : public QObject
{
    Q_OBJECT

public:
    explicit PA500(const QString &portName = "/dev/ttyUSB1", int baudrate = 9600);
    ~PA500();
    void readData(); // обработка данных из посылки эхолота

signals:
    void sendDistance(float d); // сигнал для передачи данных

private:
    QSerialPort *ser_PA500;
    QString PA500_port_name;
    int PA500_baudrate;
};

#endif // PA500_H
