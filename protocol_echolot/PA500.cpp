#include "PA500.h"

PA500::PA500(const QString &portName, int baudrate) :
    PA500_port_name(portName),
    PA500_baudrate(baudrate)
{
    // Открываем и настраиваем COM-порт
    ser_PA500 = new QSerialPort(PA500_port_name);
    ser_PA500->setBaudRate(PA500_baudrate);
    ser_PA500->setStopBits(QSerialPort::OneStop);
    ser_PA500->setDataBits(QSerialPort::Data8);
    
    if (ser_PA500->open(QIODevice::ReadWrite))
    {
        qDebug() << PA500_port_name << " is opened successfully";
        connect(ser_PA500, &QSerialPort::readyRead, this, &PA500::readData);
    }
    else
    {
        qDebug() << "Unable to open" << PA500_port_name;
    }
}

void PA500::readData() // Обработка данных из посылки эхолота
{
    QByteArray tmp = ser_PA500->readAll();
    qDebug() << "Got from port:" << tmp;

    // Данные приходят в формате "050.000m" (50 м)
    if (tmp.contains('m'))
    {
        float dist = tmp.left(tmp.indexOf("m")).toFloat();
        qDebug() << "Depth data =" << dist;
        emit sendDistance(dist); // Передаем данные о глубине через сигнал
    }
}

PA500::~PA500()
{
    if (ser_PA500->isOpen())
    {
        ser_PA500->close();
    }
    delete ser_PA500;
}
