#include "diagnostic_board.h"


Diagnostic_board::Diagnostic_board(QString portName, int baudRate, QObject *parent)
{
    diagnostic.setBaudRate(baudRate);
    diagnostic.setPortName(portName);
    diagnostic.setDataBits(QSerialPort::Data8);
    diagnostic.setStopBits(QSerialPort::OneStop);
    diagnostic.setParity(QSerialPort::NoParity);
    diagnostic.setFlowControl(QSerialPort::NoFlowControl);
    diagnostic.open(QIODevice::ReadWrite);

    if (diagnostic.isOpen()){
        qDebug()<<" port was opened";
    }
    else {
        qDebug()<<" error open port "<< diagnostic.errorString();
       }
    connect(&diagnostic, &QSerialPort::readyRead, this, &Diagnostic_board::getdata);
}

quint16 Diagnostic_board::Twobytetoint(QByteArray const buf) {

    return ((buf[0] << 8) | buf[1]);
}

void Diagnostic_board::getdata()
{
    diagnostic_buffer.append(diagnostic.readAll());
    parser();
}

void Diagnostic_board::parser()
{
    QByteArray hed(*(quint8*) &(data.header),sizeof(Header_Diagn));
    int index = diagnostic_buffer.indexOf(hed);
    if (index == -1) {
        // Не найдено сообщение
        qDebug() << "нет сообщения в буфере ";
        return;
    }
    if ( diagnostic_buffer.size() <= index + 20 ) {
        return;
    }
    auto tmp = diagnostic_buffer.mid(index, 21);
    Diagnostic_values msg;

    msg.voltage_bat = Twobytetoint(tmp.mid(2, 1));
    msg.voltage_12v = Twobytetoint(tmp.mid(4, 1));
    msg.voltage_5v = Twobytetoint(tmp.mid(6, 1));
    msg.voltage_4 = Twobytetoint(tmp.mid(8, 1));
    msg.current_1 = Twobytetoint(tmp.mid(10, 1));
    msg.current_2 = Twobytetoint(tmp.mid(12, 1));
    msg.current_3 =Twobytetoint(tmp.mid(14, 1));
    msg.current_4 = Twobytetoint(tmp.mid(16, 1));
    msg.PMW1 = tmp[18];
    msg.PMW2 = tmp[19];
    msg.PMW3 = tmp[20];
    msg.PMW4 = tmp[21];

    data = msg;

    diagnostic_buffer.remove(0, index+21);
}
