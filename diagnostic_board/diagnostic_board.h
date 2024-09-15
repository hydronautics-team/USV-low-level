#ifndef DIAGNOSTIC_BOARD_H
#define DIAGNOSTIC_BOARD_H

#include <QObject>
#include <QSerialPort>
#include <QTimer>
#include <QDebug>
#include <QString>

struct Header_Diagn {
    quint8 first = 0xff;
    quint8 second = 0xfd;
};

struct Diagnostic_values {
    Header_Diagn header;
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

class Diagnostic_board : public QObject
{
    Q_OBJECT
public:
    Diagnostic_board(QString portName, int baudRate = 115200,
                     QObject *parent = nullptr);
    void getdata();
    QSerialPort diagnostic;
    QByteArray diagnostic_buffer;
    Diagnostic_values data;
    quint16 Twobytetoint(QByteArray const buf);
private slots:
    void parser();

};
#endif // DIAGNOSTIC_BOARD_H
