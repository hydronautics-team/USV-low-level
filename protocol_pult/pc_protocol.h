#ifndef PC_PROTOCOL_H
#define PC_PROTOCOL_H

#include "udp_protocol.h"
#include "protocol.h"

//const QString ConfigFile = "protocols.conf";
//класс обмена АНПА- планировщик, который создается в АНПА
namespace ControlSystem {
class PC_Protocol: public QObject, public MetaUdpProtocol {
    Q_OBJECT
public:
explicit PC_Protocol(const QString & config = "protocols.conf",
                       const QString & name = "surface_agent", QObject *parent = 0){
    udpProtocol = new UdpProtocol <FromPult, ToPult> (config, name, parent);
    connect(timer,SIGNAL(timeout()),SLOT(sendData()));
    connect(udpProtocol->getReceiveSocket(),SIGNAL(readyRead()),SLOT(receiveData()));
    set_ip_receiver(udpProtocol->ip_receiver());
    set_ip_sender (udpProtocol->ip_sender());
    set_port_receiver(udpProtocol->port_receiver());
    set_port_sender (udpProtocol->port_sender());
}
signals:
    void dataReceived();
public slots:
    //запуск обмена
    void startExchange(){
        timer->start(1000/udpProtocol->getFrequency());
    }
    //остановить обмен
    void stopExhange(){
        timer->stop();
    }
    void sendData(){
 //       qDebug()<<"send data";
        udpProtocol->send_data = send_data;
        udpProtocol->sendData();
    }
    void receiveData(){
        udpProtocol->receiveData();
        rec_data = udpProtocol->rec_data;
        emit dataReceived();
        rec_data.controlData.yaw;

        static_cast<unsigned char>(rec_data.cSMode);
    }
public:
    ToPult send_data;
    FromPult rec_data;
    UdpProtocol <FromPult, ToPult> *udpProtocol;

    bool bindState(){return udpProtocol->bindState();}
};
} //namespace ControlSystem

namespace Pult {
class PC_Protocol: public QObject, public MetaUdpProtocol {
    Q_OBJECT
public:
explicit PC_Protocol(const QString & config = "protocols.conf",
                       const QString & name = "pult", QObject *parent = 0)
{
    udpProtocol = new UdpProtocol <ToPult, FromPult> (config, name, parent);
    connect(timer,SIGNAL(timeout()),SLOT(sendData()));
    connect(udpProtocol->getReceiveSocket(),SIGNAL(readyRead()),SLOT(receiveData()));
    set_ip_receiver(udpProtocol->ip_receiver());
    set_ip_sender (udpProtocol->ip_sender());
    set_port_receiver(udpProtocol->port_receiver());
    set_port_sender (udpProtocol->port_sender());
}
signals:
    void dataReceived();
public slots:
    //запуск обмена
    void startExchange(){
        timer->start(1000/udpProtocol->getFrequency());
    }
    //остановить обмен
    void stopExhange(){
        timer->stop();
    }
    void sendData(){
 //       qDebug()<<"send data";
        udpProtocol->send_data = send_data;
        udpProtocol->sendData();
    }

    void receiveData(){
        udpProtocol->receiveData();
        rec_data = udpProtocol->rec_data;
        emit dataReceived();
//        static_cast<unsigned char>(rec_data.cSMode);
    }
public:
    ToPult  rec_data;
    FromPult send_data;
    UdpProtocol <ToPult, FromPult> *udpProtocol;

    bool bindState(){return udpProtocol->bindState();}
};
} //namespace Pult


#endif // PC_PROTOCOL_H
