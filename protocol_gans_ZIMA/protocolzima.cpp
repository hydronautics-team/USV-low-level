#include "protocolzima.h"

/// Baudrate: 9600 bit/s
/// Data bits: 8
/// Stop bits: 1
/// Parity: No
/// Hardware flow control: No

ProtocolZIMA::ProtocolZIMA(QString portName, int baudRate,
                           QObject *parent): QObject(parent)
{
    zima.setBaudRate(baudRate);
    zima.setPortName(portName);
    zima.setDataBits(QSerialPort::Data8);
    zima.setStopBits(QSerialPort::OneStop);
    zima.setParity(QSerialPort::NoParity);
    zima.setFlowControl(QSerialPort::NoFlowControl);
    zima.open(QIODevice::ReadWrite);

    if (zima.isOpen()){
        qDebug()<<" port was opened";
    }
    else {
        qDebug()<<" error open port "<< zima.errorString();
    }

    connect(&zima, &QSerialPort::readyRead, this, [this]()
    {
        zima_buffer.append(zima.readAll());
        if (test_messege) qDebug() << "zima_buffer " << zima_buffer;
        int size = zima_buffer.size();
        if (size !=0)
        {
            char end = zima_buffer.count(10); //проверка на последний символ, пока не разобралась какой, но поверим
            if (end > 0)
            {
                parseBuffer();
            }
        }
    });
    timer_send = new QTimer(this);
    connect(timer_send, &QTimer::timeout, this, &ProtocolZIMA::sendSoundSlot);
    timer_send->start(1000);

}


void ProtocolZIMA::parseBuffer()
{
    if (test_messege) qDebug() << "parseBuffer";
    int count = zima_buffer.count(36); //считаем сколько раз встретилось начало сообщения символ - $
    if (test_messege) qDebug() << count;
    while (count != 0)
    {
        count = count - 1;
        qint8 index = zima_buffer.indexOf(36); //поиск индекса $
        qint8 crc_in = zima_buffer.indexOf(42); //поиск индекса *
        uint end = crc_in + 5; //последний символ посылки
        uint size_msg = zima_buffer.size();
        if (end > size_msg) return;
        if ((index == -1) or (crc_in == -1))
        {
            // Не найдено сообщение
            qDebug() << "Нет сообщения в буфере";
            return;
        }
        else
        {

            QByteArray title = zima_buffer.mid (index+1, 5);
            findTitle(index, crc_in, end, title);
        }
    }

}

Title stringToTitle(const QByteArray tit) {
    if (tit == "PZMAE") return PZMAE;
    if (tit == "PZMAF") return PZMAF;
    if (tit == "PZMAG") return PZMAG;
    if (tit == "PZMA0") return PZMA0;
    return UNKNOWN;
}

void ProtocolZIMA::findTitle(qint8 index, qint8 crc_in, uint end, QByteArray title)
{
    Title titleEnum = stringToTitle(title);
    int count;
    QByteArray msg;

    switch (titleEnum) {
    case PZMAE:
        msg = zima_buffer.mid(index+1, crc_in-1);
        if (crc_real(crc_in) == crc(msg))
        {
            parsePZMAE(msg);
            //                updateData(uwave);
            zima_buffer.remove(0, end);
        }
        else
        {
            if (test_messege) qDebug() << "PZMAE crc не верна";
            count = zima_buffer.count(42); //считаем сколько раз встретилось начало сообщения символ - $
            if (count != 0)
                zima_buffer.remove(0, end);
        }
        break;
    case PZMAF:
        msg = zima_buffer.mid(index+1, crc_in-1);
        if (crc_real(crc_in) == crc(msg))
        {
            parsePZMAF(msg);
            //                updateData(uwave);
            zima_buffer.remove(0, end);
        }
        else
        {
            if (test_messege) qDebug() << "PZMAF crc не верна";
            count = zima_buffer.count(42); //считаем сколько раз встретилось начало сообщения символ - $
            if (count != 0)
                zima_buffer.remove(0, end);
        }
        break;
    case PZMAG:
        msg = zima_buffer.mid(index+1, crc_in-1);
        if (crc_real(crc_in) == crc(msg))
        {
            parsePZMAG(msg);
            //                updateData(uwave);
            zima_buffer.remove(0, end);
        }
        else
        {
            if (test_messege) qDebug() << "PZMAG crc не верна";
            count = zima_buffer.count(42); //считаем сколько раз встретилось начало сообщения символ - $
            if (count != 0)
                zima_buffer.remove(0, end);
        }
        break;
    case PZMA0:
        msg = zima_buffer.mid(index+1, crc_in-1);
        if (crc_real(crc_in) == crc(msg))
        {
            parsePZMA0(msg);
            //                updateData(uwave);
            zima_buffer.remove(0, end);
        }
        else
        {
            if (test_messege) qDebug() << "PZMA0 crc не верна";
            count = zima_buffer.count(42); //считаем сколько раз встретилось начало сообщения символ - $
            if (count != 0)
                zima_buffer.remove(0, end);
        }
        break;
    case UNKNOWN:
        if (test_messege) qDebug() << "не известное сообщение";
        count = zima_buffer.count(42); //считаем сколько раз встретилось начало сообщения символ - $
        if (count != 0)
            zima_buffer.remove(0, end);
        break;
    default:
        if (test_messege) qDebug() << "не известное сообщение";
        int count = zima_buffer.count(42); //считаем сколько раз встретилось начало сообщения символ - $
        if (count != 0)
            zima_buffer.remove(0, end);
        break;

    }
}

int ProtocolZIMA::crc_real(qint8 crc_in)
{
    int crc_real = 0;
    if ((int (zima_buffer[crc_in+1]))>57)
        crc_real += ((int (zima_buffer[crc_in+1]))-'0'-7)*16;
    else crc_real += ((int (zima_buffer[crc_in+1]))-'0')*16;
    if ((int (zima_buffer[crc_in+2]))>57)
        crc_real += ((int (zima_buffer[crc_in+2]))-'0') -7;
    else crc_real += ((int (zima_buffer[crc_in+2]))-'0');
    return crc_real;
}

int ProtocolZIMA::crc(QByteArray tmp)
{
    int crc_ = 0;
    for (int i = 0; i <= (tmp.size()-1); i++)
    {
        crc_^=tmp[i];
    }
    return crc_;
}

void ProtocolZIMA::parsePZMAE(QByteArray msg)
{
    if (test_messege) qDebug() << msg;
    int index =msg.indexOf(44);//ищем первую запятую, перед ней идет не интересный заголовок
    msg.remove(0, index+1); // удаляем заголовок
    index =msg.indexOf(44);//ищем запятую
    data.pzmae.TargetID = atoi(msg.mid(0, index));
    if (test_messege) qDebug() << "pzmae.TargetID: "<< data.pzmae.TargetID;
    msg.remove(0, index+1);
    index =msg.indexOf(44);//ищем запятую
    data.pzmae.RequestID = atoi(msg.mid(0, index));
    if (test_messege) qDebug() << "pzmae.RequestID: "<< data.pzmae.RequestID;
    msg.remove(0, index+1);
    index =msg.indexOf(44);//ищем запятую
    data.pzmae.dFlag     = atoi(msg.mid(0, index));
    if (test_messege) qDebug() << "pzmae.dFlag: "<< data.pzmae.dFlag;
    msg.remove(0, index+1);
    index =msg.indexOf(44);//ищем запятую
    data.pzmae.Azimuth   = atof(msg.mid(0, index));
    if (test_messege) qDebug() << "pzmae.Azimuth: "<< data.pzmae.Azimuth;
    msg.remove(0, index+1);
    index =msg.indexOf(44);//ищем запятую
    data.pzmae.Distance  = atof(msg.mid(0, index));
    if (test_messege) qDebug() << "pzmae.Distance: "<< data.pzmae.Distance;
    msg.remove(0, index+1);
    index =msg.indexOf(44);//ищем запятую
    data.pzmae.DataValue = atof(msg.mid(0, index));
    if (test_messege) qDebug() << "pzmae.DataValue: "<< data.pzmae.DataValue;
    msg.remove(0, index+1);
    index =msg.indexOf(44);//ищем запятую
    data.pzmae.SNR       = atof(msg.mid(0, index));
    if (test_messege) qDebug() << "pzmae.SNR: "<< data.pzmae.SNR;
    msg.remove(0, index+1);
    index =msg.indexOf(44);//ищем запятую
    data.pzmae.DPL       = atof(msg.mid(0, index));
    if (test_messege) qDebug() << "pzmae.DPL: "<< data.pzmae.DPL;
    msg.remove(0, index+1);
}

void ProtocolZIMA::parsePZMAF(QByteArray msg)
{
    if (test_messege) qDebug() << msg;
    int index =msg.indexOf(44);//ищем первую запятую, перед ней идет не интересный заголовок
    msg.remove(0, index+1); // удаляем заголовок
    index =msg.indexOf(44);//ищем запятую
    data.pzmaf.Temperature = atof(msg.mid(0, index));
    if (test_messege) qDebug() << "pzmaf.Temperature: "<< data.pzmaf.Temperature;
    msg.remove(0, index+1);
    index =msg.indexOf(44);//ищем запятую
    data.pzmaf.Depth = atof(msg.mid(0, index));
    if (test_messege) qDebug() << "pzmaf.Depth: "<< data.pzmaf.Depth;
    msg.remove(0, index+1);
    index =msg.indexOf(44);//ищем запятую
    data.pzmaf.isAHRSEnabled = atof(msg.mid(0, index));
    if (test_messege) qDebug() << "pzmaf.isAHRSEnabled: "<< data.pzmaf.isAHRSEnabled;
    msg.remove(0, index+1);
    index =msg.indexOf(44);//ищем запятую
    data.pzmaf.TRX_State = atof(msg.mid(0, index));
    if (test_messege) qDebug() << "pzmaf.TRX_State: "<< data.pzmaf.TRX_State;
    msg.remove(0, index+1);
}

void ProtocolZIMA::parsePZMAG(QByteArray msg)
{
    if (test_messege) qDebug() << msg;
    int index =msg.indexOf(44);//ищем первую запятую, перед ней идет не интересный заголовок
    msg.remove(0, index+1); // удаляем заголовок
    index =msg.indexOf(44);//ищем запятую
    data.pzmag.Roll = atof(msg.mid(0, index));
    if (test_messege) qDebug() << "pzmag.Roll: "<< data.pzmag.Roll;
    msg.remove(0, index+1);
    index =msg.indexOf(44);//ищем запятую
    data.pzmag.Pitch = atof(msg.mid(0, index));
    if (test_messege) qDebug() << "pzmag.Pitch: "<< data.pzmag.Pitch;
    msg.remove(0, index+1);
}

void ProtocolZIMA::parsePZMA0(QByteArray msg)
{
    if (test_messege) qDebug() << msg;
    int index =msg.indexOf(44);//ищем первую запятую, перед ней идет не интересный заголовок
    msg.remove(0, index+1); // удаляем заголовок
    index =msg.indexOf(44);//ищем запятую
    data.pzma0.Error_code = atoi(msg.mid(0, index));
    if (test_messege) qDebug() << "pzma0.Error_code: "<< data.pzma0.Error_code;
    msg.remove(0, index+1);
}

void ProtocolZIMA::sendSoundSlot()
{
    if (test_messege) qDebug() << "TIMEOUT";
    if (zima.isOpen())
    {
        QByteArray bufArray = "$PZMAC,1,362*43\n\r";
        int size = bufArray.size();
        if (test_messege) qDebug() << "Проверочка на размер bufArray.size " << bufArray.size();
        char *PZMAC = bufArray.data();
        zima.write(PZMAC, size);
        zima.waitForBytesWritten();
    }
    else
    {
        qDebug() << "порт не открыт: " << zima.error();
        zima.close();
        zima.open(QIODevice::ReadWrite);
    }
}
