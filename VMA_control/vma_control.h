#ifndef VMA_CONTROL_H
#define VMA_CONTROL_H

#include "pca9685/pca9685.h"
#include <QDebug>
#include <QByteArray>

class VMA_control
{
public:
    VMA_control();
    void start();
    void setvalues(double data_1,double data_2, double data_3, double data_4 );

private:
    int fd = 0;
    void senddata();
    qint16 vmaVector[4];

};

#endif // VMA_CONTROL_H
