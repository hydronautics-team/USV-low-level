#include "vma_control.h"

#define PIN_BASE 300
#define MAX_PWM 4096
#define HERTZ 50
#define ADDR 0x40

VMA_control::VMA_control()
{

}
int calcTicks(float impulseMs, int hertz)
{
    float cycleMs = 1000.0f / hertz;
    return (int)(MAX_PWM * impulseMs / cycleMs + 0.5f);
}

void VMA_control::start() {

    fd = pca9685Setup(PIN_BASE, ADDR, HERTZ);
    if (fd < 0) {
        qDebug() << "микросхема не подключена";
    } else {
        qDebug() << "микросхема подключена";
    }

}

void VMA_control::senddata(){
    pca9685PWMWrite(fd, 0, 0, vmaVector[0]);
    pca9685PWMWrite(fd, 1, 0, vmaVector[1]);
    pca9685PWMWrite(fd, 2, 0, vmaVector[2]);
    pca9685PWMWrite(fd, 3, 0, vmaVector[3]);
    qDebug() << "send VMA";
    qDebug() << vmaVector[0];
    qDebug() << vmaVector[1];

}
void VMA_control::setvalues(double data_1, double data_2, double data_3, double data_4) {
    vmaVector[0] = calcTicks(data_1, HERTZ);
    vmaVector[1] = calcTicks(data_2, HERTZ);
    vmaVector[2] = calcTicks(data_3, HERTZ);
    vmaVector[3] = calcTicks(data_4, HERTZ);
    VMA_control::senddata();
}
