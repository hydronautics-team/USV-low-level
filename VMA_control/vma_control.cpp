#include "vma_control.h"

#define PIN_BASE 300
#define MAX_PWM 4096
#define HERTZ 50
#define ADDR 0x40

VMA_control::VMA_control(QObject* parent)
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
//    qDebug() << "send VMA";

}
void VMA_control::setvalues(double data_1, double data_2, double data_3, double data_4) {
    static float y = 0;
    static int z =0;
//    if (z<4)
//    {




//        if (y<0.1) y +=0.001;
//        else {y = 0;
//            z+=1;
//        }
//    }
//    else {y = 0;
//    }
    vmaVector[0] = calcTicks((1.5 + data_1 + y), HERTZ);
    vmaVector[1] = calcTicks((1.5 + data_2 + y), HERTZ);
    vmaVector[2] = calcTicks((1.5 + data_3 + y), HERTZ);
    vmaVector[3] = calcTicks((1.5 + data_4 + y), HERTZ);
    VMA_control::senddata();
}
