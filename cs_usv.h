#ifndef CS_USV_H
#define CS_USV_H

#include "protocol_BW_AH127C/AH127Cprotocol.h"
#include "protocol_gans_ZIMA/protocolzima.h"
#include "protocol_pult/pc_protocol.h"
#include "VMA_control/vma_control.h"
#include "protocol_echolot/PA500.h"
#include "protocol_GPS/nmea0183.h"
#include "diagnostic_board/diagnostic_board.h"
#include <QThread>
#include <QSettings>
#include "math.h"
#include <qmath.h>
#include <QTime>
#include <QDebug>

const QString ConfigFile = "protocols.conf";
const QString agent = "surface_agent";
const QString XI = "xi";
const QString KI = "ki";
const QString KI_MODEL = "ki_model";

class CS_USV : public QObject
{
    Q_OBJECT
public:
    CS_USV(QObject * parent = nullptr);
    void parseJsonFile(QString filePath);
    void start(int dt){
        timer.start(dt);
    }

    double limit (double value, double limit){
        if(fabs(value)>limit) return (limit*sgn(value));
        else return value;
    }

    template <typename T> int sgn(T val) {
        return (T(0) < val) - (val < T(0));
    }

    float saturation(float input,  float max, float min);
    double yawErrorCalculation(float yawDesiredDeg, float yawCurrentDeg);
    int sign(double input);

public slots:
    void tick();
    void handleDepthData(float d);

protected:
    void processDesiredValuesAutomatiz(double inputFromRUD, double &output, double &prev_output, double scaleK,
                                       bool flagLimit = false, double maxValue=180, double dt=0.01);
    void integrate(double &input, double &output, double &prevOutput, double dt);

    void readDataFromPult();
    void setModellingFlag(bool flag);
    void alternative_yaw_calculation(float dt);
    void resetValues();
    void readDataFromSensors();

    void regulators();
    void resetYawChannel();
    void resetRollChannel();
    void resetPitchChannel();
    void controlYaw(double dt);
    void controlPitch(double dt);
    void controlRoll(double dt);
    void BFS_DRK(double Upsi, double Uteta, double Ugamma, double Ux, double Uy, double Uz);

    void writeDataToVMA();
    void writeDataToPult();
    void aperiodicFilter(double &input, double &output, double &prevOutput, double K, double T, double dt);
    void JSON_init();

    AH127Cprotocol *AH127C = nullptr;
    ProtocolZIMA *GANS = nullptr;
    ControlSystem::PC_Protocol *auvProtocol = nullptr;
    VMA_control *vmaProtocol = nullptr;
    PA500 *echolot = nullptr;
    NMEA::NMEA0183 *GPS = nullptr;

    Diagnostic_board *diagnostic = nullptr;

    QTimer timer;
    QTime timeRegulator;
    QTime timeYaw;

    quint8 modellingFlag = 1;
    quint8 flag_of_mode = 100;
    quint8 contour_closure_yaw = 0;
    quint8 contour_closure_pitch = 0;
    quint8 contour_closure_roll = 0;
    quint8 contour_closure_march = 0;
    quint8 contour_closure_lag = 0;
    quint8 contour_closure_depth = 0;
    qint8 flag_switch_mode_1 = true;
    qint8 flag_switch_mode_2 = false;
    qint8 flag_switch_mode_3 = false;
    double drewYaw = 0;
    double drewYawAuto = 0;
    bool flagYawInit = false;
    bool flagYawAuto = false;
    double A[3][3];  //матрица перехода
    double I[3];   //Ix, Iy, Iz
};

#endif // CS_USV_H
