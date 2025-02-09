#include "cs_usv.h"
#include <QDebug>
#include "kx_pult/kx_protocol.h"
#include "kx_pult/qkx_coeffs.h"
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QString>

double X[2000][2];

CS_USV::CS_USV(QObject *parent)
{
    qDebug() << "start constract CS_USV";
    QString filePath = "configuration/settings.json";
    parseJsonFile(filePath); //чтение джейсона и создание объектов под все обмены

    //передача K
    Qkx_coeffs* kProtocol = new Qkx_coeffs(ConfigFile, KI);
    //передача X
    x_protocol* xProtocol = new x_protocol(ConfigFile, XI, X);

    //обмен с пультом управления
    auvProtocol = new ControlSystem::PC_Protocol(ConfigFile,"surface_agent");
    auvProtocol->startExchange();

    //обмен с ВМА
    vmaProtocol = new VMA_control();
    vmaProtocol->moveToThread(&vmaThread);
    QObject::connect(&vmaThread, &QThread::started, vmaProtocol, &VMA_control::start);
    vmaThread.start();

    //включаем ВМА установкой сигналов на ножки
    wiringPiSetup () ;
    pinMode (27, OUTPUT) ;
    digitalWrite (27, HIGH) ;

    X[91][0]=X[91][1]=0; //нулевые НУ для интегрирования угловой скорости и нахождения угла курса
    X[92][0]=X[92][1]=0; //нулевые НУ для интегрирования угловой скорости и нахождения угла дифферента
    X[609][0]=X[609][1]=0; //нулевые НУ для дифференцирования контура глубины

    // захват видеопотока
    QStringList arguments;
           arguments << "v4l2src" << "device=/dev/video0"
                     << "!image/jpeg,width=640,height=480,framerate=30/1"
                     << "!jpegparse"
                     << "!rtpjpegpay"
                     << "!udpsink" << "host=192.168.3.4" << "port=89000";

           process = new QProcess();
           process->setProgram("gst-launch-1.0");
           process->setArguments(arguments);
           process->start();

    logger = new Logger();
    connect(echolot, &PA500::sendDistance, logger, &Logger::log_Echo);
    connect(&timer, &QTimer::timeout, this, &CS_USV::tick);
    connect(gpsProt,&NMEA::NMEA0183::updateGPS, logger, &Logger::log_GPS);
    connect(GANS,&ProtocolZIMA::updateZima, logger, &Logger::log_Zima);
    connect(this, &CS_USV::updateSSP, logger, &Logger::log_SSP);

    //отключение связи с пультом управления по таймеру
    connect(auvProtocol, &ControlSystem::PC_Protocol::dataReceived, this, &CS_USV::exchangeUSV);
    connect(&timerReceived, &QTimer::timeout, this, &CS_USV::closeExchangeUSV);
    timerReceived.start(2000);

    timer.start(20);
    timeRegulator.start();
}

void CS_USV::parseJsonFile(QString filePath)
{
    QFile file(filePath);
    if (!file.exists()) {
        qDebug() << "No config file : " << filePath;
    } else {
        qDebug() << "Config file opened";
        file.open(QIODevice::ReadOnly);
        QJsonDocument doc = QJsonDocument().fromJson(file.readAll());
        QJsonObject confObject = doc.object();

        // Обработка данных от БСО и создание объекта обмена
        QJsonObject bsoObj = confObject.value("BW-AH127C").toObject();
        AH127C = new AH127Cprotocol(bsoObj.value("device").toString());
        connect_BSO = bsoObj.value("state").toString();

        qDebug() << "end AH127C";

        // Обработка данных от платы диагностики и создание объекта обмена
        QJsonObject diagnObj = confObject.value("Diagnostic").toObject();
        diagnostic = new Diagnostic_board(diagnObj.value("device").toString(), diagnObj.value("speed").toInt());

        // Обработка данных от ГАНСА и создание объекта обмена
        QJsonObject gansObj = confObject.value("Hydroacoustics").toObject();
        GANS = new ProtocolZIMA(gansObj.value("device").toString());
        ssp = new CoordSSP();
        connect(GANS, &ProtocolZIMA::updateZima, logger, &Logger::log_Zima);

        // Обработка данных от GPS и создание объекта обмена
        QJsonObject gpsObj = confObject.value("GPS").toObject();
        gpsProt = new NMEA::NMEA0183 (gpsObj.value("device").toString(), 115200);

        // Обработка данных от эхолота и создание объекта обмена
        QJsonObject echObj = confObject.value("Echolot").toObject();
        echolot = new PA500(echObj.value("device").toString(), 9600);
    }
    qDebug() << "end parseJsonFile";
}

float CS_USV::saturation(float input, float max, float min)
{
    if (input >= max) return max;
    else if (input <= min) return min;
    else return input;
}

double CS_USV::yawErrorCalculation(float yawDesiredDeg, float yawCurrentDeg)
{
    double Krad = M_PI/180.0;
    double Kdeg = 180/M_PI;
    double desiredPsi = yawDesiredDeg*Krad;
    double currentPsi = yawCurrentDeg*Krad;
    double l0 = cos(desiredPsi/2)*cos(currentPsi/2)+sin(desiredPsi/2)*sin(currentPsi/2);
    double l2 = cos(desiredPsi/2)*sin(currentPsi/2)-cos(currentPsi/2)*sin(desiredPsi/2);
    if (fabs(l0)>1) l0=sign(l0)*1;
    if (l0<0) l0*=-1;
    else l2 *=-1;
    double temp = 2*acos(l0);
    double temp_deg = 2*acos(l0)*Kdeg;
    double temp_deg_sign = 2*acos(l0)*sign(l2)*Kdeg;
    return temp_deg_sign;
}

int CS_USV::sign(double input)
{
    if (input >= 0) return 1;
    else return -1;
}

void CS_USV::tick()
{
    readDataFromPult();
    readDataFromSensors();
    regulators();
    BFS_DRK(X[101][0], X[102][0], X[103][0] , X[104][0], X[105][0], X[106][0]);
    writeDataToVMA();
    writeDataToPult();
    X[701][0] = goal_point.x_point;
    X[702][0] = goal_point.y_point;

    X[703][0] = current_point.x_point;
    X[704][0] = current_point.y_point;
}

void CS_USV::exchangeUSV()
{
    timerReceived.start(2000);
    if (!timer.isActive())
    {
        timer.start(10);
    }
}

void CS_USV::closeExchangeUSV()
{
    timer.stop();
    resetValues();
}

void CS_USV::processDesiredValuesAutomatiz(double inputFromRUD, double &output, double &prev_output, double scaleK, bool flagLimit, double maxValue, double dt)
{
    double inputScaled = inputFromRUD * scaleK;
    integrate(inputScaled,output,prev_output,dt);
    if (flagLimit){
        saturation(output,maxValue,-maxValue);
    }
}

void CS_USV::integrate(double &input, double &output, double &prevOutput, double dt)
{
    output = prevOutput + dt * input;
    prevOutput = output;
}


void CS_USV::resetValues()
{
   vmaProtocol->setvalues(1.5, 1.5, 1.5, 1.5);
}

void CS_USV::readDataFromPult()
{
    X[51][0] = auvProtocol->rec_data.controlData.yaw;
    X[52][0] = auvProtocol->rec_data.controlData.pitch;
    X[53][0] = auvProtocol->rec_data.controlData.roll;
    X[54][0] = auvProtocol->rec_data.controlData.march;
    X[55][0] = auvProtocol->rec_data.controlData.lag;
    X[56][0] = auvProtocol->rec_data.controlData.depth;

    if (auvProtocol->rec_data.cSMode == e_CSMode::MODE_AUTOMATED) qDebug() << "Я в АВТОМАТИЗИРОВАННОМ!";
    if (auvProtocol->rec_data.cSMode == e_CSMode::MODE_MANUAL) qDebug() << "Я в РУЧНОМ!";
    if (auvProtocol->rec_data.cSMode == e_CSMode::MODE_AUTOMATIC) qDebug() << "Я в АВТОМАТИЧЕСКОМ";

//    qDebug() << "auvProtocol->rec_data.reper.x_point" << QString::number(auvProtocol->rec_data.reper.x_point, 'f', 6);
//    qDebug() << "auvProtocol->rec_data.reper.y_point" <<  QString::number(auvProtocol->rec_data.reper.y_point, 'f', 6);

    if (auvProtocol->rec_data.modeAUV_selection == 1) setModellingFlag(true);
        else setModellingFlag(false);

    if(auvProtocol->rec_data.pMode == power_Mode::MODE_2)
    {
        digitalWrite (27, LOW) ;
    }
    if(auvProtocol->rec_data.pMode == power_Mode::MODE_3)
    {
        digitalWrite (27, HIGH) ;
    }

    if (auvProtocol->rec_data.reper.x_point !=0 && flag_reper_on == 0)
    {
        flag_reper_on = 1;
        GPSPoint reperData;
        reperData.latitude = auvProtocol->rec_data.reper.x_point ;
        reperData.longitude = auvProtocol->rec_data.reper.y_point;
        ssp->setReferencePoint(reperData);
    }

}

void CS_USV::setModellingFlag(bool flag)
{
    if (modellingFlag!=flag) {
        if (modellingFlag == false) resetValues();
        modellingFlag = flag;
    }
}

void CS_USV::alternative_yaw_calculation(float dt)
{
    if (connect_BSO == "connect") {
        X[170][0] = X[70][0] + K[70]; //Mx с учетом коррекции
        X[171][0] = X[71][0] + K[71]; //My с учетом коррекции
        X[172][0] = X[72][0] + sin(0.5*X[63][0]/57.3)*K[72]; //Mz с учетом коррекции

        double teta = X[62][0]*M_PI/180;
        double gamma = X[63][0]*M_PI/180;
        X[176][0] = teta;
        X[177][0] = gamma;
        A[0][0] = cos(teta); A[0][1] = sin(teta)*sin(gamma); A[0][2] = -sin(teta)*cos(gamma);
        A[1][0] = 0; A[1][1] = cos(gamma); A[1][2] = sin(gamma);
        A[2][0] = sin(teta); A[2][1] = -sin(gamma)*cos(teta); A[2][2] = cos(teta)*cos(gamma);

        X[300][0] = I[0] = A[0][0]*X[170][0] + A[0][1]*X[171][0] + A[0][2]*X[172][0];
        X[400][0] = I[1] = A[1][0]*X[170][0] + A[1][1]*X[171][0] + A[1][2]*X[172][0];
        X[500][0] = I[2] = A[2][0]*X[170][0] + A[2][1]*X[171][0] + A[2][2]*X[172][0];

        X[174][0] = I[0];
        X[175][0] = I[1];
        X[178][0] = atan2(-I[1],-I[0])*57.3;

        X[79][0] = -1/cos(X[62][0]/57.3)*(-X[69][0]*cos(X[63][0]/57.3)-X[68][0]*sin(X[63][0]/57.3));

        if (!flagYawInit) {
           flagYawInit = true;
           X[91][0] = X[91][1]= X[178][0] + K[178] + K[179]; //K[179] - магнитное склонение
           drewYaw = X[69][0];
        }
        integrate(X[79][0],X[91][0],X[91][1],dt); //интегрируем показание Z_rate для нахождения текущего угла курса

    } else X[91][0] = 0;

//    X[91][0] = 360+60 - gpsProt->gps->psat.yaw;
}

void CS_USV::readDataFromSensors()
{
    X[61][0] = AH127C->data.yaw;
    X[62][0] = AH127C->data.pitch;
    X[63][0] = AH127C->data.roll;
//    qDebug() << "AH127C->data.yaw" << X[61][0];
//    qDebug() << "AH127C->data.pitch" << X[62][0];
//    qDebug() << "AH127C->data.roll" << X[63][0];

    X[64][0] = AH127C->data.X_accel;
    X[65][0] = AH127C->data.Y_accel;
    X[66][0] = AH127C->data.Z_accel;

    X[67][0] = AH127C->data.X_rate;
    X[68][0] = AH127C->data.Y_rate;
    X[69][0] = AH127C->data.Z_rate - drewYaw;

    X[70][0] = AH127C->data.X_magn;
    X[71][0] = AH127C->data.Y_magn;
    X[72][0] = AH127C->data.Z_magn;

    X[73][0] = AH127C->data.first_qvat;
    X[74][0] = AH127C->data.second_qvat;
    X[75][0] = AH127C->data.third_qvat;
    X[76][0] = AH127C->data.four_qvat;

    //чтение данных от ГАНС
    X[41][0] = GANS->data.pzmae.Azimuth;
    X[42][0] = GANS->data.pzmae.Distance;
    X[43][0] = GANS->data.pzmae.DataValue;
    X[44][0] = GANS->data.pzmaf.Temperature;
    X[45][0] = GANS->data.pzmaf.Depth;
    X[46][0] = GANS->data.pzmag.Roll;
    X[47][0] = GANS->data.pzmag.Pitch;

//  чтение данных от Транзаса (вынесена только часть)
    X[48][0] = gpsProt->gps->gga.latitude;
    X[49][0] = gpsProt->gps->gga.longitude;
    X[191][0] = gpsProt->gps->psat.yaw;

//    if ( gpsProt->gps->gga.latitude !=0 && flag_reper_on == 0)
//    {
//        qDebug() << "установка репера";
//        flag_reper_on = 1;
//        GPSPoint reperData;
//        reperData.latitude     =gpsProt->gps->gga.latitude;
//        reperData.longitude     = gpsProt->gps->gga.longitude;
//        ssp->setReferencePoint(reperData);
//    }

    GPSPoint gpsData;
    gpsData.latitude =gpsProt->gps->gga.latitude;
    gpsData.longitude = gpsProt->gps->gga.longitude;
    ssp->getLocalCoordinates(gpsData, current_point.x_point, current_point.y_point);
    qDebug() << "current_point.x_point" << current_point.x_point;
    qDebug() << "current_point.y_point" << current_point.y_point;
}

void CS_USV::regulators()
{
    float dt = timeRegulator.elapsed()*0.001;   //реальный временной шаг цикла
    timeRegulator.start();
    alternative_yaw_calculation(dt);

    if (auvProtocol->rec_data.cSMode == e_CSMode::MODE_MANUAL) { //САУ разомкнута
        if (flag_switch_mode_1 == false) {
            X[5][0] = X[5][1] = 0;
            flag_switch_mode_1 = true;
            flag_switch_mode_2 = false;
            flag_switch_mode_3 = false;
        }
        flag_of_mode = 0;
        X[101][0] = K[101]*X[51][0]; //Upsi
        X[102][0] = K[102]*X[52][0]; //Uteta
        X[103][0] = K[103]*X[53][0]; //Ugamma
        X[104][0] = K[104]*X[54][0]; //Ux
        X[105][0] = K[105]*X[55][0]; //Uy
        X[106][0] = K[106]*X[56][0]; //Uz
        resetYawChannel();
        resetRollChannel();
        resetPitchChannel();

    } else if (auvProtocol->rec_data.cSMode == e_CSMode::MODE_AUTOMATED) { //САУ в автоматизированном режиме
        flag_of_mode = 1;
        X[104][0] = K[104]*X[54][0]; //Ux  - марш
        X[105][0] = K[105]*X[55][0]; //Uy
        X[106][0] = K[106]*X[56][0]; //Uz

        if (auvProtocol->rec_data.controlContoursFlags.yaw > 0) { //замкнут курс
            if (flag_switch_mode_2 == false) {
                X[5][1]=X[91][0];
                X[5][0] = 0;
                flag_switch_mode_2 = true;
                flag_switch_mode_1 = false;
                flag_switch_mode_3 = false;
            }
            contour_closure_yaw = 1;
            processDesiredValuesAutomatiz(X[51][0],X[5][0],X[5][1],K[2]); //пересчет рукоятки в автоматизированном режиме
            controlYaw(dt); //контур курса
        } else {
            X[117][0] = K[101]*X[51][0]; //Upsi
            resetYawChannel();
            X[118][0] = saturation(X[117][0],K[116],-K[116]);
            X[101][0] = X[118][0]*K[100];
        }
        controlRoll(dt); //контур крена
//      controlPitch(dt); //контур дифферента
    } else if (auvProtocol->rec_data.cSMode == e_CSMode::MODE_AUTOMATIC) { //САУ в автоматизированном режиме
        if  (auvProtocol->rec_data.missionControl == mission_Control::MODE_COMPLETE) {
            X[101][0] = X[102][0] = X[103][0] = X[104][0] = X[105][0] = X[106][0] = 0;
            auvProtocol->send_data.missionStatus = mission_Status::MODE_PERFOMED;
        } else {
            automated_motion(dt);
        }
    }
}

void CS_USV::calculate_yaw_for_go_to_point(float delta_x, float delta_y) {
    // пересчет в СК точки-цели
    if (delta_x < 0) {  // левая полуплоскость относительно целевой точки
        if (delta_y <= 0) {
            X[5][0] = atan(abs(delta_y/delta_x))*(180/M_PI); // 3 четверть
            qDebug() << "X[5][0] =  atan(abs(delta_y/delta_x))*(180/M_PI)!!!!!!!!!!!!";
        }
        if ((delta_y > 0)) {
            X[5][0] = - atan(abs(delta_y/delta_x))*(180/M_PI); // 2 четверть
            qDebug() << "X[5][0] = - atan(abs(delta_y/delta_x))*(180/M_PI)!!!!!!!!!!!!";
        }
    }
    if (delta_x >= 0) {  // правая полуплоскость относительно целевой точки
        if (delta_y > 0) {
            X[5][0] = -180 + atan(abs(delta_y/delta_x))*(180/M_PI); // 1 четверть
            qDebug() << "X[5][0] = -180 + atan(abs(delta_y/delta_x))*(180/M_PI)!!!!!!!!!!!!";
        }
        if (delta_y <= 0) {
            X[5][0] = 180 - atan(abs(delta_y/delta_x))*(180/M_PI); // 4 четверть
            qDebug() << "180 - atan(abs(delta_y/delta_x))*(180/M_PI)!!!!!!!!!!!!";
        }
    }
}

void CS_USV::automated_motion(double dt)
{
//МИССИЯ ВЫХОДА В ТОЧКУ
    if ((auvProtocol->rec_data.mission == mission_List::MOVE_TO_POINT) && (auvProtocol->rec_data.missionControl == mission_Control::MODE_START)) {
        qDebug() << "выход в точку";
        auvProtocol->send_data.missionList = mission_List::MOVE_TO_POINT;
        auvProtocol->send_data.missionStatus = mission_Status::MODE_RUNNING;

        GPSPoint gpsData; //перевод координат целевой точки относительно репера
        gpsData.latitude = auvProtocol->rec_data.mission_param.point_mission.x_point;
        gpsData.longitude = auvProtocol->rec_data.mission_param.point_mission.y_point;
        ssp->getLocalCoordinates(gpsData, goal_point.x_point, goal_point.y_point);

        qDebug() << "goal.x" << goal_point.x_point;
        qDebug() << "goal.y" << goal_point.y_point;

        X[912][0] = goal_point.x_point;
        X[913][0] = goal_point.y_point;

        X[914][0] = current_point.x_point;
        X[915][0] = current_point.y_point;

        float x_v_SK_sv_x_y_goal = current_point.x_point - goal_point.x_point; //координаты аппаратов, в случае СК с началом координат в точке-цели
        float y_v_SK_sv_x_y_goal = current_point.y_point - goal_point.y_point;

        qDebug() << "x_v_SK_sv_x_y_goal "  << x_v_SK_sv_x_y_goal;
        qDebug() << "y_v_SK_sv_x_y_goal "  << y_v_SK_sv_x_y_goal;

        qDebug() << "Расстояние между целью и аппаратом" << sqrt(pow(x_v_SK_sv_x_y_goal, 2) + pow(y_v_SK_sv_x_y_goal, 2));
        // проверка расстояния до целевой точки, если < утроенного радиуса, то размыкаю контура

        if (sqrt(pow(x_v_SK_sv_x_y_goal, 2) + pow(y_v_SK_sv_x_y_goal, 2)) < 3 * (auvProtocol->rec_data.mission_param.radius)) {
            X[101][0] = X[102][0] = X[103][0] = X[104][0] = X[105][0] = X[106][0] = 0;
            contour_closure_yaw = 0;
            auvProtocol->send_data.missionStatus = mission_Status::MODE_PERFOMED;
        } else {
            calculate_yaw_for_go_to_point(x_v_SK_sv_x_y_goal, y_v_SK_sv_x_y_goal);
            X[104][0] = K[107];
            qDebug() << "X[5][0]" << X[5][0];

            if (flag_switch_mode_3 == false) {
                X[59][0] = X[58][0]= X[91][0];
                flag_switch_mode_3 = true;
                flag_switch_mode_1 = false;
                flag_switch_mode_2 = false;
            }
            controlYaw(dt); //контур курса
        }

 //МИССИЯ УДЕРЖАНИЯ ПОЛОЖЕНИЯ
    } else if (auvProtocol->rec_data.mission == mission_List::KEEP_POS && auvProtocol->rec_data.missionControl == mission_Control::MODE_START) {
        auvProtocol->send_data.missionList = mission_List::KEEP_POS;
        auvProtocol->send_data.missionStatus = mission_Status::MODE_RUNNING;
        if (flag_keep_mode == 0) {
            double point_for_keeping_x = current_point.x_point;
            double point_for_keeping_y = current_point.y_point;
            X[5][0] = X[91][0];
            flag_keep_mode = 1;
        }
        controlYaw(dt); // стабилизация курса
        double dx = current_point.x_point - point_for_keeping_x;
        double dy = current_point.y_point - point_for_keeping_y;

        if (sqrt(dx * dx + dy * dy) > 3) {
            X[104][0] = 0;
            X[104][0] = K[107]/5 * dx;
//           X[105][0] = K_hold * dy;
        } else {
            X[104][0] = 0;
        }

//МИССИЯ ДВИЖЕНИЯ ПО ОКРУЖНОСТИ
    } else if ((auvProtocol->rec_data.mission == mission_List::MOVE_TO_POINT) && (auvProtocol->rec_data.missionControl == mission_Control::MODE_START)) {
        auvProtocol->send_data.missionList = mission_List::MOVE_CIRCLE;
        auvProtocol->send_data.missionStatus = mission_Status::MODE_RUNNING;
        QVector<QPointF> circlePoints;
        int additionalPoints = static_cast<int>(auvProtocol->rec_data.mission_param.radius / 5) * 4; // Увеличение точек пропорционально радиусу
        int n = 8 + additionalPoints;  // Общее количество точек
        for (int i = 0; i < n; ++i) {
            float theta = 2 * M_PI * i / n; // угол в радианах
            float x = auvProtocol->rec_data.mission_param.point_mission.x_point + auvProtocol->rec_data.mission_param.radius * cos(theta);
            float y = auvProtocol->rec_data.mission_param.point_mission.y_point + auvProtocol->rec_data.mission_param.radius * sin(theta);
            circlePoints.append(QPointF(x, y));
        }
        for (const auto& point_circle : circlePoints) {
            float x_v_SK_sv_x_y_goal = current_point.x_point - point_circle.x(); //координаты аппарата, в случае СК с началом координат в точке-цели
            float y_v_SK_sv_x_y_goal = current_point.y_point - point_circle.y();

            // проверка расстояния до целевой точки, если < радиуса, то перехожу к следующей точке

            while (sqrt(pow(x_v_SK_sv_x_y_goal, 2) + pow(y_v_SK_sv_x_y_goal, 2)) > auvProtocol->rec_data.mission_param.radius)  {
                calculate_yaw_for_go_to_point(x_v_SK_sv_x_y_goal, y_v_SK_sv_x_y_goal);
                X[104][0] = K[107];
                // X[105][0] = K[105]*X[55][0]; //Uy

                if (flag_switch_mode_3 == false) {
                    X[59][0] = X[58][0]= X[91][0];
                    flag_switch_mode_3 = true;
                    flag_switch_mode_1 = false;
                    flag_switch_mode_2 = false;
                    qDebug() << contour_closure_yaw <<"автоматический режим";
                }
                controlYaw(dt); //контур курса
            }
        }
        X[101][0] = X[102][0] = X[103][0] = X[104][0] = X[105][0] = X[106][0] = 0;
        contour_closure_yaw = 0;
        auvProtocol->send_data.missionStatus = mission_Status::MODE_PERFOMED;

    } else if ((auvProtocol->rec_data.mission == mission_List::MOVE_TACK) && (auvProtocol->rec_data.missionControl == mission_Control::MODE_START)) {
        auvProtocol->send_data.missionList = mission_List::MOVE_TACK;
        auvProtocol->send_data.missionStatus = mission_Status::MODE_RUNNING;
        // считываем первую точку, выходим в нее, как только вышли, присваиваем вторую точку первой и так дальше идти
    } else {
        X[101][0] = X[102][0] = X[103][0] = X[104][0] = X[105][0] = X[106][0] = 0;
        auvProtocol->send_data.missionList = mission_List::NO_MISSION;
        auvProtocol->send_data.missionStatus = mission_Status::MODE_IDLE;
    }
}

void CS_USV::resetYawChannel()
{
    X[5][0] = X[5][1] = 0;
    X[114][1] = X[114][0] = 0;
}

void CS_USV::resetRollChannel()
{
    X[800][0] = X[800][1] = 0;
    X[804][0] = X[804][1] = 0;
}

void CS_USV::resetPitchChannel()
{
    X[6][0] = X[6][1] = 0;
    X[314][0] = X[314][1] = 0;
}

void CS_USV::controlYaw(double dt)
{
    contour_closure_yaw = 1;
    X[111][0] = yawErrorCalculation(X[5][0],X[91][0]); //учет предела работы датчика, пересчет кратчайшего пути
    X[111][0] = X[5][0] - X[91][0];
    X[112][0] = X[111][0] * K[111];
    X[113][0] = X[112][0] * K[112];
    X[114][0] = X[114][1] + 0.5*(X[113][0] + X[113][1])*dt; //выходное значение интегратора без полок

    if (K[113] != 0){//значит заданы полки
        X[114][0] = saturation(X[114][0],K[113],K[114]); //выходное значение интегратора с полками
    }
    X[114][1] = X[114][0];
    X[113][1] = X[113][0];

    X[116][0] = X[114][0] + X[112][0];
    aperiodicFilter(X[79][0],X[401][0],X[401][1],K[402],K[403],dt);
    X[121][0] = X[401][0]*K[118];

    X[119][0] = X[51][0]*K[119];
    X[117][0] = X[116][0] - X[121][0] + X[119][0];

    X[118][0] = saturation(X[117][0],K[116],-K[116]);
    X[101][0] = X[118][0]*K[100];
}

void CS_USV::controlRoll(double dt)
{
    if (auvProtocol->rec_data.controlContoursFlags.roll>0) {
        contour_closure_roll = 1;

        processDesiredValuesAutomatiz(X[53][0],X[800][0],X[800][1],K[800]);
        X[801][0] = X[800][0] - X[63][0];

        X[802][0] = X[801][0] * K[801];
        X[803][0] = X[802][0] * K[802];
        integrate(X[803][0],X[804][0],X[804][1],dt);
        if (K[803]>0) {
            X[804][1] = saturation(X[804][0],K[804],-K[804]);
            X[804][0] = saturation(X[804][0],K[804],-K[804]);
        }
        X[814][0] = K[814] + sin((X[800][0]-K[816])/57.3) * K[815];

        X[806][0] = X[802][0] + X[804][0];
        X[805][0] = X[53][0] * K[805];

        X[807][0] = X[806][0] + X[805][0]+ X[814][0];

        //speed loop
        aperiodicFilter(X[67][0],X[808][0],X[808][1],1,K[808],dt);
        X[809][0] = X[808][0] * K[809];
        X[810][0] = X[807][0] - X[809][0] + K[817]*X[56][0];

    } else {
        X[810][0] = X[53][0] * K[103];
        resetRollChannel();
    }
    X[811][0] = saturation(X[810][0],K[810],-K[810]);
    X[103][0] = K[811] * X[811][0];
}

void CS_USV::BFS_DRK(double Upsi, double Uteta, double Ugamma, double Ux, double Uy, double Uz)
{
    X[110][0] = (K[10]*Ux + K[11]*Uy + K[12]*Uz + K[13]*Ugamma + K[14]*Uteta + K[15]*Upsi)*K[16];//U1
    X[120][0] = (K[20]*Ux + K[21]*Uy + K[22]*Uz + K[23]*Ugamma + K[24]*Uteta + K[25]*Upsi)*K[26];//U2
    X[130][0] = (K[30]*Ux + K[31]*Uy + K[32]*Uz + K[33]*Ugamma + K[34]*Uteta + K[35]*Upsi)*K[36];//U3
    X[140][0] = (K[40]*Ux + K[41]*Uy + K[42]*Uz + K[43]*Ugamma + K[44]*Uteta + K[45]*Upsi)*K[46];//U4
    X[211][0] = limit(X[110][0],K[200]);
    X[221][0] = limit(X[120][0],K[200]);
    X[231][0] = limit(X[130][0],K[200]);
    X[241][0] = limit(X[140][0],K[200]);
}

void CS_USV::writeDataToVMA()
{
         vmaProtocol->setvalues(X[211][0], X[221][0], X[231][0], X[241][0]);
}

void CS_USV::writeDataToPult()
{
    auvProtocol->send_data.headerSwap.senderID;
    auvProtocol->send_data.headerSwap.receiverID;
    auvProtocol->send_data.headerSwap.msgSize;

    auvProtocol->send_data.auvData.modeReal = flag_of_mode;
    auvProtocol->send_data.auvData.controlReal.yaw = contour_closure_yaw;
    auvProtocol->send_data.auvData.controlReal.pitch = contour_closure_pitch;
    auvProtocol->send_data.auvData.controlReal.roll = contour_closure_roll;
    auvProtocol->send_data.auvData.controlReal.march = contour_closure_march;
    auvProtocol->send_data.auvData.controlReal.depth = contour_closure_depth;
    auvProtocol->send_data.auvData.controlReal.lag = contour_closure_lag;
    auvProtocol->send_data.auvData.modeAUV_Real = modellingFlag;
    auvProtocol->send_data.auvData.ControlDataReal.yaw;
    auvProtocol->send_data.auvData.ControlDataReal.pitch;
    auvProtocol->send_data.auvData.ControlDataReal.roll;
    auvProtocol->send_data.auvData.ControlDataReal.march;
    auvProtocol->send_data.auvData.ControlDataReal.depth;
    auvProtocol->send_data.auvData.ControlDataReal.lag;
    auvProtocol->send_data.auvData.signalVMA_real;

    auvProtocol->send_data.dataAH127C.yaw = X[91][0];
    qDebug() << "X[91][0] "<< X[91][0];
//    qDebug() << "X[62][0] "<< X[62][0];
//    qDebug() << "X[63][0] "<< X[63][0];
    auvProtocol->send_data.dataAH127C.pitch = X[62][0];
    auvProtocol->send_data.dataAH127C.roll = X[63][0];
    auvProtocol->send_data.dataAH127C.X_accel = X[64][0];
    auvProtocol->send_data.dataAH127C.Y_accel = X[65][0];
    auvProtocol->send_data.dataAH127C.Z_accel = X[66][0];
    auvProtocol->send_data.dataAH127C.X_rate = X[67][0];
    auvProtocol->send_data.dataAH127C.Y_rate = X[68][0];
    auvProtocol->send_data.dataAH127C.Z_rate = X[69][0];
    auvProtocol->send_data.dataAH127C.X_magn = X[70][0];
    auvProtocol->send_data.dataAH127C.Y_magn = X[71][0];
    auvProtocol->send_data.dataAH127C.Z_magn = X[72][0];
    auvProtocol->send_data.dataAH127C.quat[0] = X[73][0];
    auvProtocol->send_data.dataAH127C.quat[1] = X[74][0];
    auvProtocol->send_data.dataAH127C.quat[2] = X[75][0];
    auvProtocol->send_data.dataAH127C.quat[3] = X[76][0];

    auvProtocol->send_data.auvData.signalVMA_real.VMA1 = X[80][0];
    auvProtocol->send_data.auvData.signalVMA_real.VMA2 = X[81][0];
    auvProtocol->send_data.auvData.signalVMA_real.VMA3 = X[82][0];
    auvProtocol->send_data.auvData.signalVMA_real.VMA4 = X[83][0];

//    auvProtocol->send_data.diagnostics.PMW1 = diagnostic ->data.PMW1;
//    auvProtocol->send_data.diagnostics.PMW2 = diagnostic ->data.PMW2;
//    auvProtocol->send_data.diagnostics.PMW3 = diagnostic ->data.PMW3;
//    auvProtocol->send_data.diagnostics.current_1 = diagnostic ->data.current_1;
//    auvProtocol->send_data.diagnostics.current_2 = diagnostic ->data.current_2;
//    auvProtocol->send_data.diagnostics.current_3 = diagnostic ->data.current_3;
//    auvProtocol->send_data.diagnostics.current_4 = diagnostic ->data.current_4;
//    auvProtocol->send_data.diagnostics.voltage_4 = diagnostic ->data.voltage_4;
//    auvProtocol->send_data.diagnostics.voltage_5v = diagnostic ->data.voltage_5v;
//    auvProtocol->send_data.diagnostics.voltage_12v = diagnostic ->data.voltage_12v;
//    auvProtocol->send_data.diagnostics.voltage_bat = diagnostic ->data.voltage_bat;

    auvProtocol->send_data.dataGANS.azimuth = X[41][0];
    auvProtocol->send_data.dataGANS.distance = X[42][0];
    auvProtocol->send_data.dataGANS.dataValue = X[43][0];
    auvProtocol->send_data.dataGANS.temperature = X[44][0];
    auvProtocol->send_data.dataGANS.depth = X[45][0];
    auvProtocol->send_data.dataGANS.roll_GANS = X[46][0];
    auvProtocol->send_data.dataGANS.pitch_GANS = X[47][0];

    auvProtocol->send_data.angularGPS.time_UTC = gpsProt->gps->psat.time;
    auvProtocol->send_data.angularGPS.yaw = gpsProt->gps->psat.yaw;
    auvProtocol->send_data.angularGPS.pitch = gpsProt->gps->psat.pitch;
    auvProtocol->send_data.angularGPS.roll = gpsProt->gps->psat.roll;
    auvProtocol->send_data.angularGPS.dataType = gpsProt->gps->psat.dataType;
    auvProtocol->send_data.coordinateGPS.time = gpsProt->gps->gga.time;
    auvProtocol->send_data.coordinateGPS.latitude = gpsProt->gps->gga.latitude;
    auvProtocol->send_data.coordinateGPS.latHemisphere = gpsProt->gps->gga.latHemisphere;
    auvProtocol->send_data.coordinateGPS.longitude = gpsProt->gps->gga.longitude;
    auvProtocol->send_data.coordinateGPS.lonHemisphere = gpsProt->gps->gga.lonHemisphere;
    auvProtocol->send_data.coordinateGPS.quality = gpsProt->gps->gga.quality;
    auvProtocol->send_data.coordinateGPS.satellitesUsed = gpsProt->gps->gga.satellitesUsed;
    auvProtocol->send_data.coordinateGPS.hdop = gpsProt->gps->gga.hdop;
    auvProtocol->send_data.coordinateGPS.altitude = gpsProt->gps->gga.altitude;
    auvProtocol->send_data.coordinateGPS.altitudeUnit = gpsProt->gps->gga.altitudeUnit;
    auvProtocol->send_data.coordinateGPS.geoidHeight = gpsProt->gps->gga.geoidHeight;
    auvProtocol->send_data.coordinateGPS.geoidUnit = gpsProt->gps->gga.geoidUnit;
    auvProtocol->send_data.coordinateGPS.dgpsAge = gpsProt->gps->gga.dgpsAge;
    auvProtocol->send_data.coordinateGPS.dgpsStationId = gpsProt->gps->gga.dgpsStationId;

    auvProtocol->send_data.altitude = echolot->echo.depth;
    auvProtocol->send_data.count_receive_gans = GANS->data.pzmae.count_answer;
    qDebug() << "count_receive_gans" << auvProtocol->send_data.count_receive_gans;

    sspData.counter +=1;
    sspData.latitudeReper = auvProtocol->rec_data.reper.x_point;
    sspData.longitudeReper= auvProtocol->rec_data.reper.y_point;
    sspData.X             = current_point.x_point;
    sspData.Y             = current_point.y_point;
    sspData.yawMagn       = X[178][0];
    sspData.yawIner       = X[91][0];
    sspData.yaw           = X[61][0];
    sspData.pitch         = X[62][0];
    sspData.roll          = X[63][0];
    sspData.X_accel       = X[64][0];
    sspData.Y_accel       = X[65][0];
    sspData.Z_accel       = X[66][0];
    sspData.X_rate        = X[67][0];
    sspData.Y_rate        = X[68][0];
    sspData.Z_rate        = X[69][0];
    sspData.X_magn        = X[70][0];
    sspData.Y_magn        = X[71][0];
    sspData.Z_magn        = X[72][0];
    sspData.quat[0]       = X[73][0];
    sspData.quat[1]       = X[74][0];
    sspData.quat[2]       = X[75][0];
    sspData.quat[3]       = X[76][0];
    sspData.VMA1          = X[80][0];
    sspData.VMA2          = X[81][0];
    sspData.VMA3          = X[82][0];
    sspData.VMA4          = X[83][0];
    sspData.flag_yaw      = contour_closure_yaw;
    sspData.flag_pitch    = contour_closure_pitch;
    sspData.flag_roll     = contour_closure_roll;
    sspData.flag_march    = contour_closure_march;
    sspData.flag_depth    = contour_closure_depth;
    sspData.flag_lag      = contour_closure_lag;
    sspData.modeReal      = flag_of_mode;
    sspData.yawSet = X[5][0];
    sspData.latitudePerpose = auvProtocol->rec_data.mission_param.point_mission.x_point;
    sspData.longitudePerpose = auvProtocol->rec_data.mission_param.point_mission.y_point;
    emit updateSSP(&sspData);
}

void CS_USV::aperiodicFilter(double &input, double &output, double &prevOutput, double K, double T, double dt)
{
    if (T !=0) output = prevOutput + dt*(1/T)*(input*K-prevOutput);
    else output = K*input;
    prevOutput = output;
}

void CS_USV::controlPitch(double dt)
{
//    if (auvProtocol->rec_data.controlContoursFlags.pitch>0) { //замкнут дифферент
//        processDesiredValuesAutomatiz(X[52][0],X[6][0],X[6][1],K[2]); //пересчет рукоятки в автоматизированном режиме
//        X[311][0] = X[6][0] - X[62][0];
//        X[312][0] = X[311][0]*K[311];
//        X[313][0] = X[312][0]*K[312];
//        X[314][0] = X[314][1] + 0.5*(X[313][0] + X[313][1])*dt; //выходное значение интегратора без полок

//        if (K[313] != 0){//значит заданы полки
//            X[314][0] = saturation(X[314][0],K[313],K[314]); //выходное значение интегратора с полками
//         }
//         X[314][1] = X[314][0];
//         X[313][1] = X[313][0];

//         X[316][0] = X[314][0] + X[312][0];

//         X[321][0] = X[68][0]*K[318];
//         X[319][0] = X[52][0]*K[319];
//         X[317][0] = X[116][0] - X[121][0] + X[119][0];
//   } else {
//         X[317][0] = K[301]*X[52][0]; //Upsi
//         resetRollChannel();
//   }
//   X[318][0] = saturation(X[317][0],K[316],-K[316]);
//   X[301][0] = X[318][0]*K[300];
}

