#ifndef INJECTOR_H
#define INJECTOR_H

#include "public.h"
#include "Devices/manipulatormp285.h"
#include "Devices/manipulatorscientifica.h"
#include "Devices/manipulatorscientificamotion8.h"
#include "Devices/manipulatorhuayue.h"
#include "Devices/syringemotorgt.h"
#include "Devices/syringemotorvince.h"
#include "Devices/psensormik_p3000.h"
#include "Devices/psensormik_p300.h"
#include "Devices/anglesensoraod.h"

class Injector: public QObject
{
public:
    Injector();
    ~Injector();

    Manipulator*    manipulator;
    SyringeMotor*   syringeMotor;
    PSensor*        pSensor;
    AngleSensor*    anglesensor;

    bool isLiquid;                  //是否为液泵

    bool isConnected(){return manipulator->isConnected&syringeMotor->isConnected&pSensor->isConnected;}//设备连接信息
    void stop(){manipulator->stop(); syringeMotor->stop();}
    void close(){manipulator->close();syringeMotor->close();pSensor->close();}
};

#endif // INJECTOR_H
