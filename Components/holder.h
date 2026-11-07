#ifndef HOLDER_H
#define HOLDER_H

#include "public.h"
#include "Devices/manipulatormp285.h"
#include "Devices/manipulatorscientifica.h"
#include "Devices/manipulatorscientificamotion8.h"
#include "Devices/manipulatorhuayue.h"
#include "Devices/syringemotorgt.h"
#include "Devices/syringemotorvince.h"
#include "Devices/psensormik_p3000.h"
#include "Devices/psensormik_p300.h"

class Holder: public QObject
{
public:
    Holder();
    ~Holder();

    Manipulator*    manipulator;
    SyringeMotor*   syringeMotor;
    PSensor*        pSensor;

    bool isConnected(){return manipulator->isConnected&syringeMotor->isConnected;}//设备连接信息 StepFetchOocyte::
    void stop(){manipulator->stop(); syringeMotor->stop();}
    void close(){manipulator->close();syringeMotor->close();pSensor->close();}
};

#endif // HOLDER_H
