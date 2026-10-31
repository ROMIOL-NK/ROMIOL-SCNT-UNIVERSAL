#include "injector.h"

Injector::Injector()
{
#if (defined SYSTEM_SZ_IPC)||(defined SYSTEM_SZ_STATION)
    manipulator = new ManipulatorHuayue("Right Manipulator Scientifica", 0x01, 1, -1, -1);
    // syringeMotor = new SyringeMotorGT();isLiquid = true;
    syringeMotor = new SyringeMotorVince(6400);isLiquid = false;
    pSensor = new PSensorMIK_P300();
    anglesensor = new AngleSensorAOD();
#endif

#ifdef SYSTEM_TJ
    manipulator = new ManipulatorScientificaMotion8("Right Manipulator ScientificaMotion8");
    // syringeMotor = new SyringeMotorGT();isLiquid = true;
    syringeMotor = new SyringeMotorVince(6400);isLiquid = false;
    pSensor = new PSensorMIK_P300();
#endif

}
Injector::~Injector()
{}
