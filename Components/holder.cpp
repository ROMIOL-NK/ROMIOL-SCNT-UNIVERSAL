#include "holder.h"

Holder::Holder()
{
    //-----------------深圳系统 start-----------------------
#if (defined SYSTEM_SZ_IPC)||(defined SYSTEM_SZ_STATION)
    manipulator = new ManipulatorHuayue("Left Manipulator", 0x01, -1, -1, -1);
    // syringeMotor = new SyringeMotorGT();
    syringeMotor = new SyringeMotorVince(); syringeMotor->b_same_dir = false;
    pSensor = new PSensorMIK_P3000();
#endif
    //-----------------深圳系统 end-----------------------

    //-----------------天津系统 start-----------------------
#ifdef SYSTEM_TJ
    manipulator = new ManipulatorMP285("Left Manipulator");
    // syringeMotor = new SyringeMotorGT();
    syringeMotor = new SyringeMotorVince(); syringeMotor->b_same_dir = false;
    pSensor = new PSensorMIK_P300();
#endif
    //-----------------天津系统 end-----------------------
}
Holder::~Holder()
{}
