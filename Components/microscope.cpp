#include "microscope.h"

Microscope::Microscope()
{
    //-----------------深圳系统 start-----------------------
#if (defined SYSTEM_SZ_IPC)||(defined SYSTEM_SZ_STATION)
    focusdevice = new FocusDevicePrior;
    stage       = new StageTrio;
    camera      = new CameraToup;
#endif
    //-----------------深圳系统 end-----------------------

    //-----------------天津系统 start-----------------------
#ifdef SYSTEM_TJ
    isTiEConnected = true;
    ::CoInitialize(NULL);
    if (pMicroscope.CreateInstance(TISCOPELib::CLSID_NikonTi) != S_OK)  // 显微镜无法初始化
    {
        ::CoUninitialize();
        isTiEConnected = false;
        qDebug() << "Can't connect Nikon microscpe";
    }
    // 106 Devices
    // focusdevice = new FocusDeviceNikon;
    // stage = new StagePrior;
    focusdevice = new FocusDevicePrior;
    stage = new StageZMotion;
    camera      = new CameraUSB;
#endif
    //-----------------天津系统 end-----------------------
    oriz = 0;
}
Microscope::~Microscope()
{
    if(focusdevice)
        delete focusdevice;
    if(stage)
        delete stage;
    if(camera)
        delete camera;
    if(isTiEConnected)
    {
        pMicroscope.Release();
        pMicroscope = NULL;
        isTiEConnected = false;
        ::CoUninitialize();
    }
}

void Microscope::captureFrame(QString frame_path)
{
    camera->captureFrame(frame_path);
}

void Microscope::moveX(float x, bool CONTINUOUS, float vel)
{
    stage->moveX(x, CONTINUOUS, vel);
}
void Microscope::moveY(float y, bool CONTINUOUS, float vel)
{
    stage->moveY(y, CONTINUOUS, vel);
}
void Microscope::moveZ(float z, bool CONTINUOUS, float vel)
{
    focusdevice->move(z, CONTINUOUS, vel);
}
void Microscope::move(float dx, float dy, float dz)
{
    stage->moveX(dx);
    stage->moveY(dy);
    focusdevice->moveDistance(dz);
}
void Microscope::moveToABS(float x, float y, float z)
{
    moveX(x-getAbsX());
    moveY(y-getAbsY());
    moveZ(z-getAbsZ());
}
void Microscope::moveToREL(float x, float y, float z)
{
    moveX(x-getRelX());
    moveY(y-getRelY());
    moveZ(z-getRelZ());
}
void Microscope::stop()
{
    focusdevice->stop();
    stage->stop();
}

void Microscope::closeCamera()
{
    camera->close();
}
void Microscope::changeObjective(Objective tarObj)
{
    if(CurObj==tarObj)//如果点击当前倍率，返回
        return;
    // QMessageBox::warning(NULL, tr("提示"), "请手动切换镜头!", QMessageBox::Yes);

    switch(CurObj)
    {
    case x4:
        if(tarObj==x10)
        {
            Pixel2Um = Pixel2Um_10x;
//            microscope->moveX(BiasX_4to10);
//            microscope->moveY(BiasY_4to10);
//            microscope->moveZ(BiasZ_4to10);
        }
        if(tarObj==x20)
        {
            Pixel2Um = Pixel2Um_20x;
//            microscope->moveX(BiasX_4to10+BiasX_10to20);
//            microscope->moveY(BiasY_4to10+BiasY_10to20);
//            microscope->moveZ(BiasZ_4to10+BiasZ_10to20);
        }
        if(tarObj==x40)
        {
            Pixel2Um = Pixel2Um_40x;
//            microscope->moveX(BiasX_4to10+BiasX_10to20+BiasX_20to40);
//            microscope->moveY(BiasY_4to10+BiasY_10to20+BiasY_20to40);
//            microscope->moveZ(BiasZ_4to10+BiasZ_10to20+BiasZ_20to40);
        }
        break;

    case x10:
        if(tarObj==x4)
        {
            Pixel2Um = Pixel2Um_4x;
//            microscope->moveX(-BiasX_4to10);
//            microscope->moveY(-BiasY_4to10);
//            microscope->moveZ(-BiasZ_4to10);
        }
        if(tarObj==x20)
        {
            Pixel2Um = Pixel2Um_20x;
//            microscope->moveX(BiasX_10to20);
//            microscope->moveY(BiasY_10to20);
//            microscope->moveZ(BiasZ_10to20);
        }
        if(tarObj==x40)
        {
            Pixel2Um = Pixel2Um_40x;
//            microscope->moveX(BiasX_10to20+BiasX_20to40);
//            microscope->moveY(BiasY_10to20+BiasY_20to40);
//            microscope->moveZ(BiasZ_10to20+BiasZ_20to40);
        }
        break;
    case x20:
        if(tarObj==x4)
        {
            Pixel2Um = Pixel2Um_4x;
//            microscope->moveX(-(BiasX_4to10+BiasX_10to20));
//            microscope->moveY(-(BiasY_4to10+BiasY_10to20));
//            microscope->moveZ(-(BiasZ_4to10+BiasZ_10to20));
        }
        if(tarObj==x10)
        {
            Pixel2Um = Pixel2Um_10x;
//            microscope->moveX(-BiasX_10to20);
//            microscope->moveY(-BiasY_10to20);
//            microscope->moveZ(-BiasZ_10to20);
        }
        if(tarObj==x40)
        {
            Pixel2Um = Pixel2Um_40x;
//            microscope->moveX(BiasX_20to40);
//            microscope->moveY(BiasY_20to40);
//            microscope->moveZ(BiasZ_20to40);
        }
        break;
    case x40:
        if(tarObj==x4)
        {
            Pixel2Um = Pixel2Um_4x;
//            microscope->moveX(-(BiasX_4to10+BiasX_10to20+BiasX_20to40));
//            microscope->moveY(-(BiasY_4to10+BiasY_10to20+BiasY_20to40));
//            microscope->moveZ(-(BiasZ_4to10+BiasZ_10to20+BiasZ_20to40));
        }
        if(tarObj==x10)
        {
            Pixel2Um = Pixel2Um_10x;
//            microscope->moveX(-(BiasX_10to20+BiasX_20to40));
//            microscope->moveY(-(BiasY_10to20+BiasY_20to40));
//            microscope->moveZ(-(BiasZ_10to20+BiasZ_20to40));
        }
        if(tarObj==x20)
        {
            Pixel2Um = Pixel2Um_20x;
//            microscope->moveX(-BiasX_20to40);
//            microscope->moveY(-BiasY_20to40);
//            microscope->moveZ(-BiasZ_20to40);
        }
        break;
    }
    CurObj = tarObj;
}
