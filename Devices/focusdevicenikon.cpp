#include "focusdevicenikon.h"

FocusDeviceNikon::FocusDeviceNikon()
{
    b_same_dir = true;                  //设备正方向与规定正方向是否相同
    DeviceName = "Focus Device Nikon";
    pMicroscope = NULL;
    pZDrive = NULL;
    isConnected = false;
    isTiEConnected = true;
    queryStateTimer = new QTimer(this);
    queryStateInterval = 1000;
    connect(queryStateTimer, &QTimer::timeout, [this](){stateQuery();});
}

FocusDeviceNikon::~FocusDeviceNikon()
{
    pZDrive->Release();
    pZDrive = NULL;
    if(isTiEConnected)
    {
        pMicroscope.Release();
        pMicroscope = NULL;
        isTiEConnected = false;
        ::CoUninitialize();
    }
    delete queryStateTimer;
}


void FocusDeviceNikon::connectDevice(QString portName)
{
    ::CoInitialize(NULL);
    if (pMicroscope.CreateInstance(TISCOPELib::CLSID_NikonTi) != S_OK)  // 显微镜无法初始化
    {
        isTiEConnected = false;
        qDebug() << "Can't connect Nikon microscpe";
        return;
    }
    if (pMicroscope->get_ZDrive(&pZDrive) != S_OK)  // 无法获得z轴
    {
        qDebug() << "Can't get " + DeviceName;
        return;
    }
    if (long(pZDrive->IsMounted) != TISCOPELib::StatusTrue)        // z轴没有安装
    {
        qDebug() << "Can't mount " + DeviceName;
    }
    else    // 成功连接
    {
        isConnected = true;
        emit startConnection();
        queryStateTimer->start(queryStateInterval);
    }
}

void FocusDeviceNikon::move(float z, bool CONTINUOUS, float vel)
{
    if(vel!=0&&inchingSpeed!=abs(vel))
        setInchingSpeed(abs(vel));
    if(CONTINUOUS)
        moveContinuous(vel);
    else
        moveDistance(z);
}

void FocusDeviceNikon::moveDistance(float z)//move z [um]; z>0 : Up; z<0 : Down
{
    if (!isConnected)   // 没有连接，直接返回
    {
        return;
    }
    if (long(pZDrive->IsMounted) != TISCOPELib::StatusTrue)        // z轴没有安装
    {
        qDebug() << "Can't mount " + DeviceName;
        emit noConnection();
        return;
    }
    float zz = z*40; // 1 um = 40 steps
    if (!b_same_dir)
        zz = -zz;
    float current_zpos = pZDrive->Position;
    current_zpos += zz;
    pZDrive->Position = current_zpos;

    raw_pos = current_zpos/40;
    emit updateZPos();
}

void FocusDeviceNikon::close()
{
    if(pZDrive)
        pZDrive->Release();
    pZDrive = NULL;
    queryStateTimer->stop();
    isConnected = false;
}

void FocusDeviceNikon::stateQuery()
{
    if (long(pZDrive->IsMounted) != TISCOPELib::StatusTrue)        // z轴没有安装
    {
        qDebug() << "Can't mount " + DeviceName;
        isConnected = false;
        emit noConnection();
        return;
    }

    float current_zpos = pZDrive->Position;
    raw_pos = current_zpos/40;
    emit updateZPos();

}

// ----------------------------------//
// Nikon显微镜调焦暂时无法实现
void FocusDeviceNikon::setInchingSpeed(float V)
{
    return;
}
void FocusDeviceNikon::moveContinuous(float v)
{
    return;
}
void FocusDeviceNikon::stop()
{
    return;
}


