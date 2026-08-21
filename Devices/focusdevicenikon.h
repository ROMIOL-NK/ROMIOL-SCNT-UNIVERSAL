#ifndef FOCUSDEVICENIKON_H
#define FOCUSDEVICENIKON_H

#include "Base/focusdevice.h"

#import <NikonTi.dll>named_guids
#import <MipParam2.dll>named_guids
// Nikon TiE，参考安装目录下**/TiSDK/Help/NIKONTIE.chm

class FocusDeviceNikon : public FocusDevice
{
    Q_OBJECT
public:
    FocusDeviceNikon();
    ~FocusDeviceNikon();

private:
    TISCOPELib::INikonTiPtr pMicroscope;    // 显微镜指针
    bool isTiEConnected;                    // 显微镜实例化是否成功
    TISCOPELib::IZDrive * pZDrive;          // 调焦轴指针

public slots:
    void connectDevice(QString portName=strPortFocus);              //连接设备
    void move(float z, bool CONTINUOUS, float vel);
    void moveDistance(float z);                                     //移动 z um, 速度为 inchingSpeed
    void moveContinuous(float v);                                   //以速度 v um/s 移动
    void stop();                                                    //停止运动
    void close();                                                   //关闭设备
    void setInchingSpeed(float speed);                              //设置联动速度

    void stateQuery();
};

#endif // FOCUSDEVICENIKON_H
