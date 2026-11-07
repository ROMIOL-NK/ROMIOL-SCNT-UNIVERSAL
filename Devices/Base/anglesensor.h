#ifndef ANGLESENSOR_H
#define ANGLESENSOR_H
#include "public.h"

class AngleSensor: public QObject
{
    Q_OBJECT
public:
    AngleSensor(){DeviceName="Angle Sensor";};
    QString DeviceName;     //设备名称
    QString unit;
    bool isConnected;                                       //设备连接状态
    float angle;

    float getAngle(){return angle;};

signals:
    void updateAngle();
    void startConnection();                                 //通知 UI 已建立与该端口的通讯
    void openPortFailure();                                 //通知 UI 打开端口失败
    void noConnection();                                    //通知 UI 失去与该设备的通讯


public slots:
    //建立与设备之间的通讯：1.打开端口；2.发送测试指令，收到返回信息(endflag为结尾符)则成功建立 TODO：测试指令与设备相关
    virtual void connectDevice(QString portName)=0;
    virtual void close()=0;
};

#endif // ANGLESENSOR_H
