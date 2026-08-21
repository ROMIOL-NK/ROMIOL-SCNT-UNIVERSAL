#ifndef PSENSOR_H
#define PSENSOR_H
#include "public.h"

class PSensor: public QObject
{
    Q_OBJECT
public:
    PSensor(){DeviceName="Pressure Sensor";}
    QString DeviceName;     //设备名称
    friend class Injector;
    friend class Holder;
    QString unit;
    bool isConnected;                                       //设备连接状态
    float pressure;

    float getPressure(){return pressure;}

signals:
    void updatePressure();
    void updateUnit();
    void startConnection();                                 //通知 UI 已建立与该端口的通讯
    void openPortFailure();                                 //通知 UI 打开端口失败
    void noConnection();                                    //通知 UI 失去与该设备的通讯


public slots:
    //建立与设备之间的通讯：1.打开端口；2.发送测试指令，收到返回信息(endflag为结尾符)则成功建立 TODO：测试指令与设备相关
    virtual void connectDevice(QString portName)=0;
    virtual void close()=0;
//    virtual void setUnit(QString unit)=0;                   //设置气压单位
    QString getUnit(){return unit;}
};

#endif // PSENSOR_H
