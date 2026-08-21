/* 调焦设备：建立通讯、运动控制（点动、连动）、设置参数*/
#ifndef FOCUSDEVICE_H
#define FOCUSDEVICE_H

#include "public.h"

class FocusDevice : public QObject
{
    Q_OBJECT
public:
    FocusDevice(){isConnected=false;};
    friend class Microscope;

protected:
    float raw_pos;      //原始 z 坐标
    float inchingSpeed; //点动速度
    bool  b_same_dir;   //与规范中的正方向是否一致
public:
    QString DeviceName; //设备名称
    bool  isConnected;  //连接状态

    QTimer* queryStateTimer;//查询位置的定时器
    int queryStateInterval; //查询位置的时间间隔
public slots:
    virtual void connectDevice(QString portName=strPortFocus)=0;            //连接设备
    virtual void move(float dis, bool CONTINUOUTS, float vel)=0;            //
    virtual void moveDistance(float z)=0;                                   //移动 z um, 速度为 inchingSpeed
    virtual void moveContinuous(float v)=0;                                 //以速度 v um/s 移动
    virtual void stop()=0;                                                  //停止运动
    virtual void close()=0;                                                 //关闭设备

    virtual void setInchingSpeed(float speed){inchingSpeed = speed;}        //设置点动速度
    float getInchingSpeed(){return inchingSpeed;}                           //获取点动速度
    float getZPos(){return b_same_dir?raw_pos:-raw_pos;}                    //获取 z 坐标

signals:
    void openPortFailure();         //通知 UI 打开端口失败
    void startConnection();         //通知 UI 已建立与该端口的通讯
    void noConnection();            //通知 UI 失去与该设备的通讯
    void updateZPos();              //通知 UI 更新 z 轴位置
};

#endif // FOCUSDEVICE_H
