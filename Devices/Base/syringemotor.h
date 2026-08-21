#ifndef SYRINGEMOTOR_H
#define SYRINGEMOTOR_H

#include "public.h"

class SyringeMotor: public QObject
{
    Q_OBJECT
public:
    SyringeMotor(){isConnected=false;ori = 0;DeviceName="Syringe Motor";};
    friend class Injector;
    friend class Holder;

public:
    QString DeviceName;     //设备名称
    float inchingSpeed;     //点动速度
    float raw_pos;          //原始坐标
    float raw_vel;          //原始速度
    bool  b_same_dir;       //与规范中的正方向是否一致
    bool  isConnected;      //连接状态
    float upperBound;       //电机上限位
    float lowerBound;       //电机下限位
    float ori;              //自定义原点
    int   pulsePerRound;    //一圈对应的脉冲数
public slots:
    virtual void connectDevice(QString portName="")=0;                      //连接设备
    virtual void move(float disp, float vel=0, bool CONTINUOUS=false)=0;    //相对运动
    virtual void suck(float disp, float vel=0, bool CONTINUOUS=false)=0;    //注射器吸
    virtual void blow(float disp, float vel=0, bool CONTINUOUS=false)=0;    //注射器注
    virtual void stop()=0;                                                  //停止运动
    virtual void close()=0;                                                 //关闭设备
    virtual void setInchingSpeed(float speed){inchingSpeed = speed;}        //设置点动速度

    float getAbsPos(){return b_same_dir?raw_pos:-raw_pos;}                  //获取绝对坐标
    float getRelPos(){return getAbsPos()-ori;}                              //获取相对坐标
    float getVel(){return b_same_dir?raw_vel:-raw_vel;}                     //获取实际速度
    float getInchingSpeed(){return inchingSpeed;}                           //获取点动速度

signals:
    void openPortFailure();         //通知 UI 打开端口失败
    void startConnection();         //通知 UI 已建立与该端口的通讯
    void noConnection();            //通知 UI 失去与该设备的通讯
    void updatePos();               //通知 UI 更新位置
};
#endif // SYRINGEMOTOR_H
