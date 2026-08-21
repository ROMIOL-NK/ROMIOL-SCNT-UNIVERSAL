#ifndef MANIPULATOR_H
#define MANIPULATOR_H

#include "public.h"

class Manipulator: public QObject
{
    Q_OBJECT
public:
    Manipulator(QString name="Manipulator"){isConnected=false; DeviceName=name;orix=0;oriy=0;oriz=0;}
    friend class Injector;
    friend class Holder;

protected:
    QString DeviceName;     //设备名称
    float inchingSpeed;     //点动速度
    float raw_xpos;         //原始 x 坐标
    float raw_ypos;         //原始 y 坐标
    float raw_zpos;         //原始 z 坐标
    bool  b_same_xdir;      //与规范中 x轴 的正方向是否一致
    bool  b_same_ydir;      //与规范中 y轴 的正方向是否一致
    bool  b_same_zdir;      //与规范中 z轴 的正方向是否一致
public:
    bool  isConnected;      //连接状态

    float orix;//x的自定义原点
    float oriy;//y的自定义原点
    float oriz;//z的自定义原点
public slots:
    virtual void connectDevice(QString portName)=0;                         //连接设备
    virtual void moveX(float x, float vel=0, bool CONTINUOUS=false)=0;      //x轴相对运动
    virtual void moveY(float y, float vel=0, bool CONTINUOUS=false)=0;      //y轴相对运动
    virtual void moveZ(float z, float vel=0, bool CONTINUOUS=false)=0;      //z轴相对运动
    virtual void move(float x, float y, float z, float v=0)=0;              //任意方向相对运动 v=0: 默认速度 v > 0 以设定速度运动
    virtual void stop()=0;                                                  //停止运动
    virtual void close()=0;                                                 //关闭设备
    virtual void setInchingSpeed(float speed){inchingSpeed = speed;}        //设置点动速度

    virtual void velocityPlan(){}                                           //line速度刺入
    virtual void velocityPlanSin(){}                                        //sin速度刺入
    virtual void velocityPlanQua(){}                                        //sin速度刺入

    float getAbsX(){return b_same_xdir?raw_xpos:-raw_xpos;}                 //获取 x 绝对坐标
    float getAbsY(){return b_same_ydir?raw_ypos:-raw_ypos;}                 //获取 y 绝对坐标
    float getAbsZ(){return b_same_zdir?raw_zpos:-raw_zpos;}                 //获取 z 绝对坐标
    float getRelX(){return getAbsX()-orix;}                                 //获取 x 相对坐标
    float getRelY(){return getAbsY()-oriy;}                                 //获取 y 相对坐标
    float getRelZ(){return getAbsZ()-oriz;}                                 //获取 z 相对坐标
    float getInchingSpeed(){return inchingSpeed;}                           //获取点动速度

signals:
    void openPortFailure();         //通知 UI 打开端口失败
    void startConnection();         //通知 UI 已建立与该端口的通讯
    void noConnection();            //通知 UI 失去与该设备的通讯
    void updatePos();               //通知 UI 更新位置
};

#endif // MANIPULATOR_H
