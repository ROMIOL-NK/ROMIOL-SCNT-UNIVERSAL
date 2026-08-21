#ifndef STAGE_H
#define STAGE_H

#include "public.h"

class Stage : public QObject
{
    Q_OBJECT
public:
    Stage(){orix = 0;oriy = 0;};
    ~Stage(){};
    friend class Microscope;

public:
    QString DeviceName;         //设备名称
    float raw_xpos;             //从下位机获取的原始 x 位置: um
    float raw_ypos;             //从下位机获取的原始 y 位置: um
    float inchingSpeed;         //点动速度
    bool b_same_dirX;           //下位机 x正方向与程序中规定方向相同的标志
    bool b_same_dirY;           //下位机 y正方向与程序中规定方向相同的标志
    bool isConnected;           //设备已连接的标志

    float orix;                 //x的自定义原点
    float oriy;                 //y的自定义原点

    QTimer* queryStateTimer;    //位置查询的定时器
    int   queryStateInterval;   //查询位置的时间间隔


public slots:
    virtual void connectDevice(QString portName=strPortStage)=0;        //连接设备
    virtual void moveX(float x, bool CONTINUOUS=false, float vel=0)=0;  //x 方向运动
    virtual void moveY(float y, bool CONTINUOUS=false, float vel=0)=0;  //y 方向运动
    virtual void move(float x, float y, float vel=0)=0;                 //x y 方向运动
    virtual void stop()=0;                                              //停止运动
    virtual void close()=0;                                             //关闭设备
    virtual void setInchingSpeed(float speed){inchingSpeed = speed;}    //设置点动速度
    float getInchingSpeed(){return inchingSpeed;}                       //获取点动速度
    float getXPos(){return b_same_dirX?raw_xpos:-raw_xpos;}             //返回 X 坐标
    float getYPos(){return b_same_dirY?raw_ypos:-raw_ypos;}             //返回 Y 坐标
    void moveToArea(cv::Point2f area){
        if(area!=cv::Point2f(INFINITY, INFINITY))
            move(area.x-(getXPos()-orix), area.y-(getYPos()-oriy), STAGE_VEL_AREA);
    }

signals:
    void startConnection();//通知 UI 已建立与该端口的通讯
    void openPortFailure();//通知 UI 打开端口失败
    void noConnection();   //通知 UI 失去与该设备的通讯
    void updateXYPos();
};

#endif // STAGE_H
