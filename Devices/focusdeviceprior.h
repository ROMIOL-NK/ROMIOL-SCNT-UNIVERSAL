#ifndef FOCUSDEVICEPRIOR_H
#define FOCUSDEVICEPRIOR_H

#include "Base/focusdevice.h"
// Prior ES10ZE https://www.prior.com/wp-content/uploads/2017/07/Updated-ES10ZE-Manual.pdf
class FocusDevicePrior : public FocusDevice
{
    Q_OBJECT
public:
    FocusDevicePrior();
    ~FocusDevicePrior();
private:
    QSerialPort *m_port;
    QByteArray RxBuffer;                //ReadAll读到的当前返回数据
    QByteArray msg;                     //有结束标记位的接受信号
    QByteArray endflag;                 //返回信息的结尾符
    QVector<QByteArray> commandList;    //等待发送的命令列表
    QVector<bool>       responseTypes;  //命令类型（是否有返回信号）的标志列表
    QTimer* connectionTimer;            //等待返回信息的定时器，超时将认为设备已断开连接
    bool loseConnection;
    int connectionInterval;

public slots:
    void connectDevice(QString portName=strPortFocus);              //连接设备
    void move(float z, bool CONTINUOUS, float vel);
    void moveDistance(float z);                                     //移动 z um, 速度为 inchingSpeed
    void moveContinuous(float v);                                   //以速度 v um/s 移动
    void stop();                                                    //停止运动
    void close();                                                   //关闭设备
    void setInchingSpeed(float speed);                              //设置点动速度
    void queue(QByteArray, bool b_with_response=true);
    void send();
    void recv();
    void setStepW(double);
};

#endif // FOCUSDEVICEPRIOR_H
