#ifndef ANGLESENSOR_AOD_H
#define ANGLESENSOR_AOD_H
#include "Base/anglesensor.h"

class AngleSensorAOD: public AngleSensor
{
    Q_OBJECT
public:
    AngleSensorAOD(QString name="Angle Sensor AOD");
    ~AngleSensorAOD();
private:

    QSerialPort *m_port;
    QByteArray RxBuffer;                //ReadAll读到的当前返回数据
    QByteArray msg;                     //有结束标记位的接受信号
    QByteArray queryStateCommand;

    QVector<QByteArray> commandList;    //等待发送的命令列表
    QVector<bool>       responseTypes;  //命令类型（是否有返回信号）的标志列表
    QTimer* timer;                      //等待返回信息的定时器，超时将认为设备已断开连接
    QTimer* queryStateTimer;            //查询位置的定时器
    bool testMode;                      //连接时发送测试信号，收到返回信息则认为建立连接
    int queryStateInterval;             //查询位置的时间间隔

private slots:
    void connectDevice(QString portName);                           //连接设备
    void close();
    double calculateAngle(QByteArray msg);
    void queue(QByteArray, bool b_with_response=true);
    void send();
    void recv();
};

#endif // ANGLESENSOR_AOD_H
