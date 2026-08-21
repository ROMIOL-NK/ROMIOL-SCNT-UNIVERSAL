#ifndef PSENSORMIK_P300_H
#define PSENSORMIK_P300_H
#include "Base/psensor.h"

class PSensorMIK_P300: public PSensor
{
    Q_OBJECT
public:
    PSensorMIK_P300(QString name="Pressure Sensor Mik-P300");
    ~PSensorMIK_P300();
private:

    QSerialPort *m_port;
    QByteArray RxBuffer;                //ReadAll读到的当前返回数据
    QByteArray msg;                     //有结束标记位的接受信号
    QByteArray queryStateCommand;

    QVector<QByteArray> commandList;    //等待发送的命令列表
    QVector<bool>       responseTypes;  //命令类型（是否有返回信号）的标志列表
    QTimer* connectionTimer;                      //等待返回信息的定时器，超时将认为设备已断开连接
    QTimer* queryStateTimer;            //查询位置的定时器
    bool loseConnection;
    int queryStateInterval;             //查询位置的时间间隔
    int connectionInterval;

public slots:
    void connectDevice(QString portName);                           //连接设备
    void close();
    int convert(QByteArray msg);
    void queue(QByteArray, bool b_with_response=true);
    void send();
    void recv();
};

#endif // PSENSORMIK_P300_H
