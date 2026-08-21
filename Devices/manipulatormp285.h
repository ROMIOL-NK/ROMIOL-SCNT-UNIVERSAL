#ifndef MANIPULATORMP285_H
#define MANIPULATORMP285_H

#include "Base/manipulator.h"
// Sutter MP285 https://www.sutter.com/MICROMANIPULATION/mp285.html
class ManipulatorMP285 : public Manipulator
{
    Q_OBJECT
public:
    ManipulatorMP285(QString name);
    ~ManipulatorMP285();

public:
    QSerialPort *m_port;
    QByteArray queryStateCommand;       //查询状态的指令
    QByteArray RxBuffer;                //ReadAll读到的当前返回数据
    QByteArray msg;                     //有结束标记位的接受信号
    QByteArray endflag;                 //返回信息的结尾符
    QVector<QByteArray> commandList;    //等待发送的命令列表
    QVector<bool>       responseTypes;  //命令类型（是否有返回信号）的标志列表
    QTimer* connectionTimer;          //等待返回信息的定时器，超时将认为设备已断开连接
    QTimer* queryStateTimer;//查询位置的定时器
    int queryStateInterval; //查询位置的时间间隔
    int connectionInterval;
    bool loseConnection;

    long global_x;//x的全局绝对位置 (step)
    long global_y;//y的全局绝对位置 (step)
    long global_z;//z的全局绝对位置 (step)

public slots:
    void connectDevice(QString portName);              //连接设备
    void moveX(float x, float vel=0, bool CONTINUOUS=false);        //x轴相对运动
    void moveY(float y, float vel=0, bool CONTINUOUS=false);        //y轴相对运动
    void moveZ(float z, float vel=0, bool CONTINUOUS=false);        //z轴相对运动
    void move(float x, float y, float z, float v=0);                //任意方向相对运动
    void stop();                                                    //停止运动
    void close();                                                   //关闭设备
    void setInchingSpeed(float speed);                              //设置点动速度
    void queue(QByteArray, bool b_with_response=true);
    void send();
    void recv();
    void setVelandRes(long vel,bool res);
};

#endif // MANIPULATORMP285_H
