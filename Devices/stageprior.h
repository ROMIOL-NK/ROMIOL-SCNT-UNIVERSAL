#ifndef STAGEPRIOR_H
#define STAGEPRIOR_H

#include "Base/stage.h"
#import <Prior.dll>named_guids
// Prior ProScan III https://www.prior.com/wp-content/themes/prior-scientific/download.php?file=226

// enum AxisNum{Axis_X=0, Axis_Y};

class StagePrior : public Stage
{
    Q_OBJECT
public:
    StagePrior();
    ~StagePrior();

private:
    QSerialPort *m_port;
    QByteArray RxBuffer;                //ReadAll读到的当前返回数据
    QByteArray msg;                     //有结束标记位的接受信号
    QByteArray endflag;                 //返回信息的结尾符
    QVector<QByteArray> commandList;    //等待发送的命令列表
    QVector<bool>       responseTypes;  //命令类型（是否有返回信号）的标志列表
    QTimer* connectionTimer;          //等待返回信息的定时器，超时将认为设备已断开连接
    bool loseConnection;
    int connectionInterval;

public slots:
    virtual void connectDevice(QString portName=strPortFocus);            //连接设备
    virtual void moveX(float x, bool CONTINUOUS=false, float vel=0);      //x 方向运动
    virtual void moveY(float y, bool CONTINUOUS=false, float vel=0);      //y 方向运动
    virtual void move(float x, float y, float vel=0);                     //x y 方向运动（点动）
    virtual void stop();                                                    //停止运动
    virtual void close();                                                   //关闭设备
    virtual void setInchingSpeed(float speed);                              //设置点动速度

    void queue(QByteArray, bool b_with_response=true);
    void send();
    void recv();

};


#endif // STAGEPRIOR_H
