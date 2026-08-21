#ifndef SYRINGEMOTORVINCE_H
#define SYRINGEMOTORVINCE_H
//http://vincetech.com/?list_65/
#include "public.h"
#include "Base/syringemotor.h"
class SyringeMotorVince: public SyringeMotor
{
    Q_OBJECT
public:
    SyringeMotorVince(int ppR=123136, QString name="Syringe Motor Vince");
    ~SyringeMotorVince();

private:
    QSerialPort *m_port;
    QByteArray RxBuffer;                //ReadAll读到的当前返回数据
    QByteArray msg;                     //有结束标记位的接受信号
    QByteArray endflag;                 //返回信息的结尾符
    QByteArray begflag;                 //返回信息的开始符
    QVector<QByteArray> commandList;    //等待发送的命令列表
    QVector<bool>       responseTypes;  //命令类型（是否有返回信号）的标志列表
    QTimer* connectionTimer;            //等待返回信息的定时器，超时则清空命令列表
    QTimer* queryStateTimer;            //查询位置的定时器
    bool loseConnection;
    int queryStateInterval;             //查询位置的时间间隔
    int connectionInterval;
public slots:
    void connectDevice(QString portName);                           //连接设备
    void move(float r, float vel, bool CONTINUOUS);                 //任意方向相对运动 (单位： r, r/s)
    void suck(float disp, float vel=0, bool CONTINUOUS=false);      //注射器吸 (单位： r, r/s)
    void blow(float disp, float vel=0, bool CONTINUOUS=false);      //注射器注 (单位： r, r/s)
    void stop();                                                    //停止运动
    void close();                                                   //关闭设备
    void setInchingSpeed(float speed);                              //设置点动速度

    int convert(QByteArray msg);
    void queue(QByteArray, bool b_with_response=true);
    void send();
    void recv();
};

#endif // SYRINGEMOTORVINCE_H
