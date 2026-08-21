#ifndef MANIPULATORSCIENTIFICAMOTION8_H
#define MANIPULATORSCIENTIFICAMOTION8_H

#include "Base/manipulator.h"
#include<QVector>
// Scientifica
class ManipulatorScientificaMotion8 : public Manipulator
{
    Q_OBJECT
public:
    ManipulatorScientificaMotion8(QString name="Manipulator scientifica");
    ~ManipulatorScientificaMotion8();

public:
    QSerialPort *m_port;
    QByteArray queryStateCommand;       //查询状态的指令
    QByteArray RxBuffer;                //ReadAll读到的当前返回数据
    QByteArray msg;                     //有结束标记位的接受信号
    QByteArray endflag;                 //返回信息的结尾符
    QVector<QByteArray> commandList;    //等待发送的命令列表
    QVector<bool>       responseTypes;  //命令类型（是否有返回信号）的标志列表
    QTimer* timer;          //等待返回信息的定时器，超时将认为设备已断开连接
    QTimer* queryStateTimer;//查询位置的定时器
    int queryStateInterval; //查询位置的时间间隔
    bool testMode;          //连接时发送测试信号，收到返回信息则认为建立连接
    QByteArray cobsEncode(const QByteArray &data);
    QByteArray cobsDecode(const QByteArray &encodedData);
    QByteArray convertTo4Bytes(int value);
    QByteArray convertTo8Bytes(int value);
    qint32 littleEndian4ToUInt(const QByteArray &data, int startIndex);
    long global_x;//x的全局绝对位置 (step)
    long global_y;//y的全局绝对位置 (step)
    long global_z;//z的全局绝对位置 (step)

private slots:
    void connectDevice(QString portName);              //连接设备
    void moveX(float x, float vel=0, bool CONTINUOUS=false);        //x轴相对运动
    void moveY(float y, float vel=0, bool CONTINUOUS=false);        //y轴相对运动
    void moveZ(float z, float vel=0, bool CONTINUOUS=false);        //z轴相对运动
    void move(float x, float y, float z, float v);                  //任意方向相对运动
    void stop();                                                    //停止运动
    void close();                                                   //关闭设备
    void setInchingSpeed(float speed);                              //设置点动速度
    void queue(QByteArray, bool b_with_response=true);
    void send();
    void recv();
};

#endif // MANIPULATORSCIENTIFICAMOTION8_H
