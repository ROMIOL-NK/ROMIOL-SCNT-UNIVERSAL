#ifndef SYRINGEMOTORGT_H
#define SYRINGEMOTORGT_H
#define AXIS 1 //轴的编号
#include "public.h"
#include "utils/gts.h"
#include "Base/syringemotor.h"

//注射器活塞电机：固高驱动器
class SyringeMotorGT : public SyringeMotor
{
    Q_OBJECT
public:
    SyringeMotorGT(int ppR=350000, QString name="Syringe Motor Googol");
    ~SyringeMotorGT();
    friend class Injector;
    friend class Holder;
public slots:
    void connectDevice(QString portName);                       //连接电机
    void move(float r, float vel, bool CONTINUOUS);             //电机运动 (单位： r, r/s)
    void suck(float disp, float vel=0, bool CONTINUOUS=false);  //注射器吸 (单位： r, r/s)
    void blow(float disp, float vel=0, bool CONTINUOUS=false);  //注射器注 (单位： r, r/s)
    void stop();
    void close();

    void moveDistance(float z);                                 //活塞运动距离 (um)
    void moveContinuous(double v);                              //活塞运动速度 (um/s)

private:
    short sRtn;
    long sts;
    TJogPrm jog;
    double prfPos, prfVel;
    QTimer* queryStateTimer;            //查询位置的定时器
    int queryStateInterval;             //查询位置的时间间隔
};

#endif // SYRINGEMOTORGT_H
