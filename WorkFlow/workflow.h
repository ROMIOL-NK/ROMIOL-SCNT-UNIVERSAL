#ifndef WORKFLOW_H
#define WORKFLOW_H
#include "public.h"
#include "Objects/objects.h"
#include "Components/components.h"
#define DEAL_FAULT(msg)     {emit sgnFault(msg);isDealingFault = true;while(isDealingFault)QThread::msleep(10);}
class WorkFlow: public QObject
{
    Q_OBJECT
public:
    WorkFlow(){};
    bool isDealingFault;
public slots:
    virtual WORKSTEP run()=0;    //运行当前步骤，返回下一步要执行步骤的编号
protected:
    bool moveHoldWait(float x, float y, float z, float v);
    bool moveInjWait(float x, float y, float z, float v);
    bool moveStageWait(float x, float y, float v);
    bool moveFocusWait(float z, float v);
signals:
    void finish();
    void sgnFault(QString);

    void moveHold(float, float, float, float);      // x,y,z, vel
    void moveInj(float, float, float, float);       // x,y,z, vel
    void moveStage(float, float, float);            //移动平台   x,y,vel
    void gotoArea(cv::Point2f);
    void moveFocus(float, bool, float);

    void stopMoveHold();
    void stopMoveInj();
    void stopMoveStage();
    void stopMoveFocus();
    void stopHoldSyringe();
    void stopInjSyringe();

    void setLManipSpeed(float);                     //设置左操作臂运动速度 (um/s)
    void setRManipSpeed(float);                     //设置右操作臂运动速度 (um/s)

    void holdBlow(float, float, bool);
    void injSuck(float, float, bool);

    void funcAdjInjP(float);
    void funcWithdrawInj();
    void funcAdjHoldP(float);
    void funcHoldOocyte();                      //指派 functions 完成 holdOocyte 功能

    void moveInjX(float, float, bool);
    void moveInjY(float, float, bool);
    void moveInjZ(float, float, bool);

    void stopAllMotors();
};
#endif // WORKFLOW_H
