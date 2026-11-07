#include "workflow.h"
using namespace cv;
bool WorkFlow::moveHoldWait(float x, float y, float z, float v)
{
    float x0, y0, z0, x1, y1, z1, dx, dy, dz;
    x0 = holder->manipulator->getAbsX();
    y0 = holder->manipulator->getAbsY();
    z0 = holder->manipulator->getAbsZ();
    emit moveHold(x,y,z,v);
    bool isMoveFinished = false;
    QDateTime startTime=  QDateTime::currentDateTime();
    QTime time = QTime::currentTime().addMSecs(5000);
    while(!isMoveFinished && QTime::currentTime() < time)
    {
        x1 = holder->manipulator->getAbsX();
        y1 = holder->manipulator->getAbsY();
        z1 = holder->manipulator->getAbsZ();
        dx = x1-x0;
        dy = y1-y0;
        dz = z1-z0;
        float eps = 1;      //TODO:不同设备误差设置需要修改
        if(abs(dx-x)<eps && abs(dy-y)<eps && abs(dz-z)<eps)
            isMoveFinished = true;
        // qDebug()<< "dx = "<<dx<<"; dy = "<<dy<<"; dz = "<<dz;
    }
    return isMoveFinished;
}
bool WorkFlow::moveInjWait(float x, float y, float z, float v)
{
    float x0, y0, z0, x1, y1, z1, dx, dy, dz;
    x0 = injector->manipulator->getAbsX();
    y0 = injector->manipulator->getAbsY();
    z0 = injector->manipulator->getAbsZ();
    emit moveInj(x,y,z,v);
    bool isMoveFinished = false;
    QDateTime startTime=  QDateTime::currentDateTime();
    QTime time = QTime::currentTime().addMSecs(1000);
    while(!isMoveFinished && QTime::currentTime() < time)
    {
        x1 = injector->manipulator->getAbsX();
        y1 = injector->manipulator->getAbsY();
        z1 = injector->manipulator->getAbsZ();
        dx = x1-x0;
        dy = y1-y0;
        dz = z1-z0;
        // qDebug()<< "dx = "<<dx<<"; dy = "<<dy<<"; dz = "<<dz<< "x = "<<x<<"; y = "<<y<<"; z = "<<z;
        float eps = 0.02;      //TODO:不同设备误差设置需要修改
        if(abs(dx-x)<eps && abs(dy-y)<eps && abs(dz-z)<eps)
            isMoveFinished = true;
    }
    qDebug()<<dx-x<<dy-y<<dz-z;
    return isMoveFinished;
}
bool WorkFlow::moveStageWait(float x, float y, float v)
{
    float x0, y0, x1, y1, dx, dy;
    x0 = microscope->stage->getXPos();
    y0 = microscope->stage->getYPos();
    emit moveStage(x,y,v);
    bool isMoveFinished = false;
    QDateTime startTime=  QDateTime::currentDateTime();
    QTime time = QTime::currentTime().addMSecs(5000);
    while(!isMoveFinished && QTime::currentTime() < time)
    {
        x1 = microscope->stage->getXPos();
        y1 = microscope->stage->getYPos();
        dx = x1-x0;
        dy = y1-y0;
        float eps = 1;      //TODO:不同设备误差设置需要修改
        if(abs(dx-x)<eps && abs(dy-y)<eps)
        {
            isMoveFinished = true;
            // qDebug()<< "dx = "<<dx<<"; dy = "<<dy<< "x = "<<x<<"; y = "<<y;
        }
    }
    // if(!isMoveFinished)
    //     qDebug()<< "Time up! dx = "<<dx<<"; dy = "<<dy<< "x = "<<x<<"; y = "<<y;
    QThread::msleep(100);
    // qDebug()<<dx-x<<dy-y;
    return isMoveFinished;
}
bool WorkFlow::moveFocusWait(float z, float v)
{
    float z0, z1, dz;
    z0 = microscope->focusdevice->getZPos();
    emit moveFocus(z, false, v);
    bool isMoveFinished = false;
    QDateTime startTime=  QDateTime::currentDateTime();
    QTime time = QTime::currentTime().addMSecs(5000);
    while(!isMoveFinished && QTime::currentTime() < time)
    {
        z1 = microscope->focusdevice->getZPos();
        dz = z1-z0;
        float eps = 0.1;      //TODO:不同设备误差设置需要修改
        if(abs(dz-z)<eps)
            isMoveFinished = true;
    }
    return isMoveFinished;
}
bool WorkFlow::holdBlowWait(float x, float v, bool b)
{
    float r0,r, dr;
    r0 = holder->syringeMotor->getAbsPos();
    emit holdBlow(x, v, b);
    bool isMoveFinished = false;
    QDateTime startTime=  QDateTime::currentDateTime();
    QTime time = QTime::currentTime().addMSecs(3000);
    while(!isMoveFinished && QTime::currentTime() < time)
    {
        r = holder->syringeMotor->getAbsPos();
        dr = abs(r-r0);
        float eps = 0.00001;      //TODO:不同设备误差设置需要修改
        if(abs(dr-abs(x))<eps)
            isMoveFinished = true;
    }
    return isMoveFinished;
}