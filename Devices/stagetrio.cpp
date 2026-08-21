#include "stagetrio.h"

StageTrio::StageTrio()
{
    isConnected = false;
    trioPc = new TrioPC;
    DeviceName = "Stage Device";
    queryStateTimer = new QTimer(this);
    queryStateInterval = 200;
    //定时更新位置
    connect(queryStateTimer, &QTimer::timeout, [this](){double AxisPos=0;
        if(!trioPc->IsOpen(3240))
        {
            emit noConnection();
            return;
        }
        trioPc->GetAxisVariable(tr("MPOS"),Axis_X,AxisPos);
        raw_xpos = 1000*AxisPos;
        trioPc->GetAxisVariable(tr("MPOS"),Axis_Y,AxisPos);
        raw_ypos = 1000*AxisPos;
        emit updateXYPos();});
    inchingSpeed = 1000;
    b_same_dirX = false;
    b_same_dirY = true;
}
StageTrio::~StageTrio()
{
    delete trioPc;
}
void StageTrio::connectDevice(QString portName)
{
    if(!trioPc->IsOpen(3240))
    {
        //建立通讯链接
        trioPc->SetHostAddress(portName);
        isConnected = trioPc->Open(2,3240);
    }
    if(isConnected)
    {
        qDebug() << "Date:" << QDate::currentDate()<<tr("建立通讯链接");
        trioPc->SetAxisVariable(tr("SPEED"),Axis_X,inchingSpeed/1000);//设置 x 轴运动速度
        trioPc->SetAxisVariable(tr("SPEED"),Axis_Y,inchingSpeed/1000);//设置 y 轴运动速度
        queryStateTimer->start(queryStateInterval);
        emit startConnection();
    }
    else
    {
        emit openPortFailure();
        qDebug() << "Can't connect " + DeviceName;
    }
}

void StageTrio::moveX(float x, bool CONTINUOUS, float vel)
{
    if(vel!=0&&getInchingSpeed()!=abs(vel))
        setInchingSpeed(abs(vel));
    if(CONTINUOUS)
    {
        trioPc->SetAxisVariable(tr("SPEED"),Axis_X,abs(vel)/1000);
        if((vel<0) ^ b_same_dirX)
            trioPc->Forward(Axis_X);
        else
            trioPc->Reverse(Axis_X);
    }
    else
    {
        trioPc->MoveRel(1, b_same_dirX?x/1000:-x/1000, Axis_X);//MoveRel的单位是 mm, 指令的单位是 um
    }
}
void StageTrio::moveY(float y, bool CONTINUOUS, float vel)
{
    if(vel!=0&&getInchingSpeed()!=abs(vel))
        setInchingSpeed(abs(vel));
    if(CONTINUOUS)
    {
        trioPc->SetAxisVariable(tr("SPEED"),Axis_Y,abs(vel)/1000);
        if((vel<0) ^ b_same_dirY)
            trioPc->Forward(Axis_Y);
        else
            trioPc->Reverse(Axis_Y);
    }
    else
    {
        trioPc->MoveRel(1, b_same_dirY?y/1000:-y/1000, Axis_Y);//MoveRel的单位是 mm, 指令的单位是 um
    }
}
void StageTrio::move(float x, float y, float vel)
{
    double AxisPos=0;
    float x0, y0, x1, y1;
    x0 = getXPos();
    y0 = getYPos();

    if(vel!=0&&getInchingSpeed()!=abs(vel))
        setInchingSpeed(abs(vel));
    trioPc->MoveRel(1, b_same_dirX?x/1000:-x/1000, Axis_X);//MoveRel的单位是 mm, 指令的单位是 um
    trioPc->MoveRel(1, b_same_dirY?y/1000:-y/1000, Axis_Y);//MoveRel的单位是 mm, 指令的单位是 um

    float eps = 1;
    bool isMoveFinished = false;
    QTime time = QTime::currentTime().addMSecs(5000);
    while(!isMoveFinished && QTime::currentTime() < time)
    {
        if(!trioPc->IsOpen(3240))
        {
            emit noConnection();
            return;
        }
        trioPc->GetAxisVariable(tr("MPOS"),Axis_X,AxisPos);
        raw_xpos = 1000*AxisPos;
        trioPc->GetAxisVariable(tr("MPOS"),Axis_Y,AxisPos);
        raw_ypos = 1000*AxisPos;
        x1 = getXPos();
        y1 = getYPos();
        emit updateXYPos();

        if(abs(abs(x1-x0)-abs(x))<eps && abs(abs(y1-y0)-abs(y))<eps)
            isMoveFinished = true;
    }
    QThread::msleep(500);
}
void StageTrio::stop()
{
    trioPc->RapidStop();
}
void StageTrio::setInchingSpeed(float speed)
{
    inchingSpeed = speed;
    trioPc->SetAxisVariable(tr("SPEED"),Axis_X,speed/1000);
    trioPc->SetAxisVariable(tr("SPEED"),Axis_Y,speed/1000);
}
