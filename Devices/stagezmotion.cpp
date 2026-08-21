#include "stagezmotion.h"

StageZMotion::StageZMotion()
{
    isConnected = false;
    DeviceName = "Stage Device ZMotion";
    queryStateTimer = new QTimer(this);
    queryStateInterval = 20;
    connect(queryStateTimer, &QTimer::timeout, [this](){
        float tempx, tempy;
        int status_xpos = ZAux_Direct_GetDpos(g_handle, 0, &tempx);
        int status_ypos = ZAux_Direct_GetDpos(g_handle, 1, &tempy);
        raw_xpos = tempx * ratio;
        raw_ypos = tempy * ratio;
        if (status_xpos==0 && status_ypos==0){emit updateXYPos();}
        });
    inchingSpeed = 1000;
    b_same_dirX = false;
    b_same_dirY = true;
    g_handle = NULL;
}
StageZMotion::~StageZMotion()
{
    delete queryStateTimer;
}
void StageZMotion::connectDevice(QString portName)
{
    char *ipaddr =(char *)"192.168.0.11"; //控制器 IP
    auto isConnected = ZAux_OpenEth(ipaddr,& g_handle); //以太网链接控制器
    if(isConnected == 0)
    {
        qDebug() << "Date:" << QDate::currentDate()<<tr("建立通讯链接");
        queryStateTimer->start(queryStateInterval);
        emit startConnection();
    }
    else
    {
        emit openPortFailure();
        qDebug() << "Can't connect " + DeviceName;
    }
}

void StageZMotion::moveX(float x, bool CONTINUOUS, float vel)
{
    x /= ratio;
    vel = vel*x/abs(x);
    ZAux_Direct_SetSpeed(g_handle, 0, vel);      // 设置轴0速度
    ZAux_Direct_Single_Move(g_handle, 0, -x);
    // int axislist[1] = {0};
    // float poslist[1] = {x};
    // ZAux_Direct_SetSpeed(g_handle,axislist[0],vel);
    // ZAux_Direct_Base(g_handle,1,axislist);
    // ZAux_Direct_Move(g_handle,1, axislist, poslist);
}
void StageZMotion::moveY(float y, bool CONTINUOUS, float vel)
{
    y /= ratio;
    vel = vel*y/abs(y);
    ZAux_Direct_SetSpeed(g_handle, 1, vel);      // 设置轴0速度
    ZAux_Direct_Single_Move(g_handle, 1, y);
    // int axislist[1] = {1}; //运动BASE 轴列表，其中轴1 为主轴
    // float poslist[1] = {y}; //运动列表，轴1-100，轴0-200
    // ZAux_Direct_SetSpeed(g_handle,axislist[0],vel); //设置插补速度100，设置在主轴上
    // ZAux_Direct_Base(g_handle,1,axislist); //选择轴列表
    // ZAux_Direct_Move(g_handle,1, axislist, poslist);
}
void StageZMotion::move(float x, float y, float vel)
{
    // float velx = vel * x / abs(x);
    // float vely = vel * y / abs(y);
    // int axislist[2] = {0, 1};
    // float poslist[2] = {x, y};
    // ZAux_Direct_SetSpeed(g_handle, axislist[0], velx);
    // ZAux_Direct_SetSpeed(g_handle, axislist[1], vely);
    x /= ratio;
    y /= ratio;
    int axislist[2] = {0, 1};
    float poslist[2] = {-x, y};
    ZAux_Direct_SetSpeed(g_handle, axislist[0], vel);
    ZAux_Direct_Base(g_handle, 2, axislist);
    ZAux_Direct_Move(g_handle, 2, axislist, poslist);
}
void StageZMotion::stop()
{
    ZAux_Direct_Rapidstop(g_handle, 0);
}
void StageZMotion::setInchingSpeed(float speed)
{
}
