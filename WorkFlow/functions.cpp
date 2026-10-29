#include "functions.h"
using namespace std;
using namespace cv;
float co_vel = 3000;//吸持针&注射针共同上升和下降时的速度 um/s
double GetCla(Rect rect)
{
    Mat gray_img = CurrentGray.clone();
    assert(!gray_img.empty());
    Mat roi = gray_img(rect);

    double result = .0f;
    for (int i = 0; i < roi.rows; ++i){
        uchar *data = roi.ptr<uchar>(i);
        for (int j = 0; j < roi.cols - 2; ++j)
        {
            result += pow(data[j + 2] - data[j], 2);
        }
    }
    return result/roi.total();
    // assert (!gray_img.empty());
    // cv::Mat lap_image;
    // Mat roi = gray_img(rect);
    // cv::Laplacian(roi, lap_image, CV_32FC1);
    // lap_image = cv::abs(lap_image);
    // return cv::mean(lap_image)[0];
}

Functions::Functions()
{
    ON = true;
    //---------------------------------- Default Controller ----------------------------------
    holdP_Controller = new Controller_PID;// k=3e-6 [-0.01, 0.01]
    holdP_Controller->uM[0]=0.2;
    holdP_Controller->um[0]=-0.2;
    holdP_Controller->saveParas();
    holdP_Controller->qParas["kp"]=1e-5;
    holdP_Controller->qParas["kd"]=7.8e-6;
    holdP_Controller->qParas["ki"]=1.1e-7;
    holdP_Controller->qParas["delta_t"]=0.1;
    holdP_Controller->setParas(holdP_Controller->qParas);

    injGLI_Controller = new Controller_TEST;
    injGLI_Controller->uM[0]=4000;
    injGLI_Controller->um[0]=-4000;
    injGLI_Controller->U_name = "p";
    injGLI_Controller->nz=2;
    injGLI_Controller->saveParas();
    injGLI_Controller->qParas["kp"] = 2;//0.001
    injGLI_Controller->qParas["ki"] = 0.12;//1e-6
    injGLI_Controller->qParas["kd"] = 0.0025;
    injGLI_Controller->setParas(injGLI_Controller->qParas); //使用默认参数或者设置新参数

    injP_Controller = new Controller_PNEUMATIC;
    // injP_Controller = new Controller_PID;
    // injP_Controller->uM[0]=0.3;
    // injP_Controller->um[0]=-0.3;
    // injP_Controller->nz=2;
    // injP_Controller->saveParas();
    // injP_Controller->qParas["kp"]=5e-5;
    // injP_Controller->qParas["kd"]=1e-5;
    // injP_Controller->qParas["ki"]=1e-8;
    // injP_Controller->qParas["delta_t"]=0.1;
    // injP_Controller->setParas(injP_Controller->qParas);
}
Functions::~Functions()
{
    delete injGLI_Controller;
    delete holdP_Controller;
}
void Functions::adjInjGLI()
{
    ON = true;
    if(!injectPipette->isDetected)
    {
        emit finish("ADJ_INJ_GLI");
        return;
    }
    // emit sgnStartPServo();
    int tarPos = injectPipette->xm+0.6*(injectPipette->xM-injectPipette->xm);

    //设置轨迹参数
    vector<map<string, double>> trajParas(1, map<string, double>());
    trajParas[0]["setPoint"]=tarPos;
    trajParas[0]["MODE"] = TRAJ_MODE_SETPOINT;
    injGLI_Controller->setTrajParas(trajParas);
    injGLI_Controller->initialize();
    injP_Controller->initialize();
    vector<double> y, u, z;
    if(injector->pSensor->isConnected&&injector->syringeMotor->isConnected)
    while(ON)
    {        //获取当前时刻
        QTime time = QTime::currentTime().addMSecs(injGLI_Controller->getDT()*1000);
        injectPipette->findGLI(CurrentGray);
        y.clear(); y.push_back(injectPipette->xGLI);
        z.clear(); z.push_back(injector->pSensor->pressure); z.push_back(injector->syringeMotor->getAbsPos());
        injGLI_Controller->set_y(y);
        injGLI_Controller->set_z(z);
        injGLI_Controller->get_yd();
        injGLI_Controller->get_u(u);
        injGLI_Controller->step();
        if(injGLI_Controller->U_name=="p")
        {
            injBiasP = u[0];
            //内环气压控制
            y.clear(); y.push_back(injector->pSensor->pressure);
            injP_Controller->set_y(y);
            y.clear(); y.push_back(injBalanceP-injBiasP);
            injP_Controller->set_yd(y);
            injP_Controller->get_u(u);
            injP_Controller->step();
            emit injSuck(0, -u[0], true);
        }
        else if(injGLI_Controller->U_name=="pos")
        {
            emit injSuck(u[0], 0.5, false);//以 0.5r/s速度转动 u[0] r
        }
        else if(injGLI_Controller->U_name=="spd")
            emit injSuck(0, u[0], true);

        //满足结束条件，退出
        if(abs(injectPipette->xGLI-tarPos)<10)
        {
            // emit injSuck(0, 0, true);
            // break;
        }
        //等待一段时间进行下一步
        while(QTime::currentTime() < time);
    }
    // emit sgnStopPServo();
    emit injSuck(0, 0, true);
    injGLI_Controller->save_data("GLI_data.txt");
    emit finish("ADJ_INJ_GLI");
}
void Functions::adjHoldP(float tarP)
{
    holdP_Controller->initialize();
    vector<double> y, u, z;
    vector<map<string, double>> trajParas(1, map<string, double>());
    trajParas[0]["MODE"] = TRAJ_MODE_SETPOINT;
    trajParas[0]["setPoint"]=tarP;
    holdP_Controller->setTrajParas(trajParas);
    if(holder->pSensor->isConnected&&holder->syringeMotor->isConnected)
    {
        ON = true;
        // while((ON&&isHCI) || abs(holder->pSensor->pressure-tarP)>100)
        while(ON&&abs(holder->pSensor->pressure-tarP)>100)
        {
            QTime time = QTime::currentTime().addMSecs(holdP_Controller->getDT()*1000);
            y.clear(); y.push_back(holder->pSensor->pressure);
            z.clear(); z.push_back(holder->pSensor->pressure); z.push_back(holder->syringeMotor->getAbsPos());
            holdP_Controller->set_y(y);
            holdP_Controller->get_yd();
            holdP_Controller->set_z(z);
            holdP_Controller->get_u(u);
            holdP_Controller->step();
            if(holdP_Controller->U_name=="spd")
                emit holdBlow(0, u[0], true);
            else if(holdP_Controller->U_name=="pos")
                holdBlowWait(u[0], 0.1, false);
            //等待一段时间进行下一步
            while(QTime::currentTime() < time);
        }
        emit holdBlow(0, 0, true);
        ON = false;
    }
    emit finish("ADJ_HOLD_P");
}
void Functions::adjInjP(float tarP)
{
    injP_Controller->initialize();
    vector<double> y, u, z;
    vector<map<string, double>> trajParas(1, map<string, double>());
    trajParas[0]["MODE"] = TRAJ_MODE_SETPOINT;
    trajParas[0]["setPoint"]=tarP;
    injP_Controller->setTrajParas(trajParas);
    if(injector->pSensor->isConnected&&injector->syringeMotor->isConnected)
    {
        ON = true;
        // while((ON&&isHCI) || abs(injector->pSensor->pressure-tarP)>100)
        while(ON&&abs(injector->pSensor->pressure-tarP)>100)
        {
            QTime time = QTime::currentTime().addMSecs(injP_Controller->getDT()*1000);
            y.clear(); y.push_back(injector->pSensor->pressure);
            z.clear(); z.push_back(injector->pSensor->pressure); z.push_back(injector->syringeMotor->getAbsPos());
            injP_Controller->set_y(y);
            injP_Controller->get_yd();
            injP_Controller->set_z(z);
            injP_Controller->get_u(u);
            injP_Controller->step();
            emit injSuck(0, -u[0], true);
            //等待一段时间进行下一步
            while(QTime::currentTime() < time);
        }
        emit injSuck(0, 0, true);
        ON = false;
    }
    emit finish("ADJ_INJ_P");
}
void Functions::withdrawInj()
{
    moveInjWait(180, 0, 0, 250);
    emit finish("WITHDRAW_INJ");
}
void Functions::co_up()
{
    emit moveHoldZ(-petriDishHeight, co_vel, false);
    emit moveInjZ(-petriDishHeight, co_vel, false);
    emit finish("CO_UP");
}
void Functions::co_down()
{
    emit moveHoldZ(petriDishHeight, co_vel, false);
    emit moveInjZ(petriDishHeight, co_vel, false);
    emit finish("CO_DOWN");
}
void Functions::nextOocyte()
{
    if(allOocytes->somas.empty())
    {
        emit finish("NEXT_OOCYTE");
        return;
    }
    emit gotoArea(allOocytes->oocytes.back().centerUm);
    allOocytes->oocytes.pop_back();
    emit finish("NEXT_OOCYTE");
}
void Functions::findOocyte()
{
    ON = true;
    int i=1, j=0;// (1,0),(0,1), (-2,0),(0,-2),(3,0),(0,3)
    currentOocyte->find(CurrentGray);
    float dt = 2000*(WIDTH_UM/2+HEIGHT_UM/2)/STAGE_VEL;//两倍时间
    while(!currentOocyte->isDetected && ON)
    {
        int k=abs(i)!=0?abs(i):abs(j);
        int sign = min(i,j)<0?-1:1;
        bool isX = abs(i)>0?true:false;
        for(int cnt=0; cnt<k; ++cnt)
        {
            emit moveStage(isX*sign*WIDTH_UM/2, (!isX)*sign*HEIGHT_UM/2, STAGE_VEL);
            currentOocyte->find(CurrentGray);
            if(currentOocyte->isDetected || !ON)
                break;
        }
        if(i!=0)
        {
            j=i;
            i=0;
        }
        else
        {
            i=pow(-1, abs(j))*(abs(j)+1);
            j=0;
        }
        if(abs(i)>5 || abs(j)>5)
            break;
    }
    if(currentOocyte->isDetected)
        emit moveStage((currentOocyte->center.x-0.65*WIDTH)*Pixel2Um, (currentOocyte->center.y-HEIGHT/2)*Pixel2Um, STAGE_VEL);
    emit finish("FIND_OOCYTE");
    ON = false;
}
void Functions::holdOocyte()
{
    adjHoldP(holdPressure);
    emit finish("HOLD_OOCYTE");
}
void Functions::releaseOocyte()
{
    adjHoldP(holdBalanceP);
    emit finish("RELEASE_OOCYTE");
}
void Functions::focus(QString s)
{
    if (!microscope->isFocusDeviceConnected())
    {
        qDebug()<<"调焦设备未连接！！！";
        emit finish("FOCUS_OOCYTE");emit finish("FOCUS_HOLD");emit finish("FOCUS_INJ");return;
    }
    Rect objectRect;
    if(s=="Oocyte")
    {
        currentOocyte->find(CurrentGray);
        if(!currentOocyte->isDetected)
        {emit finish("FOCUS_OOCYTE");return;}
        objectRect = Rect(currentOocyte->xm,currentOocyte->ym,currentOocyte->xM - currentOocyte->xm,currentOocyte->yM - currentOocyte->ym);
    }
    else if(s=="Hold")
    {
        holdPipette->find(CurrentGray);
        if(!holdPipette->isDetected)
        {emit finish("FOCUS_HOLD");return;}
        objectRect = Rect(holdPipette->xm,holdPipette->ym,holdPipette->xM - holdPipette->xm,holdPipette->yM - holdPipette->ym);
    }
    else if(s=="Inj")
    {
        injectPipette->find(CurrentGray);
        if(!injectPipette->isDetected)
        {emit finish("FOCUS_INJ");return;}
        objectRect = Rect(injectPipette->xm,injectPipette->ym,injectPipette->xM - injectPipette->xm,injectPipette->yM - injectPipette->ym);
    }

    double Focus_vel=-30;  //调焦速度um/s
    int sleepTime = 200;
    //double direction=-15; //正方向
    double Oocyte_cla;
    double definite_direction[5];
    Oocyte_cla = GetCla(objectRect);
    definite_direction[0]=Oocyte_cla;
    emit moveFocus(0, true, Focus_vel);
    int num=0;
    for(int i=1;i<5;i++)
    {
        QThread::msleep(sleepTime);
        definite_direction[i]=GetCla(objectRect);
        // qDebug()<<definite_direction[i];
        if (definite_direction[i]>definite_direction[0])
        {
            num=num+1;
        }
    }
    emit moveFocus(0, true, 0);
    QThread::msleep(sleepTime);
    if (num>=2)
    {
        Focus_vel=Focus_vel;//方向不变
    }
    else
    {
        Focus_vel=-Focus_vel;//方向改变
    }
    double search_top[5];
    emit moveFocus(0, true, Focus_vel);
    Oocyte_cla=GetCla(objectRect);
    double Oocyte_cla_now;
    Oocyte_cla_now=Oocyte_cla;

    int err=0; //防止死循环
    while(1)   //寻找最大值 每一秒为一帧 具体速度可调
    {
        QThread::msleep(sleepTime);
        Oocyte_cla_now=GetCla(objectRect);
        // qDebug()<<Oocyte_cla;
        if (Oocyte_cla_now>Oocyte_cla)
        {
            Oocyte_cla=Oocyte_cla_now;
        }
        else
        {
            search_top[0]=Oocyte_cla_now;
            double top;
            top=search_top[0];
            for(int i=1;i<5;i++)
            {
                QThread::msleep(sleepTime);
                search_top[i]=GetCla(objectRect);
                if (search_top[i]>=top)
                {
                    top=search_top[i];
                }
            }
            if ((top-search_top[0])/search_top[0] > 0.05) //当前不是最高峰 继续搜索
            {
                continue;
            }
            else  //是最高峰 在当前五帧进行搜索
            {
                emit moveFocus(0, true, 0);
                QThread::msleep(sleepTime);
                emit moveFocus(0, true, -Focus_vel);
                Oocyte_cla=GetCla(objectRect);

                for(int i=1;i<5;i++)
                {
                    if (abs(top-Oocyte_cla)/Oocyte_cla <= 0.015)
                    {
                        break;
                    }
                    QThread::msleep(sleepTime);
                    Oocyte_cla=GetCla(objectRect);
                }
                emit moveFocus(0, true, 0);
                QThread::msleep(sleepTime);
                break;
            }
        }
        err=err+1;
        if (err>=500) //防止死循环
        {
            emit moveFocus(0, true, 0);
            QThread::msleep(sleepTime);
            break;
        }
    }
    if(s=="Oocyte")
        emit finish("FOCUS_OOCYTE");
    else if(s=="Hold")
        emit finish("FOCUS_HOLD");
    else if(s=="Inj")
        emit finish("FOCUS_INJ");
}

void Functions::holdBlowWait(float x, float v, bool b)
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
}
bool Functions::moveHoldWait(float x, float y, float z, float v)
{
    float x0, y0, z0, x1, y1, z1, dx, dy, dz;
    x0 = holder->manipulator->getAbsX();
    y0 = holder->manipulator->getAbsY();
    z0 = holder->manipulator->getAbsZ();
    emit moveHold(x,y,z,v);
    bool isMoveFinished = false;
    QDateTime startTime=  QDateTime::currentDateTime();
    QTime time = QTime::currentTime().addMSecs(3000);
    while(!isMoveFinished && QTime::currentTime() < time)
    {
        x1 = holder->manipulator->getAbsX();
        y1 = holder->manipulator->getAbsY();
        z1 = holder->manipulator->getAbsZ();
        dx = x1-x0;
        dy = y1-y0;
        dz = z1-z0;
        qDebug()<< "dx = "<<dx<<"; dy = "<<dy<<"; dz = "<<dz;
        float eps = 1;      //TODO:不同设备误差设置需要修改
        if(abs(dx-x)<eps && abs(dy-y)<eps && abs(dz-z)<eps)
            isMoveFinished = true;
    }
    return isMoveFinished;
}
bool Functions::moveInjWait(float x, float y, float z, float v)
{
    float x0, y0, z0, x1, y1, z1, dx, dy, dz;
    x0 = injector->manipulator->getAbsX();
    y0 = injector->manipulator->getAbsY();
    z0 = injector->manipulator->getAbsZ();
    emit moveInj(x,y,z,v);
    bool isMoveFinished = false;
    QDateTime startTime=  QDateTime::currentDateTime();
    QTime time = QTime::currentTime().addMSecs(5000);
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
    // qDebug()<<dx-x<<dy-y<<dz-z;
    return isMoveFinished;
}
void Functions::test()
{
    // holdBlowWait(0.03,  0.2, false);
    // holdBlowWait(-0.03, 0.2, false);
    moveInjWait(100, 0, 0, 0);
    moveInjWait(0, 100, 0, 0);
    moveInjWait(-100, 0, 0, 0);
    moveInjWait(0, -100, 0, 0);
    moveInjWait(-100, 0, 100, 0);
    moveInjWait(100, 0, -100, 0);
    emit finish("TEST");
}

void Functions::run(QString str)
{
    if(str=="ADJ_INJ_GLI")
        adjInjGLI();
    else if(str=="ADJ_HOLD_P")
        adjHoldP(tarHoldP);
    else if(str=="ADJ_INJ_P")
        adjInjP(tarInjP);
    else if(str=="WITHDRAW_INJ")
        withdrawInj();
    else if(str=="CO_UP")
        co_up();
    else if(str=="CO_DOWN")
        co_down();
    else if(str=="NEXT_OOCYTE")
        nextOocyte();
    else if(str=="FIND_OOCYTE")
        findOocyte();
    else if(str=="HOLD_OOCYTE")
        holdOocyte();
    else if(str=="RELEASE_OOCYTE")
        releaseOocyte();
    else if(str=="FOCUS_OOCYTE")
        focus("Oocyte");
    else if(str=="FOCUS_HOLD")
        focus("Hold");
    else if(str=="FOCUS_INJ")
        focus("Inj");
    else if(str=="TEST")
        test();
}


