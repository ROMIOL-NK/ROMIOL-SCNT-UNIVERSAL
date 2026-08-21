#include "stepexpel.h"
using namespace cv;
using namespace std;
StepExpel::StepExpel(){}
StepExpel::~StepExpel(){}

WORKSTEP StepExpel::run()
{
    isWorking = true;

    //将卵母细胞释放后再吐核

    if(spitArea!=Point2f(INFINITY, INFINITY))
    {
        moveHoldWait(0,0,-areaMoveHeight, 3000);
        moveInjWait(0, 0,-areaMoveHeight, 3000);
        emit gotoArea(spitArea);
        moveHoldWait(0,0,areaMoveHeight, 3000);
        moveInjWait(0, 0,areaMoveHeight, 3000);
    }

    if(isHCI)//人机交互模式
    {
        DEAL_FAULT("请手动完成吐核操作！");

        DEAL_FAULT("吐核操作已完成！");
        //------------------------------------------------------步骤完成---------------------------------------------------
        nextStep = FETCHSOMA;
        isWorking = false;
        emit finish();
        return nextStep;
    }
    QThread::msleep(1000);
    injectPipette->find(CurrentGray);
    if(!injectPipette->isDetected)
    {DEAL_FAULT("请手动标注注射针");}
    injectPipette->setBeforeBubble(CurrentGray);
    if(injector->isLiquid)
    {
        Mat img0 = CurrentGray.clone(), img, biasImg;
        bool isExpeled = false;
        int i=1;    float step = 0.7;
        injector->syringeMotor->ori = injector->syringeMotor->getAbsPos();
        while(!isExpeled&&injector->syringeMotor->getRelPos()<1.2)
        {
            QTime time = QTime::currentTime().addMSecs(4000/i);
            img = CurrentGray.clone();
            biasImg = img0-img;
            imshow("biasImg", biasImg);
            waitKey(1);
            emit injSuck(-step, 1, false);
            ++i;            step /= 2;
            while(QTime::currentTime() < time);
        }
    }
    else
    {
        // adjInjP(injReleaseP);
        ON = true;
        if(injector->pSensor->isConnected&&injector->syringeMotor->isConnected)
        {
            double k=1e-4, tar=injReleaseP, cur=injector->pSensor->pressure, val;
            while(ON && !injectPipette->detectBubble(CurrentGray) && cur<tar*2)
            {
                if(tar>cur)
                    val = max(min(-k*(tar-cur), 0.3), -0.3);
                else
                    val = -0.02;
                emit injSuck(0, val, true);
                cur = injector->pSensor->pressure;
                QThread::msleep(100);
                if(injectPipette->detectGasColumn(CurrentGray))
                {
                    qDebug()<<"detectGasColumn";
                    break;
                }
            }
            emit injSuck(0, 0, true);
            qDebug()<<"expel stage 1 finished";
        }
        holdPipette->find(CurrentGray);
        float dy = 0;
        if(holdPipette->isDetected)
        {
            dy = (holdPipette->ym-injectPipette->yM)*Pixel2Um;
            moveInjWait(0, dy, 0, 0);
        }
        // emit moveHold(-500, 0, 0);
        moveHoldWait(-500, 0, 0, 1000);
        if(spitEdge!=Point2f(INFINITY, INFINITY))
            emit gotoArea(spitEdge);
        if(spitArea!=Point2f(INFINITY, INFINITY) && spitEdge!=Point2f(INFINITY, INFINITY))
            emit gotoArea(0.5*(spitArea+spitEdge));
        emit funcAdjInjP(injector->pSensor->pressure-4000);
        moveHoldWait(500, 0, 0, 1000);
        moveInjWait(0, -dy, 0, 0);
    }
    nextStep = FETCHSOMA;
    isWorking = false;
    emit finish();
    return nextStep;
}
