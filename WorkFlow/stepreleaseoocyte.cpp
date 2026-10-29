#include "stepreleaseoocyte.h"
using namespace cv;
StepReleaseOocyte::StepReleaseOocyte() {}
WORKSTEP StepReleaseOocyte::run()
{
    isWorking = true;
    if (!output_Oocyte()) {DEAL_FAULT("请手动释放卵母细胞，完成后关闭该窗口！！！");}

    moveHoldWait(0,0,-areaMoveHeight, 3000);
    moveInjWait(0, 0,-areaMoveHeight, 3000);
    emit moveStage(0, -1500, STAGE_VEL);
    moveHoldWait(0,0,areaMoveHeight, 3000);
    moveInjWait(0, 0,areaMoveHeight, 3000);
    // 原逻辑使用 Functions::adjHoldP，速度上限过大，容易在 +3300 Pa 目标处严重过冲：
    emit funcAdjHoldP(holdBalanceP);
    // emit setHoldPressureTarget(holdBalanceP);
    //------------------------------------------------------步骤完成---------------------------------------------------
    nextStep = EXPEL;
    DEAL_FAULT("是否结束自动操作？");
    isWorking = false;
    emit finish();
    return nextStep;
}

bool StepReleaseOocyte::output_Oocyte()
{
    const float releaseSpeed = 0.0020f;
    const float releaseDistance = 5.0f;
    const float fastReleasePressure = -800.0f;
    const float maxReleasePressure = 1000.0f;
    const int checkInterval = 100;
    const int servoPressureTimeout = 15000;
    const int slowReleaseTimeout = 30000;

    ON = true;
    emit sgnStopPServo();
    emit stopHoldSyringe();

    // 直接使用压力伺服调到 -800 Pa，避免开环高速运动因传感器滞后而过冲。
    if(abs(holder->pSensor->pressure-fastReleasePressure) > 100)
    {
        // emit setHoldPressureTarget(fastReleasePressure);
        QTime timeout = QTime::currentTime().addMSecs(servoPressureTimeout);
        while(ON && abs(holder->pSensor->pressure-fastReleasePressure) > 100 && QTime::currentTime() < timeout)
            QThread::msleep(50);
        emit sgnStopPServo();
        emit stopHoldSyringe();
        if(!ON || abs(holder->pSensor->pressure-fastReleasePressure) > 100)
            return false;
    }
    emit stopHoldSyringe();
    QThread::msleep(200);

    // 穿刺、注射和撤针后细胞位置可能发生变化，释放检测前重新建立当前细胞框。
    currentOocyte->find(CurrentBGR);
    if(!currentOocyte->isDetected)
        return true;
    float detectedDx = (currentOocyte->centerZP.x-holdPipette->xM)*Pixel2Um;
    float detectedDy = (currentOocyte->centerZP.y-0.5*(holdPipette->yM+holdPipette->ym))*Pixel2Um;
    if(sqrt(detectedDx*detectedDx+detectedDy*detectedDy) > 90)
        return true;

    float holdTipY = 0.5*(holdPipette->yM + holdPipette->ym);
    float initialDist = sqrt(detectedDx*detectedDx + detectedDy*detectedDy);
    int detachedCount = 0;

    // 以固定低速连续释放，每 100 ms 检测一次，脱落或达到压力上限时立即停止。
    QTime slowTimeout = QTime::currentTime().addMSecs(slowReleaseTimeout);
    emit holdBlow(0, releaseSpeed, true);
    while(ON && QTime::currentTime() < slowTimeout)
    {
        QThread::msleep(checkInterval);
        if(holder->pSensor->pressure >= maxReleasePressure)
        {
            emit stopHoldSyringe();
            return false;
        }

        if(CurrentBGR.empty())
        {
            emit stopHoldSyringe();
            return false;
        }
        currentOocyte->find(CurrentBGR);
        if(!currentOocyte->isDetected)
        {
            emit stopHoldSyringe();
            return true;
        }

        float dx = (currentOocyte->centerZP.x - holdPipette->xM)*Pixel2Um;
        float dy = (currentOocyte->centerZP.y - holdTipY)*Pixel2Um;
        float dist = sqrt(dx*dx + dy*dy);

        if(dist > 90 || dist-initialDist > releaseDistance)
            detachedCount++;
        else
            detachedCount = 0;

        // 连续两帧确认已离开针口，避免检测框单帧抖动造成误判。
        if(detachedCount >= 1)
        {
            emit stopHoldSyringe();
            return true;
        }
    }
    emit stopHoldSyringe();
    return false;

    /*
     * 原释放逻辑保留，若现场效果不合适，可删除上面的新逻辑并恢复下面代码：
    int i=0;
    float amount = 0.04;
    ON = true;
    while(ON)
    {
        emit holdBlow(amount, 0, false);
        QThread::sleep(1);
        currentOocyte->find(CurrentGray);
        holdPipette->find(CurrentGray);
        float dx, dy, dist;
        dx = (currentOocyte->centerZP.x - holdPipette->xM)*Pixel2Um;
        dy = (currentOocyte->centerZP.y - 0.5*(holdPipette->yM+holdPipette->ym))*Pixel2Um;
        dist = sqrt(dx*dx+dy*dy);
        if(!currentOocyte->isDetected || dist > 90)//卵细胞离开视野或远离吸持针
            return true;
        i+=1;
        amount=0.01;
        if(i>=5)
            return false;
    }
    return false;
    */
}
