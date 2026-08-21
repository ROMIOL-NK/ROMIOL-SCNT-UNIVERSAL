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
    emit funcAdjHoldP(holdBalanceP);
    //------------------------------------------------------步骤完成---------------------------------------------------
    if(mode==SCNT_BATCH)
        nextStep = EXPEL;
    else if(mode==SCNT_SINGLE)
        nextStep = FETCHOOCYTE;
    DEAL_FAULT("是否结束自动操作？");
    isWorking = false;
    emit finish();
    return nextStep;
}

bool StepReleaseOocyte::output_Oocyte()
{
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
}
