#include "stepfetchsoma.h"
using namespace cv;
StepFetchSoma::StepFetchSoma()
{
    isReady = false;
    somaController = new Controller_PID;
}
StepFetchSoma::~StepFetchSoma()
{
    delete somaController;
}
WORKSTEP StepFetchSoma::run()
{
    isWorking = true;

    if(!isReady)
    {
        if(donorArea==Point2f(INFINITY, INFINITY)){DEAL_FAULT("请手动设定体细胞区域，完成后关闭该窗口！！！");}
        moveHoldWait(0,0,-areaMoveHeight, 3000);
        moveInjWait(0, 0,-areaMoveHeight, 3000);
        emit gotoArea(donorArea);
        moveHoldWait(0,0,areaMoveHeight, 3000);
        moveInjWait(0, 0,areaMoveHeight, 3000);
        if(injector->pSensor->pressure>injBalanceP*1.1)
            emit funcAdjInjP(injBalanceP);
        if(focusHigh==INFINITY||focusBottom==INFINITY||injHigh==INFINITY||injBottom==INFINITY){DEAL_FAULT("请手动设定调焦&注射针的高处&底面，完成后关闭该窗口！！！");}
        moveFocusWait(focusBottom-microscope->getAbsZ(),300);
        moveInjWait(0,0,injBottom-injector->manipulator->getAbsZ(),500);
        isReady = true;
    }
    currentSoma->find(CurrentGray);
    injectPipette->find(CurrentGray);
    if(!currentSoma->isDetected||!injectPipette->isDetected){DEAL_FAULT("请标记体细胞和注射针的位置！");}
    moveInjWait(0, 0,-areaMoveHeight, 3000);
    emit moveStage((currentSoma->xm-injectPipette->tip.x)*Pixel2Um, (currentSoma->ym-injectPipette->tip.y)*Pixel2Um, STAGE_VEL);
    moveInjWait(0, 0,areaMoveHeight, 3000);

    if(isHCI)//人机交互模式
    {
        DEAL_FAULT("请手动完成吸取体细胞操作！");
        if(mode==SCNT_BATCH)
            DEAL_FAULT("是否继续吸取体细胞？");
    }
    //添加处理代码: 移动到体细胞区域，聚焦到底面，注射针移动到底面，选择体细胞，吸取体细胞；聚焦到卵，注射针移动到卵平面；刺入，注射体细胞，撤针；
    // currentSoma->find(CurrentGray);
    // ON = true;
    // while(ON)
    // {
    //     currentSoma->track(CurrentGray.clone());
    // }

    //------------------------------------------------------步骤完成---------------------------------------------------
    if(mode==SCNT_SINGLE)
        nextStep = INJECTSOMA;
    else if(areAllSomasFetched)
    {
        isReady = false;
        nextStep = BUILDMAP;
        moveFocusWait(focusHigh-microscope->getAbsZ(),300);
        moveInjWait(0,0,injHigh-injector->manipulator->getAbsZ(),500);

        moveHoldWait(0,0,-areaMoveHeight, 3000);
        moveInjWait(0, 0,-areaMoveHeight, 3000);
        emit gotoArea(oocyteArea);
        moveHoldWait(0,0,areaMoveHeight, 3000);
        moveInjWait(0, 0,areaMoveHeight, 3000);
    }
    else
        nextStep = FETCHSOMA;
    isWorking = false;
    emit finish();
    return nextStep;
}
