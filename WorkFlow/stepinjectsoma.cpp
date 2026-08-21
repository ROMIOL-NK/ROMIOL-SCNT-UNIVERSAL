#include "stepinjectsoma.h"
using namespace cv;
StepInjectSoma::StepInjectSoma(){}

WORKSTEP StepInjectSoma::run()
{
    isWorking = true;

    if(isHCI)//人机交互模式
    {
        DEAL_FAULT("请手动完成注入体细胞操作！");
        emit funcWithdrawInj();
        if(mode==SCNT_BATCH)
            DEAL_FAULT("是否继续对下一个细胞注核？");
        //------------------------------------------------------步骤完成---------------------------------------------------
        if(mode==SCNT_SINGLE||areAllSomasInjected)
            nextStep = RELEASEOOCYTE;
        else
            nextStep = FETCHOOCYTEFORSOMA;
        isWorking = false;
        emit finish();
        return nextStep;
    }
    //添加处理代码: 移动到操作区，吸持针移入视野，卵细胞聚焦，注射针聚焦，重新刺入，注入体细胞，撤针；移动到存储区，释放卵细胞

    //注射体细胞
    // emit funcAdjInjP(injReleaseP);

    //注射针撤出
    emit funcWithdrawInj();

    //------------------------------------------------------步骤完成---------------------------------------------------
    if(mode==SCNT_SINGLE||areAllSomasInjected)
        nextStep = RELEASEOOCYTE;
    else
        nextStep = FETCHOOCYTEFORSOMA;
    isWorking = false;
    emit finish();
    return nextStep;
}
