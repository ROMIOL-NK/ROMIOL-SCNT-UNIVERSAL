#include "stepexception.h"

StepException::StepException()
{

}

WORKSTEP StepException::run()
{
    isWorking = true;

    //添加异常处理代码: 对 exceptionInfo 解析，依次处理其中的故障
    // if(exceptionInfo=="换卵时无法找到卵细胞进行吸持")
    // {
    //     DEAL_FAULT("请自行寻找卵细胞！！！");
    //     nextStep = FETCHOOCYTE;
    //     isWorking = false;
    //     emit finish();
    //     return nextStep;
    // }
    //步骤完成
    nextStep = END;
    isWorking = false;
    emit finish();
    return nextStep;
}
