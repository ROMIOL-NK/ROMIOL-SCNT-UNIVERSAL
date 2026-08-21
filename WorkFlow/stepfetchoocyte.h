//前提准备：卵细胞地图构建完成，卵细胞位置信息已获取；吸持针在适当高度，水平位置已知
//执行操作：将卵细胞移到视野中 -> 精细定位后调整与吸持针相对位置 -> 将卵细胞吸住 -> 去操作区域
//结束状态：卵细胞被成功吸持，进入操作区域

//注：当前操作的卵细胞为 allOocytes[0]
#ifndef STEPFETCHOOCYTE_H
#define STEPFETCHOOCYTE_H
#include "workflow.h"
class StepFetchOocyte: public WorkFlow
{
    Q_OBJECT
public:
    StepFetchOocyte();

    bool ON;

    void findOocyteAround();                    //在附近寻找卵细胞
    void findNextOocyte();                      //在附近寻找卵细胞
    bool holdOocyte();                          //将卵细胞吸持住
    bool moveToNextOocyteInView();

public slots:
    WORKSTEP run();
};

#endif // STEPFETCHOOCYTE_H
