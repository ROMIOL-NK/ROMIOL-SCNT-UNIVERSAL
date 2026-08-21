#include "steprotateinplane.h"
using namespace std;
using namespace cv;


StepRotateInPlane::StepRotateInPlane()
{
//去往操作区
    curInjectTipRel = vector<float>(3);
    penDepth = 15;
    iniDisRatio = 1.5;
    iniAngle = 50;
    oneBigAngle = 40;
    oneSmallAngle = 10;
    rotatedAngle = 0;
    maxRotateAngle = 720;
    ANTORAD = 3.14/180;

    isPBdetected = false;
}

WORKSTEP StepRotateInPlane::run()
{
    isWorking = true;
    isPBdetected = false;
    if(isPBinPlace())
    {
        emit sgnStopTrackPB();
        nextStep = PENETRATE;
        isWorking = false;
        emit finish();
        return nextStep;
    }

    //添加处理代码: 确认吸持足够松弛，移动注射微针拨动细胞到极体在目标位置
    //开启 平面外旋转操作
    //0. 操作前提：细胞被稳定吸持在吸持针上， 细胞、吸持针、注射针三者处于同一焦平面并清晰。
    //1. 信息初始化
    //  1.1 图像处理，得到:细胞中心位置，细胞半径，注射针针尖位置。(TODO:细胞透明带的神经网络处理)
    holdPipette->find(CurrentBGR);
    if(!holdPipette->isDetected)
        DEAL_FAULT("无法找到吸持针！请手动标记吸持针！");
    currentOocyte->find(CurrentGray);
    injectPipette->find(CurrentGray);
    if(!currentOocyte->isDetected || !injectPipette->isDetected)
    {
        DEAL_FAULT("未检测到卵细胞或注射针！");
        emit sgnStopTrackPB();
        nextStep = ROTATEINPLANE;
        isWorking = false;
        emit finish();
        return nextStep;
    }


    oocytesRadius = currentOocyte->radiusZP*Pixel2Um; //单位微米。TODO: 透明带检测后，oocytesRadius*Pixel2Um
    //  1.2 以当前细胞中心位置为原点，建立相对坐标系，单位微米，更新操作过程中当前吸持针位置  (TODO:局部函数，在当前坐标系下实现 move_to操作）。
    curInjectTipRel[0] = (injectPipette->tip.x-currentOocyte->centerZP.x)*Pixel2Um;
    curInjectTipRel[1] = (injectPipette->tip.y-currentOocyte->centerZP.y)*Pixel2Um;
    curInjectTipRel[2] = 0;
    //2. 双针初始化
    //  2.1 吸持针减小负压 (TODO:加入吸持压调整的硬件和软件)
    //  2.2 注射针移动到初始位置(x轴上距离细胞边缘0.5R)
    injet_move_to_rel(iniDisRatio*oocytesRadius,0,0);
    //3. 开启旋转
    //  3.0 (TODO:若极体已出现，直接跳过平面外拨动。)
    //  3.1 移动到平面外循环旋转的初始位置（30 °）
    injet_move_to_rel(iniDisRatio*oocytesRadius,(oocytesRadius-penDepth)*sin(iniAngle*ANTORAD),0);
    //  3.2 开启循环, 循环结束条件：1.旋转140 °确认无极体。2.旋转60 ° (时极体出现)。3. 按钮急停？,(TODO:使用神经网络获取极体是否存在的信息)(TODO:确定按钮急停对操作的影响)
    int smallRotatedAngle = 0;
    rotatedAngle = 0;
    ON = true;
    // zyj 20251029 拨动时追踪卵细胞
    emit sgnInitTrackerOocyte(CurrentBGR, currentOocyte->rectZP);
    emit sgnStartPServo();
    while (ON&&(!isPBinPlace()) && rotatedAngle < maxRotateAngle)
    {
        //zyj 2025.11.03 拨动情况监测
        emit sgnStartCompare(CurrentGray(currentOocyte->rectZP), currentOocyte->rectZP);
        //      3.2.1 刺入一定深度
        inject_rot_arc_xy(iniAngle,0,0);
        //      3.2.2 沿着弧线移动，旋转细胞（40 °） (TODO:局部函数，每次旋转10 °，每次调用确定是否满足退出旋转条件）
        while (ON&&(!isPBinPlace())&&(smallRotatedAngle<oneBigAngle)&&(rotatedAngle < maxRotateAngle))
        {
            inject_rot_arc_xy(iniAngle,smallRotatedAngle,oneSmallAngle);
            smallRotatedAngle += oneSmallAngle ;
            rotatedAngle += oneSmallAngle ;
        }
        //      3.2.3 分两步回归平面外循环旋转的初始位置（30 °）
        injet_move_to_rel(iniDisRatio*oocytesRadius,(oocytesRadius-penDepth)*sin((iniAngle+smallRotatedAngle)*ANTORAD),0);
        injet_move_to_rel(iniDisRatio*oocytesRadius,(oocytesRadius-penDepth)*sin(iniAngle*ANTORAD),0);
        smallRotatedAngle = 0;
        //zyj 2025.11.03 拨动情况监测
        QThread::msleep(600);
        emit sgnCompareAndCalHoldP(CurrentGray);
    }

    //  3.3 注射针返回初始位置
    injet_move_to_rel(iniDisRatio*oocytesRadius,0,0);
    //  3.4 吸持针恢复负压
    //4. 根据不同的旋转停止条件，返回相应的状态信息：1.极体出现，开启平面内旋转。2.细胞无极体，等待更换细胞。3. 急停，等待人工介入
    //    DEAL_FAULT("请手动拨动极体到目标位置，完成后关闭该窗口！！！");//遇到需要人工处理的故障，给主线程发送信号要求处理故障
    //------------------------------------------------------步骤完成---------------------------------------------------
    emit sgnStopTrackPB();
    emit sgnStopTrackOocyte();
    emit sgnStopPServo();
    nextStep = PENETRATE;
    isWorking = false;
    emit finish();
    return nextStep;
}

bool StepRotateInPlane::isPBinPlace()
{
    if(!isPBdetected)//本次平面内选择是否找到过极体
    {
        currentOocyte->findPB(CurrentBGR);
        if(currentOocyte->isPBDetected) //当前检测到极体
        {
            isPBdetected = true;

            emit sgnInitTrackerPB(CurrentBGR, currentOocyte->rectPB);
            // currentOocyte->tracker->init(CurrentBGR, currentOocyte->rectPB);
            // currentOocyte->centerPB.x = currentOocyte->rectPB.x + 0.5*currentOocyte->rectPB.width;
            // currentOocyte->centerPB.y = currentOocyte->rectPB.y + 0.5*currentOocyte->rectPB.height;
        }
        else
            return false;
    }
    // else//极体第一次检测到之后用跟踪算法
    // {
    //     currentOocyte->tracker->update(CurrentBGR, currentOocyte->rectPB);
    //     currentOocyte->centerPB.x = currentOocyte->rectPB.x + 0.5*currentOocyte->rectPB.width;
    //     currentOocyte->centerPB.y = currentOocyte->rectPB.y + 0.5*currentOocyte->rectPB.height;
    // }
    float aim_angel;
    if((injectPipette->tip.y)>((injectPipette->ym+injectPipette->yM)/2))
        aim_angel = -30*CV_PI/180;
    else aim_angel = 30*CV_PI/180;

     qDebug() << "错误:" << atan2(currentOocyte->centerPB.y-currentOocyte->centerZP.y, currentOocyte->centerPB.x-currentOocyte->centerZP.x)<<aim_angel;
    if(abs(atan2(currentOocyte->centerPB.y-currentOocyte->centerZP.y, currentOocyte->centerPB.x-currentOocyte->centerZP.x)-aim_angel) < 15*CV_PI/180)
        return true;
    return false;
}

void StepRotateInPlane::injet_move_to_rel(float x,float y,float z,int v)
{
    moveInjWait(x-curInjectTipRel[0],y-curInjectTipRel[1],z-curInjectTipRel[2], v);
    curInjectTipRel.clear();
    curInjectTipRel.push_back(x);
    curInjectTipRel.push_back(y);
    curInjectTipRel.push_back(z);
}

void StepRotateInPlane::inject_rot_arc_xy(int iniAngle,int smallRotatedAngle,int oneSmallAngle)
{

    injet_move_to_rel((oocytesRadius-penDepth)*cos((iniAngle+smallRotatedAngle+oneSmallAngle)*3.14/180),
                      (oocytesRadius-penDepth)*sin((iniAngle+smallRotatedAngle+oneSmallAngle)*3.14/180),0,250);
}
