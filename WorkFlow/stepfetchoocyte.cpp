#include "stepfetchoocyte.h"
using namespace cv;
StepFetchOocyte::StepFetchOocyte(){}
WORKSTEP StepFetchOocyte::run()
{
    isWorking = true;

    //判断当前是否有被吸持的卵细胞（吸持针和卵细胞检测、距离判断）
    float dx, dy, dist;
    holdPipette->find(CurrentBGR);
    if(!holdPipette->isDetected)
        DEAL_FAULT("无法找到吸持针！请手动标记吸持针！");
    // 如果没有被吸持的卵，则找到第一个卵进行吸持
    currentOocyte->find(CurrentBGR);
    if(currentOocyte->isDetected)
    {
        dx = (currentOocyte->centerZP.x - holdPipette->xM)*Pixel2Um;
        dy = (currentOocyte->centerZP.y - 0.5*(holdPipette->yM+holdPipette->ym))*Pixel2Um;
        dist = sqrt(dx*dx+dy*dy);
    }
    if(!currentOocyte->isDetected||dist>90)//根据卵中心与吸持针针尖距离判断是否吸持，若没有被吸持的卵，执行取第一个卵的操作
    {
        moveHoldWait(0,0,-areaMoveHeight, 3000);
        moveInjWait(0, 0,-areaMoveHeight, 3000);
        if(!allOocytes->oocytes.empty())
        {
            Point2f p = allOocytes->oocytes.back().centerUm;
            emit gotoArea(p);
            allOocytes->total_num--;
            allOocytes->oocytes.pop_back();
        }
        findOocyteAround();
        // /*----------------------------------------定位&检测卵细胞----------------------------------------*/
        if(!currentOocyte->isDetected)
        {
            exceptionInfo = "换卵时无法找到卵细胞进行吸持";
            nextStep = EXCEPTION;
            isWorking = false;
            emit finish();
            return nextStep;
        }
        /*--------------------------------------调整细胞&吸持针位置--------------------------------------*/
        else        //将卵细胞移动到视野中心
            emit moveStage((currentOocyte->center.x-0.65*WIDTH)*Pixel2Um, (currentOocyte->center.y-HEIGHT/2)*Pixel2Um, STAGE_VEL);
        moveHoldWait(0,0,areaMoveHeight, 3000);
        moveInjWait(0, 0,areaMoveHeight, 3000);

        if(!holdOocyte())
            nextStep = FETCHOOCYTE;
        else
            nextStep = ROTATEOUTPLANE;

        // ------------------------------------------------------步骤完成---------------------------------------------------
        isWorking = false;
        emit finish();
        return nextStep;
    }
    // ----------------------------------------------------------如果已经有被吸持的细胞，则执行换卵操作----------------------------------------------------------
    if(!currentOocyte->isDetected)
        DEAL_FAULT("无法找到卵细胞！请手动标记当前卵细胞");//TODO:自动搜寻卵
    float replace_pressure = -800;
    emit funcAdjHoldP(replace_pressure);
    // 保证细胞和吸持针在同一高度 调节气压 -500？、 移动到下一个卵附近（当前卵位置上方）、 移动注射针↑←↓
    injectPipette->find(CurrentBGR);
    if(!injectPipette->isDetected)
        DEAL_FAULT("无法检测到注射针！请手动标记注射针后关闭该窗口！");//注射针检测不到的情况下人口进行辅助标记
    moveHoldWait(0,0, -200, 1000);
    moveInjWait(0,0, -200, 1000);    //抬升微针
    // emit gotoArea(allOocytes->oocytes.back().centerUm-(currentOocyte->centerZP-Point2f(WIDTH/2, HEIGHT/2))*Pixel2Um+Point2f(0,150)); QThread::msleep(1200);//将下一个卵移动到当前卵的上方150um处
    if(!moveToNextOocyteInView())
        findNextOocyte();//TODO
    moveHoldWait(0,0, 200, 1000);
    moveInjWait(0,0, 200, 1000);     //下降微针
    float left_dist = (currentOocyte->xMZP-injectPipette->tip.x)*Pixel2Um+100;

    if(injectPipette->tip.x<=currentOocyte->xMZP) {moveInjWait(left_dist, 0, 0, 1000);}
    else left_dist = 0;

    moveInjWait(0, (currentOocyte->ymZP-injectPipette->tip.y)*Pixel2Um-20, 0, 1000);

    QDateTime startTime=  QDateTime::currentDateTime();
    QTime time = QTime::currentTime().addMSecs(5000);
    ON = true;
    while(ON && abs(holder->pSensor->pressure-replace_pressure)>100 && QTime::currentTime() < time){}
    moveInjWait((0.8*currentOocyte->centerZP.x+0.2*currentOocyte->xmZP-injectPipette->tip.x)*Pixel2Um-left_dist, 0, 0, 1000);//移动到初始拨动位置
    moveInjWait(0, 260, 0, 100);
    moveInjWait(400, 0, 0, 1000);
    // ------------------------------------------------------步骤完成---------------------------------------------------
    nextStep = ROTATEOUTPLANE;
    isWorking = false;
    // DEAL_FAULT("换卵是否完成？（去核循环）");
    emit finish();
    return nextStep;
}
bool StepFetchOocyte::moveToNextOocyteInView()
{
    bool isFound = false;
    Rect intersection;
    double interArea, unionArea, iou;
    Point tar_pos(currentOocyte->centerZP.x, currentOocyte->centerZP.y-150/Pixel2Um);
    oocytesInView->find(CurrentBGR);
    for(int i=oocytesInView->total_num-1; i>=0; --i)
    {
        intersection = oocytesInView->oocytes[i].rectZP & currentOocyte->rectZP;
        // 2. 计算交集面积。如果不相交，面积为0
        interArea = intersection.area();
        // 3. 计算并集面积：A面积 + B面积 - 交集面积
        unionArea = oocytesInView->oocytes[i].rectZP.area() + currentOocyte->rectZP.area() - interArea;
        if(unionArea<=0)
            iou = 0;
        else
            iou = interArea/unionArea;
        if(iou>0.1)
            continue;

        if(oocytesInView->oocytes[i].center.y<currentOocyte->center.y-20)
        {
            tar_pos = oocytesInView->oocytes[i].center;
            isFound = true;
            break;
        }
    }
    moveStageWait((tar_pos.x-currentOocyte->centerZP.x)*Pixel2Um, (tar_pos.y-currentOocyte->centerZP.y)*Pixel2Um+150, STAGE_VEL);
    return isFound;
}

void StepFetchOocyte::findOocyteAround()
{
    int i=1, j=0;// (1,0),(0,1), (-2,0),(0,-2),(3,0),(0,3)
    currentOocyte->find(CurrentGray);
    ON = true;
    while(!currentOocyte->isDetected && ON)
    {
        int k=abs(i)!=0?abs(i):abs(j);
        int sign = min(i,j)<0?-1:1;
        bool isX = abs(i)>0?true:false;
        for(int cnt=0; cnt<k; ++cnt)
        {
            emit moveStage(isX*sign*WIDTH_UM/2, (!isX)*sign*HEIGHT_UM/2, STAGE_VEL);
            currentOocyte->find(CurrentGray);
            if(currentOocyte->isDetected)
                return;
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
            return;
    }
}

void StepFetchOocyte::findNextOocyte()
{

}

bool StepFetchOocyte::holdOocyte()
{
    int i=0;
    bool isNear = false;
    float amount = 0.06, dx, dy, dist;
    ON = true;
    while(ON && i<5)
    {
        emit holdBlow(-amount, 0.1, false);
        QThread::msleep(1000);
        currentOocyte->find(CurrentGray);
        dx = (currentOocyte->centerZP.x - holdPipette->xM)*Pixel2Um;
        dy = (currentOocyte->centerZP.y - 0.5*(holdPipette->yM+holdPipette->ym))*Pixel2Um;
        dist = sqrt(dx*dx+dy*dy);
        if(currentOocyte->isDetected && dist <= 90)//卵细胞离开视野或远离吸持针
        {
            isNear = true;
            break;
        }
        i+=1;
        amount=0.02;
    }
    if (isNear) //如果距离满足要求 移动平台判断卵母细胞是否吸住
    {
        // 根据计算卵母细胞前后清晰度差值判断是否吸住
        currentOocyte->find(CurrentGray);
        int xM_pre,yM_pre;
        xM_pre = currentOocyte->xM;
        yM_pre = currentOocyte->yM;
        emit moveStage(50,50, STAGE_VEL); // 移动平台
        currentOocyte->find(CurrentGray);
        if (abs(xM_pre - currentOocyte->xM) <= 50 && abs(yM_pre - currentOocyte->yM) <= 50)   //吸住了
            return true;
    }
    return false;
}
