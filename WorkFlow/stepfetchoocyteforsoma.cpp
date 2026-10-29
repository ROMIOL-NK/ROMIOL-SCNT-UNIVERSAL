#include "stepfetchoocyteforsoma.h"
using namespace cv;
StepFetchOocyteForSoma::StepFetchOocyteForSoma(): firstOocytePending(true), manualNextOocyteAccepted(false) {}
void StepFetchOocyteForSoma::prepareFirstOocyte()
{
    firstOocytePending = true;
}
WORKSTEP StepFetchOocyteForSoma::run()
{
    isWorking = true;

    // 取完体细胞并重新构建地图后，强制使用新地图中最下面的卵作为注核首卵。
    if(firstOocytePending)
    {
        if(allOocytes->oocytes.empty())
        {
            DEAL_FAULT("地图中没有可操作卵细胞，请重新构建地图或人工移动");
            exceptionInfo = "重新构建地图后没有找到可用于注核的卵细胞";
            nextStep = EXCEPTION;
            isWorking = false;
            emit finish();
            return nextStep;
        }

        holdPipette->find(CurrentBGR);
        if(!holdPipette->isDetected)
        {
            DEAL_FAULT("无法找到吸持针！请手动标记吸持针！");
            holdPipette->find(CurrentBGR);
        }
        if(!holdPipette->isDetected)
        {
            exceptionInfo = "注核首卵吸持前无法找到吸持针";
            nextStep = EXCEPTION;
            isWorking = false;
            emit finish();
            return nextStep;
        }

        injectPipette->find(CurrentBGR);
        if(!injectPipette->isDetected)
        {
            DEAL_FAULT("无法检测到注射针！请手动标记注射针后关闭该窗口！");
            injectPipette->find(CurrentBGR);
        }
        if(!injectPipette->isDetected)
        {
            exceptionInfo = "注核首卵吸持前无法检测到注射针";
            nextStep = EXCEPTION;
            isWorking = false;
            emit finish();
            return nextStep;
        }
        float injectOriginalY = injectPipette->tip.y;

        currentOocyte->reset();
        moveHoldWait(0,0,-areaMoveHeight, 3000);
        moveInjWait(0, 0,-areaMoveHeight, 3000);
        Point2f p = allOocytes->oocytes.back().centerUm;
        emit gotoArea(p);
        findOocyteAround();
        if(!currentOocyte->isDetected)
        {
            exceptionInfo = "无法在新地图最下方卵细胞位置重新检测到注核首卵";
            nextStep = EXCEPTION;
            isWorking = false;
            emit finish();
            return nextStep;
        }

        emit moveStage((currentOocyte->center.x-0.65*WIDTH)*Pixel2Um, (currentOocyte->center.y-HEIGHT/2)*Pixel2Um, STAGE_VEL);
        // 注射针保持抬起并先向屏幕下方移动，与吸持针下降、吸持卵细胞同时进行。
        float injectSafeY = 0.75*HEIGHT;
        float injectMoveToSafeY = (injectSafeY-injectOriginalY)*Pixel2Um;
        float injectOriginalAbsY = injector->manipulator->getAbsY();
        float injectSafeAbsY = injectOriginalAbsY+injectMoveToSafeY;
        emit moveInj(0, injectMoveToSafeY, 0, MANIP_VEL);
        moveHoldWait(0,0,areaMoveHeight, 3000);
        // 原动作会让注射针在当前纵向位置直接下降，地图中的细胞可能正好位于针下方：
        // moveInjWait(0, 0,areaMoveHeight, 3000);
        if(!holder->isConnected())
            DEAL_FAULT("请确保操作臂连接正常，完成后关闭该窗口！！！");

        bool isAutoHeld = holdOocyte();
        if(!isAutoHeld)
        {
            DEAL_FAULT("请手动吸持住卵细胞，完成后关闭该窗口！！！");//遇到需要人工处理的故障，给主线程发送信号要求处理故障
            // ------------------------------------------------------步骤完成---------------------------------------------------
        }

        QTime injectMoveTimeout = QTime::currentTime().addMSecs(5000);
        while(ON && abs(injector->manipulator->getAbsY()-injectSafeAbsY)>=0.02 && QTime::currentTime()<injectMoveTimeout)
            QThread::msleep(5);
        if(abs(injector->manipulator->getAbsY()-injectSafeAbsY)>=0.02 ||
            !moveInjWait(0, 0, areaMoveHeight, 3000))
        {
            exceptionInfo = "注核首卵注射针安全下降动作未完成";
            nextStep = EXCEPTION;
            isWorking = false;
            emit finish();
            return nextStep;
        }

        // 下降后以速度 200 返回原纵向位置，同时完成吸持压力调整，避免串行等待。
        emit moveInj(0, -injectMoveToSafeY, 0, 200);
        if(isAutoHeld)
        {
            emit funcAdjHoldP(holdPressure);
            QTime time = QTime::currentTime().addMSecs(5000);
            ON = true;
            while(ON && abs(holder->pSensor->pressure-holdPressure)>100 && QTime::currentTime() < time){}
        }

        injectMoveTimeout = QTime::currentTime().addMSecs(5000);
        while(ON && abs(injector->manipulator->getAbsY()-injectOriginalAbsY)>=0.02 && QTime::currentTime()<injectMoveTimeout)
            QThread::msleep(5);
        if(abs(injector->manipulator->getAbsY()-injectOriginalAbsY)>=0.02)
        {
            exceptionInfo = "注核首卵注射针返回原纵向位置失败";
            nextStep = EXCEPTION;
            isWorking = false;
            emit finish();
            return nextStep;
        }

        // 当前最下面的卵已经进入操作流程，从地图剩余列表移除；下一次换卵继续取新的 back()。
        if(!allOocytes->oocytes.empty())
            allOocytes->oocytes.pop_back();
        firstOocytePending = false;
        nextStep = PENETRATEFORSOMA;
        isWorking = false;
        emit finish();
        return nextStep;
    }

    // 后续换卵与去核循环保持同样结构；注射后不再因一次自动检测失败直接阻断换卵动作。
    if(!currentOocyte->isDetected)
        DEAL_FAULT("无法找到卵细胞！请手动标记当前卵细胞");
    injectPipette->find(CurrentBGR);
    if(!injectPipette->isDetected)
        DEAL_FAULT("无法检测到注射针！请手动标记注射针后关闭该窗口！");
    if(!moveHoldWait(0,0, -200, 1000) ||
        !moveInjWait(0,0, -200, 1000))    //抬升微针
    {
        exceptionInfo = "注核换卵前双针抬升动作未完成";
        nextStep = EXCEPTION;
        isWorking = false;
        emit finish();
        return nextStep;
    }

    // 优先寻找当前视野中的下一颗卵；没有时再利用重建地图移动到剩余卵附近。
    bool isNextOocyteFound = moveToNextOocyteInView();
    if(!isNextOocyteFound && allOocytes->oocytes.empty())
    {
        DEAL_FAULT("地图中没有可操作卵细胞，请重新构建地图或人工移动");
    }
    if(!isNextOocyteFound && !allOocytes->oocytes.empty())
    {
        Point2f p = allOocytes->oocytes.back().centerUm
                    -(currentOocyte->centerZP-Point2f(WIDTH/2, HEIGHT/2))*Pixel2Um
                    +Point2f(0,150);
        emit gotoArea(p);
        isNextOocyteFound = moveToNextOocyteInView();
    }

    // 自动搜索失败时保持双针抬起，允许用户手动移动平台；确认后重新使用同一视野搜索。
    while(!isNextOocyteFound)
    {
        manualNextOocyteAccepted = false;
        DEAL_FAULT("未搜索到其他细胞，请手动移动到下一个待操作细胞附近");
        if(!manualNextOocyteAccepted)
        {
            bool isHoldLowered = moveHoldWait(0,0, 200, 1000);
            bool isInjLowered = moveInjWait(0,0, 200, 1000);
            if(!isHoldLowered || !isInjLowered)
            {
                exceptionInfo = "取消注核换卵后双针下降动作未完成";
                nextStep = EXCEPTION;
            }
            else
                nextStep = RELEASEOOCYTE;
            isWorking = false;
            emit finish();
            return nextStep;
        }
        isNextOocyteFound = moveToNextOocyteInView();
    }

    if(!moveHoldWait(0,0, 200, 1000) ||
        !moveInjWait(0,0, 200, 1000))     //下降微针
    {
        exceptionInfo = "找到下一颗卵后双针下降动作未完成";
        nextStep = EXCEPTION;
        isWorking = false;
        emit finish();
        return nextStep;
    }

    if(!allOocytes->oocytes.empty())
        allOocytes->oocytes.pop_back();

    // 以下 Y+260、X+400 是换卵时把当前细胞拨下的轨迹，不属于后续极体拨动步骤。
    float left_dist = (currentOocyte->xMZP-injectPipette->tip.x)*Pixel2Um+100;

    if(injectPipette->tip.x<=currentOocyte->xMZP) {moveInjWait(left_dist, 0, 0, 1000);}
    else left_dist = 0;

    moveInjWait(0, (currentOocyte->ymZP-injectPipette->tip.y)*Pixel2Um-20, 0, 1000);

    moveInjWait((0.8*currentOocyte->centerZP.x+0.2*currentOocyte->xmZP-injectPipette->tip.x)*Pixel2Um-left_dist, 0, 0, 1000);//移动到初始拨动位置
    if(!moveInjWait(0, 260, 0, 100) ||
        !moveInjWait(200, 0, 0, 1000))
    {
        exceptionInfo = "注核换卵拨落轨迹未完成";
        nextStep = EXCEPTION;
        isWorking = false;
        emit finish();
        return nextStep;
    }

    // ------------------------------------------------------步骤完成---------------------------------------------------
    nextStep = PENETRATEFORSOMA;
    isWorking = false;
    // DEAL_FAULT("换卵是否完成？（注核循环）");
    emit finish();
    return nextStep;
}
bool StepFetchOocyteForSoma::moveToNextOocyteInView()
{
    bool isFound = false;
    Rect intersection;
    double interArea, unionArea, iou;
    Point tar_pos;
    oocytesInView->find(CurrentBGR);
    int oocyteCount = static_cast<int>(oocytesInView->oocytes.size());
    for(int i=oocyteCount-1; i>=0; --i)
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
    if(isFound)
        emit moveStage((tar_pos.x-currentOocyte->centerZP.x)*Pixel2Um, (tar_pos.y-currentOocyte->centerZP.y)*Pixel2Um+150, STAGE_VEL);
    // 原逻辑即使未找到下一颗卵也会移动平台到默认位置，已取消：
    // emit moveStage((tar_pos.x-currentOocyte->centerZP.x)*Pixel2Um, (tar_pos.y-currentOocyte->centerZP.y)*Pixel2Um+150, STAGE_VEL);
    return isFound;
}

void StepFetchOocyteForSoma::findOocyteAround()
{
    int i=1, j=0, n_max = 2;// (1,0),(0,1), (-2,0),(0,-2),(3,0),(0,3)
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
        if(abs(i)>n_max || abs(j)>n_max)
            return;
    }
}

void StepFetchOocyteForSoma::findNextOocyte()
{

}

bool StepFetchOocyteForSoma::holdOocyte()
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
        amount=0.01;
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
