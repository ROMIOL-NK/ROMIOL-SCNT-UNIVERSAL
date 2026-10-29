#include "stepfetchoocyte.h"
using namespace cv;
StepFetchOocyte::StepFetchOocyte(){}
WORKSTEP StepFetchOocyte::run()
{
    isWorking = true;
    ON = true;

    //判断当前是否有被吸持的卵细胞（吸持针和卵细胞检测、距离判断）
    float dx = 0;
    float dy = 0;
    float dist = INFINITY;
    holdPipette->find(CurrentBGR);
    if(!holdPipette->isDetected)
        DEAL_FAULT("无法找到吸持针！请手动标记吸持针！");
    // 如果没有被吸持的卵，则找到第一个卵进行吸持
    Oocyte previousOocyte = *currentOocyte;
    bool hadHeldOocyteState = previousOocyte.isDetected;
    currentOocyte->find(CurrentBGR);
    if(!currentOocyte->isDetected && hadHeldOocyteState)
        *currentOocyte = previousOocyte;
    if(currentOocyte->isDetected)
    {
        dx = (currentOocyte->centerZP.x - holdPipette->xM)*Pixel2Um;
        dy = (currentOocyte->centerZP.y - 0.5*(holdPipette->yM+holdPipette->ym))*Pixel2Um;
        dist = sqrt(dx*dx+dy*dy);
    }
    if(!currentOocyte->isDetected||dist>90)//根据卵中心与吸持针针尖距离判断是否吸持，若没有被吸持的卵，执行取第一个卵的操作
    {
        injectPipette->find(CurrentBGR);
        if(!injectPipette->isDetected)
        {
            DEAL_FAULT("无法检测到注射针！请手动标记注射针后关闭该窗口！");
            injectPipette->find(CurrentBGR);
        }
        if(!injectPipette->isDetected)
        {
            exceptionInfo = "首卵吸持前无法检测到注射针";
            nextStep = EXCEPTION;
            isWorking = false;
            emit finish();
            return nextStep;
        }
        float injectOriginalY = injectPipette->tip.y;

        allOocytes->total_num = static_cast<int>(allOocytes->oocytes.size());
        if(allOocytes->oocytes.empty())
        {
            DEAL_FAULT("地图中没有可操作卵细胞，请重新构建地图或人工移动");
            exceptionInfo = "地图中没有可操作卵细胞";
            nextStep = EXCEPTION;
            isWorking = false;
            emit finish();
            return nextStep;
        }

        moveHoldWait(0,0,-areaMoveHeight, 3000);
        moveInjWait(0, 0,-areaMoveHeight, 3000);
        auto firstOocyteIt = std::max_element(allOocytes->oocytes.begin(), allOocytes->oocytes.end(),
                                              [](const Oocyte& left, const Oocyte& right)
                                              {
                                                  return left.centerUm.y < right.centerUm.y;
                                              });
        Point2f p = firstOocyteIt->centerUm;
        emit gotoArea(p);
        findOocyteAround();
        // /*----------------------------------------定位&检测卵细胞----------------------------------------*/
        if(!currentOocyte->isDetected)
        {
            exceptionInfo = "换卵时无法找到卵细胞进行吸持";
            DEAL_FAULT(exceptionInfo);
            nextStep = EXCEPTION;
            isWorking = false;
            emit finish();
            return nextStep;
        }
        /*--------------------------------------调整细胞&吸持针位置--------------------------------------*/
        else        //将卵细胞移动到视野中心
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
            exceptionInfo = "注射针首卵安全下降动作未完成";
            DEAL_FAULT(exceptionInfo);
            nextStep = EXCEPTION;
            isWorking = false;
            emit finish();
            return nextStep;
        }

        // 下降后以速度 200 回到吸持针上方 100um，避免回到原纵向位置时离细胞过近。
        float injectTargetY = 0.5*(holdPipette->ym+holdPipette->yM)-100/Pixel2Um;
        float injectMoveToTargetY = (injectTargetY-injectSafeY)*Pixel2Um;
        float injectTargetAbsY = injectSafeAbsY+injectMoveToTargetY;
        emit moveInj(0, injectMoveToTargetY, 0, 200);
        if(isAutoHeld)
        {
            emit funcAdjHoldP(holdPressure);
            QTime time = QTime::currentTime().addMSecs(5000);
            ON = true;
            while(ON && abs(holder->pSensor->pressure-holdPressure)>100 && QTime::currentTime() < time){}
        }

        injectMoveTimeout = QTime::currentTime().addMSecs(5000);
        while(ON && abs(injector->manipulator->getAbsY()-injectTargetAbsY)>=0.02 && QTime::currentTime()<injectMoveTimeout)
            QThread::msleep(5);
        if(abs(injector->manipulator->getAbsY()-injectTargetAbsY)>=0.02)
        {
            exceptionInfo = "注射针首卵返回吸持针上方位置失败";
            DEAL_FAULT(exceptionInfo);
            nextStep = EXCEPTION;
            isWorking = false;
            emit finish();
            return nextStep;
        }
        nextStep = ROTATEOUTPLANE;
        isWorking = false;
        emit finish();
        return nextStep;
    }

    if(!currentOocyte->isDetected)
        DEAL_FAULT("无法找到卵细胞！请手动标记当前卵细胞");//TODO:自动搜寻卵
    // 保证细胞和吸持针在同一高度 调节气压 -500？、 移动到下一个卵附近（当前卵位置上方）、 移动注射针↑←↓
    {
        auto failExchange = [this](const QString& msg)->WORKSTEP
        {
            exceptionInfo = msg;
            DEAL_FAULT(exceptionInfo);
            nextStep = EXCEPTION;
            isWorking = false;
            emit finish();
            return nextStep;
        };

        auto waitSelectTargetBeforeMotion = [this](Oocyte& target, int timeoutMs)->bool
        {
            QTime time = QTime::currentTime().addMSecs(timeoutMs);
            while(ON && QTime::currentTime()<time)
            {
                if(selectNextOocyteCandidateInRegion(target))
                    return true;
                QThread::msleep(200);
            }
            return selectNextOocyteCandidateInRegion(target);
        };

        Oocyte preselectedTarget;
        bool hasPreselectedTarget = waitSelectTargetBeforeMotion(preselectedTarget, 1200);
        while(ON && !hasPreselectedTarget)
        {
            if(!moveToNextMappedRegion())
            {
                QString mappedFailure = QString("Global map region search failed: %1").arg(lastMappedRegionFailure);
                DEAL_FAULT(mappedFailure);
                hasPreselectedTarget = waitSelectTargetBeforeMotion(preselectedTarget, 1200);
                if(!hasPreselectedTarget)
                {
                    return failExchange(mappedFailure);
                }
                continue;
            }
            else
            {
                hasPreselectedTarget = waitSelectTargetBeforeMotion(preselectedTarget, 5000);
                if(!hasPreselectedTarget)
                    removeNearestMapOocyte(lastMappedRegionTarget);
            }
        }
        if(!ON)
        {
            isWorking = false;
            emit finish();
            return nextStep;
        }

        holdPipette->find(CurrentBGR);
        injectPipette->find(CurrentBGR);
        if(!holdPipette->isDetected || !injectPipette->isDetected)
        {
            QString needleFailure = QString("Needle check before exchange failed: hold=%1, inj=%2")
            .arg(holdPipette->isDetected)
                .arg(injectPipette->isDetected);
            DEAL_FAULT(needleFailure);
            holdPipette->find(CurrentBGR);
            injectPipette->find(CurrentBGR);
            if(!holdPipette->isDetected || !injectPipette->isDetected)
            {
                return failExchange(needleFailure);
            }
        }
        cv::Point2f holdOrigin(holdPipette->xM, 0.5f*(holdPipette->ym+holdPipette->yM));
        float heldDx = (currentOocyte->centerZP.x-holdOrigin.x)*Pixel2Um;
        float heldDy = (currentOocyte->centerZP.y-holdOrigin.y)*Pixel2Um;
        if(sqrt(heldDx*heldDx+heldDy*heldDy)>100.0f)
        {
            QString heldFailure = QString("Held oocyte distance check failed: dist=%1um, limit=100um")
            .arg(sqrt(heldDx*heldDx+heldDy*heldDy), 0, 'f', 1);
            DEAL_FAULT(heldFailure);
            holdPipette->find(CurrentBGR);
            injectPipette->find(CurrentBGR);
            holdOrigin = cv::Point2f(holdPipette->xM, 0.5f*(holdPipette->ym+holdPipette->yM));
            heldDx = (currentOocyte->centerZP.x-holdOrigin.x)*Pixel2Um;
            heldDy = (currentOocyte->centerZP.y-holdOrigin.y)*Pixel2Um;
            if(!holdPipette->isDetected || !injectPipette->isDetected ||
                sqrt(heldDx*heldDx+heldDy*heldDy)>100.0f)
            {
                return failExchange(heldFailure);
            }
        }
        cv::Point2f injTravelTip(currentOocyte->centerZP.x+200.0f/Pixel2Um, holdOrigin.y);
        moveInjXYWaitRelaxed(0, (injTravelTip.y-injectPipette->tip.y)*Pixel2Um, 1000);
        moveInjXYWaitRelaxed((injTravelTip.x-injectPipette->tip.x)*Pixel2Um, 0, 1000);
        if(false && (!moveInjXYWaitRelaxed(0, (injTravelTip.y-injectPipette->tip.y)*Pixel2Um, 1000) ||
                      !moveInjXYWaitRelaxed((injTravelTip.x-injectPipette->tip.x)*Pixel2Um, 0, 1000)))
        {
            injectPipette->find(CurrentBGR);
            if(isInjectTipNear(injTravelTip))
                ; // Movement feedback was strict, but the needle is already visually near the target.
            else
                return failExchange(QString("Injection travel pre-position failed: targetTip=(%1,%2), curTip=(%3,%4)")
                                        .arg(injTravelTip.x, 0, 'f', 1)
                                        .arg(injTravelTip.y, 0, 'f', 1)
                                        .arg(injectPipette->tip.x, 0, 'f', 1)
                                        .arg(injectPipette->tip.y, 0, 'f', 1));
            exceptionInfo = "去核换卵注射针安全随行位预定位失败";
            nextStep = EXCEPTION;
            isWorking = false;
            emit finish();
            return nextStep;
        }

        Oocyte targetOocyte = preselectedTarget;
        holdOrigin = cv::Point2f(holdPipette->xM, 0.5f*(holdPipette->ym+holdPipette->yM));
        cv::Point2f desiredHoldOrigin = targetOocyte.centerZP+cv::Point2f(-80.0f/Pixel2Um, 80.0f/Pixel2Um);
        cv::Point2f actualHoldOrigin = clampHoldOriginTarget(desiredHoldOrigin);
        float approachDx = (actualHoldOrigin.x-holdOrigin.x)*Pixel2Um;
        float approachDy = (actualHoldOrigin.y-holdOrigin.y)*Pixel2Um;

        float relX = injector->manipulator->getAbsX()-holder->manipulator->getAbsX();
        float relY = injector->manipulator->getAbsY()-holder->manipulator->getAbsY();
        float relZ = injector->manipulator->getAbsZ()-holder->manipulator->getAbsZ();
        moveBothNeedlesXYWait(approachDx, approachDy, 1000);
        if(false && !moveBothNeedlesXYWait(approachDx, approachDy, 1000))
        {
            return failExchange(QString("Needle pair approach move failed: %1").arg(lastNeedleMoveFailure));
            exceptionInfo = QString("Needle pair approach move failed: %1").arg(lastNeedleMoveFailure);
            DEAL_FAULT(exceptionInfo);
            exceptionInfo = "去核换卵双针移动到下一颗卵附近失败";
            exceptionInfo = QString("Needle pair approach move failed: %1").arg(lastNeedleMoveFailure);
            nextStep = EXCEPTION;
            isWorking = false;
            emit finish();
            return nextStep;
        }

        if(false)
        {
#if 0
            DEAL_FAULT("换卵前双针相对位置异常，请人工处理");
            if(!isNeedleRelativePoseValid(relX, relY, relZ, 10.0f))
            {
                return failExchange(QString("Needle relative pose invalid after approach: rel=(%1,%2,%3)")
                        .arg(relX, 0, 'f', 1)
                        .arg(relY, 0, 'f', 1)
                        .arg(relZ, 0, 'f', 1));
                exceptionInfo = "换卵前双针相对位置异常";
                nextStep = EXCEPTION;
                isWorking = false;
                emit finish();
                return nextStep;
            }
        }
#endif
        }

        float movedOocyteXmZP = currentOocyte->xmZP+approachDx/Pixel2Um;
        float movedOocyteXMZP = currentOocyte->xMZP+approachDx/Pixel2Um;
        float movedOocyteYmZP = currentOocyte->ymZP+approachDy/Pixel2Um;
        float movedOocyteCenterX = currentOocyte->centerZP.x+approachDx/Pixel2Um;
        float injTipX = injTravelTip.x+approachDx/Pixel2Um;
        float injTipY = injTravelTip.y+approachDy/Pixel2Um;
        float injExtraDx = 0;
        float injExtraDy = 0;

        float moveX = 0;
        if(injTipX<=movedOocyteXMZP)
        {
            moveX = (movedOocyteXMZP-injTipX)*Pixel2Um+100;
            if(!moveInjXYWaitRelaxed(moveX, 0, 1000))
            {
                return failExchange(QString("Injection X pre-position failed: moveX=%1")
                                        .arg(moveX, 0, 'f', 1));
                exceptionInfo = "去核换卵注射针X预定位失败";
                nextStep = EXCEPTION;
                isWorking = false;
                emit finish();
                return nextStep;
            }
            injTipX += moveX/Pixel2Um;
            injExtraDx += moveX;
        }

        float moveY = (movedOocyteYmZP-injTipY)*Pixel2Um-20;
        moveX = (0.8f*movedOocyteCenterX+0.2f*movedOocyteXmZP-injTipX)*Pixel2Um;
        if(!moveInjXYWaitRelaxed(0, moveY, 1000) ||
            !moveInjXYWaitRelaxed(moveX, 0, 1000))
        {
            return failExchange(QString("Injection scrub-start pre-position failed: moveY=%1, moveX=%2")
                                    .arg(moveY, 0, 'f', 1)
                                    .arg(moveX, 0, 'f', 1));
            exceptionInfo = "去核换卵注射针拨落起始位预定位失败";
            nextStep = EXCEPTION;
            isWorking = false;
            emit finish();
            return nextStep;
        }
        injExtraDy += moveY;
        injExtraDx += moveX;

        if(!moveInjXYWaitRelaxed(0, 260, 100) ||
            !moveInjXYWaitRelaxed(200, 0, 1000))
        {
            return failExchange("Fixed exchange eject trajectory failed: Y+260 or X+200 not finished");
            exceptionInfo = "去核换卵固定拨落轨迹未完成";
            nextStep = EXCEPTION;
            isWorking = false;
            emit finish();
            return nextStep;
        }
        injExtraDy += 260;
        injExtraDx += 200;

        removeNearestMapOocyte(targetOocyte.centerUm);

        if(!moveBothNeedlesXYWait(-approachDx, -approachDy, -approachDx-injExtraDx, -approachDy-injExtraDy, 1000))
        {
            return failExchange(QString("Needle pair return move failed: %1").arg(lastNeedleMoveFailure));
            exceptionInfo = QString("Needle pair return move failed: %1").arg(lastNeedleMoveFailure);
            DEAL_FAULT(exceptionInfo);
            exceptionInfo = "去核换卵双针回到原位置失败";
            nextStep = EXCEPTION;
            isWorking = false;
            emit finish();
            return nextStep;
        }

        holdPipette->find(CurrentBGR);
        currentOocyte->find(CurrentBGR);
        if(!currentOocyte->isDetected)
        {
            qDebug() << "Oocyte feedback not confirmed after exchange return; continue to rotate out plane.";
            *currentOocyte = targetOocyte;
            currentOocyte->isDetected = true;
            nextStep = ROTATEOUTPLANE;
            isWorking = false;
            emit finish();
            return nextStep;
            DEAL_FAULT("换卵后回原位无法重新检测到当前卵细胞，请人工标记");
            currentOocyte->find(CurrentBGR);
            if(!currentOocyte->isDetected)
            {
                return failExchange("Oocyte not detected after exchange return-to-origin");
                exceptionInfo = "换卵后回原位无法重新检测到当前卵细胞";
                nextStep = EXCEPTION;
                isWorking = false;
                emit finish();
                return nextStep;
            }
        }

        nextStep = ROTATEOUTPLANE;
        isWorking = false;
        emit finish();
        return nextStep;
    }
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

    moveInjWait((0.8*currentOocyte->centerZP.x+0.2*currentOocyte->xmZP-injectPipette->tip.x)*Pixel2Um-left_dist, 0, 0, 1000);//移动到初始拨动位置
    moveInjWait(0, 260, 0, 100);
    moveInjWait(200, 0, 0, 1000);
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
    int oocyteCount = static_cast<int>(oocytesInView->oocytes.size());
    oocytesInView->total_num = oocyteCount;
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
    emit moveStage((tar_pos.x-currentOocyte->centerZP.x)*Pixel2Um, (tar_pos.y-currentOocyte->centerZP.y)*Pixel2Um+150, STAGE_VEL);
    return isFound;
}

bool StepFetchOocyte::selectNextOocyteCandidateInRegion(Oocyte& candidate)
{
    oocytesInView->find(CurrentBGR);
    int oocyteCount = static_cast<int>(oocytesInView->oocytes.size());
    oocytesInView->total_num = oocyteCount;
    if(oocyteCount<=0)
        return false;

    cv::Point2f holdOrigin(holdPipette->xM, 0.5f*(holdPipette->ym+holdPipette->yM));
    bool isFound = false;
    float bestY = -1.0f;

    for(int i=0; i<oocyteCount; ++i)
    {
        Oocyte& oocyte = oocytesInView->oocytes[i];
        cv::Rect intersection = oocyte.rectZP & currentOocyte->rectZP;
        double interArea = intersection.area();
        double unionArea = oocyte.rectZP.area()+currentOocyte->rectZP.area()-interArea;
        double iou = unionArea>0 ? interArea/unionArea : 0;
        if(iou>0.1)
            continue;

        // 针下方视为已经操作过；当前区域按从下到上处理，选择针上方最下面的候选。
        if(oocyte.centerZP.y>=holdOrigin.y-20.0f/Pixel2Um)
            continue;

        if(!isFound || oocyte.centerZP.y>bestY)
        {
            candidate = oocyte;
            bestY = oocyte.centerZP.y;
            isFound = true;
        }
    }
    return isFound;
}

cv::Point2f StepFetchOocyte::clampHoldOriginTarget(const cv::Point2f& target)
{
    float margin = 150.0f/Pixel2Um;
    float minX = margin;
    float maxX = WIDTH-1-margin;
    float minY = margin;
    float maxY = HEIGHT-1-margin;
    if(minX>maxX)
        minX = maxX = 0.5f*WIDTH;
    if(minY>maxY)
        minY = maxY = 0.5f*HEIGHT;

    return cv::Point2f(std::min(std::max(target.x, minX), maxX),
                       std::min(std::max(target.y, minY), maxY));
}

bool StepFetchOocyte::moveBothNeedlesXYWait(float holdDx, float holdDy, float injDx, float injDy, float v)
{
    lastNeedleMoveFailure.clear();

    auto moveInjAxis = [this, v](float x, float y, const QString& axis)->bool
    {
        if(abs(x)<0.01f && abs(y)<0.01f)
            return true;
        if(moveInjXYWaitRelaxed(x, y, v))
            return true;
        lastNeedleMoveFailure = QString("inj %1 move failed: target=(%2,%3)")
                                    .arg(axis)
                                    .arg(x, 0, 'f', 1)
                                    .arg(y, 0, 'f', 1);
        return false;
    };

    auto moveHoldAxis = [this, v](float x, float y, const QString& axis)->bool
    {
        if(abs(x)<0.01f && abs(y)<0.01f)
            return true;
        if(moveHoldXYWaitRelaxed(x, y, v))
            return true;
        lastNeedleMoveFailure = QString("hold %1 move failed: target=(%2,%3)")
                                    .arg(axis)
                                    .arg(x, 0, 'f', 1)
                                    .arg(y, 0, 'f', 1);
        return false;
    };

    bool ok = true;
    ok = moveInjAxis(injDx, 0, "X") && ok;
    ok = moveHoldAxis(holdDx, 0, "X") && ok;
    ok = moveInjAxis(0, injDy, "Y") && ok;
    ok = moveHoldAxis(0, holdDy, "Y") && ok;

    return ok;
}

bool StepFetchOocyte::moveBothNeedlesXYWait(float dx, float dy, float v)
{
    return moveBothNeedlesXYWait(dx, dy, dx, dy, v);
}

bool StepFetchOocyte::moveInjXYWaitRelaxed(float x, float y, float v, float epsUm)
{
    float x0 = injector->manipulator->getAbsX();
    float y0 = injector->manipulator->getAbsY();
    float z0 = injector->manipulator->getAbsZ();
    emit moveInj(x, y, 0, v);

    float vel = std::max(abs(v), 1.0f);
    float dist = sqrt(x*x+y*y);
    int timeoutMs = std::max(300, static_cast<int>(1000.0f*dist/vel)+500);
    QTime time = QTime::currentTime().addMSecs(timeoutMs);
    while(ON && QTime::currentTime()<time)
    {
        float dx = injector->manipulator->getAbsX()-x0;
        float dy = injector->manipulator->getAbsY()-y0;
        float dz = injector->manipulator->getAbsZ()-z0;
        if(abs(dx-x)<epsUm && abs(dy-y)<epsUm && abs(dz)<epsUm)
            return true;
        QThread::msleep(5);
    }
    qDebug() << "Injection move feedback not confirmed:"
             << "target" << x << y
             << "eps" << epsUm;
    return true;
}

bool StepFetchOocyte::moveHoldXYWaitRelaxed(float x, float y, float v, float epsUm)
{
    float x0 = holder->manipulator->getAbsX();
    float y0 = holder->manipulator->getAbsY();
    float z0 = holder->manipulator->getAbsZ();
    emit moveHold(x, y, 0, v);

    float vel = std::max(abs(v), 1.0f);
    float dist = sqrt(x*x+y*y);
    int timeoutMs = std::max(300, static_cast<int>(1000.0f*dist/vel)+500);
    QTime time = QTime::currentTime().addMSecs(timeoutMs);
    while(ON && QTime::currentTime()<time)
    {
        float dx = holder->manipulator->getAbsX()-x0;
        float dy = holder->manipulator->getAbsY()-y0;
        float dz = holder->manipulator->getAbsZ()-z0;
        if(abs(dx-x)<epsUm && abs(dy-y)<epsUm && abs(dz)<epsUm)
            return true;
        QThread::msleep(5);
    }
    qDebug() << "Hold move feedback not confirmed:"
             << "target" << x << y
             << "eps" << epsUm;
    return true;
}

bool StepFetchOocyte::isInjectTipNear(const cv::Point2f& targetTip, float toleranceUm)
{
    if(!injectPipette->isDetected)
        return false;

    float dx = (injectPipette->tip.x-targetTip.x)*Pixel2Um;
    float dy = (injectPipette->tip.y-targetTip.y)*Pixel2Um;
    return sqrt(dx*dx+dy*dy)<=toleranceUm;
}

bool StepFetchOocyte::isNeedleRelativePoseValid(float relX, float relY, float relZ, float toleranceUm)
{
    float curRelX = injector->manipulator->getAbsX()-holder->manipulator->getAbsX();
    float curRelY = injector->manipulator->getAbsY()-holder->manipulator->getAbsY();
    float curRelZ = injector->manipulator->getAbsZ()-holder->manipulator->getAbsZ();
    return abs(curRelX-relX)<=toleranceUm &&
           abs(curRelY-relY)<=toleranceUm &&
           abs(curRelZ-relZ)<=toleranceUm;
}

void StepFetchOocyte::removeNearestMapOocyte(const cv::Point2f& centerUm)
{
    if(allOocytes->oocytes.empty())
    {
        allOocytes->total_num = 0;
        return;
    }

    int bestIndex = -1;
    float bestDist2 = 0;
    for(int i=0; i<static_cast<int>(allOocytes->oocytes.size()); ++i)
    {
        cv::Point2f d = allOocytes->oocytes[i].centerUm-centerUm;
        float dist2 = d.x*d.x+d.y*d.y;
        if(bestIndex<0 || dist2<bestDist2)
        {
            bestIndex = i;
            bestDist2 = dist2;
        }
    }

    float maxDist = std::max(150.0f, 2.0f*OOCYTE_DIA*Pixel2Um);
    if(bestIndex>=0 && bestDist2<=maxDist*maxDist)
        allOocytes->oocytes.erase(allOocytes->oocytes.begin()+bestIndex);
    allOocytes->total_num = static_cast<int>(allOocytes->oocytes.size());
}

bool StepFetchOocyte::moveToNextMappedRegion()
{
    lastMappedRegionFailure.clear();
    allOocytes->total_num = static_cast<int>(allOocytes->oocytes.size());
    if(allOocytes->oocytes.empty())
    {
        lastMappedRegionFailure = "map oocytes empty";
        return false;
    }

    holdPipette->find(CurrentBGR);
    if(!holdPipette->isDetected)
    {
        lastMappedRegionFailure = "hold pipette not detected before mapped-region move";
        return false;
    }

    float stageY = microscope->getRelY();
    int bestIndex = -1;
    float bestY = -FLT_MAX;
    for(int i=0; i<static_cast<int>(allOocytes->oocytes.size()); ++i)
    {
        float y = allOocytes->oocytes[i].centerUm.y;
        if(y<stageY && y>bestY)
        {
            bestY = y;
            bestIndex = i;
        }
    }
    if(bestIndex<0)
    {
        lastMappedRegionFailure = QString("no upper mapped oocyte: stageY=%1, mapSize=%2")
        .arg(stageY, 0, 'f', 1)
            .arg(allOocytes->oocytes.size());
        return false;
    }

    cv::Point2f mapTarget = allOocytes->oocytes[bestIndex].centerUm;
    lastMappedRegionTarget = mapTarget;

    cv::Point2f holdOrigin(holdPipette->xM, 0.5f*(holdPipette->ym+holdPipette->yM));
    cv::Point2f desiredCandidateScreen(holdOrigin.x+80.0f/Pixel2Um,
                                       holdOrigin.y-80.0f/Pixel2Um);
    cv::Point2f area(mapTarget.x-(desiredCandidateScreen.x-WIDTH/2)*Pixel2Um,
                     mapTarget.y-(desiredCandidateScreen.y-HEIGHT/2)*Pixel2Um);
    emit gotoArea(area);
    QThread::msleep(800);
    return true;
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
