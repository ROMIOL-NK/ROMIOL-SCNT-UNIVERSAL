#include "steprotateoutplane.h"
using namespace std;
using namespace cv;

StepRotateOutPlane::StepRotateOutPlane()
{
    //去往操作区
    curInjectTipRel = vector<float>(3);
    penDepth = 15;
    iniDisRatio = 1.5;
    iniAngle = 30;
    oneBigAngle = 30;
    oneSmallAngle = 10;
    rotatedAngle = 0;
    maxRotateAngle = 720;
    ANTORAD = 3.14/180;
    rotateOutPlaneMode = ROTATE_OUT_X_LINE_UP;  //ROTATE_OUT_XYZ  ROTATE_OUT_X_LINE_DOWN  ROTATE_OUT_X_LINE_UP   ROTATE_OUT_ORIGINAL_3D
    outPlaneVel = 250;
}

static const int ROTATE_OUT_SHORT_MAX_COUNT = 5;

WORKSTEP StepRotateOutPlane::run()
{
    isWorking = true;
    QThread::msleep(200);
    currentOocyte->find(CurrentGray);
    if(isPBinPlace())
    {
        nextStep = PENETRATE;
        isWorking = false;
        emit finish();
        return nextStep;
    }
    if(currentOocyte->isPBDetected)//极体已经在焦平面，不需要进行平面外拨动
    {
        nextStep = ROTATEINPLANE;
        isWorking = false;
        emit finish();
        return nextStep;
    }

    // 平面外拨动公共准备：检测吸持针、卵母细胞、注射针，并建立相对卵中心的针尖坐标。
    holdPipette->find(CurrentBGR);
    if(!holdPipette->isDetected)
        DEAL_FAULT("无法找到吸持针！请手动标记吸持针！");
    currentOocyte->find(CurrentGray);
    injectPipette->find(CurrentGray);
    if(!currentOocyte->isDetected || !injectPipette->isDetected)
    {DEAL_FAULT("请手动标注卵细胞|注射针，完成后关闭窗口！")}
    oocytesRadius = currentOocyte->radiusZP*Pixel2Um; //单位微米
    curInjectTipRel[0] = (injectPipette->tip.x-currentOocyte->centerZP.x)*Pixel2Um;
    curInjectTipRel[1] = (injectPipette->tip.y-currentOocyte->centerZP.y)*Pixel2Um;
    curInjectTipRel[2] = 0;

    ON = true;
    emit sgnInitTrackerOocyte(CurrentBGR, currentOocyte->rectZP);

    switch(rotateOutPlaneMode)
    {
    case ROTATE_OUT_ORIGINAL_3D:
        rotate_original_3d();
        break;
    case ROTATE_OUT_X_LINE_DOWN:
        rotate_x_line_down();
        break;
    case ROTATE_OUT_X_LINE_UP:
        rotate_x_line_up();
        break;
    case ROTATE_OUT_XYZ:
    default:
        rotate_xyz();
        break;
    }

    emit sgnStopTrackOocyte();

    // 根据停止条件返回下一步：急停交互、极体进入焦平面、或直接进入穿刺。
    if(ON==false)
    {
        DEAL_FAULT("平面外拨动完成，是否进行平面内拨动");
        isWorking = false;
        emit finish();
        return nextStep;
    }
    if(currentOocyte->isPBDetected)
    {
        nextStep = ROTATEINPLANE;
        isWorking = false;
        emit finish();
        return nextStep;
    }
    else
    {
        nextStep = PENETRATE;
        isWorking = false;
        emit finish();
        return nextStep;
    }
}

void StepRotateOutPlane::rotate_original_3d()
{
    // 原始3D弧线拨法：保持旧版run()中的XZ圆弧拨动语义。
    injet_move_to_rel(iniDisRatio*oocytesRadius,0,0);
    injet_move_to_rel(iniDisRatio*oocytesRadius,0,(oocytesRadius-penDepth)*sin(iniAngle*ANTORAD));

    int smallRotatedAngle = 0;
    rotatedAngle = 0;
    while (ON&&(!currentOocyte->isPBDetected) && rotatedAngle < maxRotateAngle)
    {
        inject_rot_arc_xz(iniAngle,0,0);
        while ((smallRotatedAngle<oneBigAngle)&&(rotatedAngle < maxRotateAngle))
        {
            inject_rot_arc_xz(iniAngle,smallRotatedAngle,oneSmallAngle);
            smallRotatedAngle += oneSmallAngle ;
            rotatedAngle += oneSmallAngle ;
            if(currentOocyte->findPB(CurrentBGR))
                break;
        }
        if(currentOocyte->isPBDetected)
            break;
        else
        {
            injet_move_to_rel(iniDisRatio*oocytesRadius,0,(oocytesRadius-penDepth)*sin((iniAngle+smallRotatedAngle)*ANTORAD));
            injet_move_to_rel(iniDisRatio*oocytesRadius,0,(oocytesRadius-penDepth)*sin(iniAngle*ANTORAD));
            smallRotatedAngle = 0;
            QThread::msleep(600);
            if(currentOocyte->findPB(CurrentBGR))
                break;
        }
    }
    injet_move_to_rel(iniDisRatio*oocytesRadius,0,0);
}

void StepRotateOutPlane::rotate_x_line_down()
{
    // x_line_down：针壁从down侧接触，沿X/Y直线蹭动细胞。
    rotate_x_line(true);
}

void StepRotateOutPlane::rotate_x_line_up()
{
    // x_line_up：针壁从up侧接触，沿X/Y直线蹭动细胞。
    rotate_x_line(false);
}

void StepRotateOutPlane::rotate_x_line(bool isDown)
{
    // 正Z方向与原始3D拨法一致；先让针壁贴到细胞，再沿Y方向蹭动。
    float r = oocytesRadius;
    float workR = r - penDepth;
    if(workR < 10)
        workR = 10;
    float contactX = safe_injection_side_x(0.35*r);
    float startX = iniDisRatio*r;
    float safeZ = cos(25*ANTORAD)*workR;
    if(safeZ < 20)
        safeZ = 20;

    float sideSign = isDown ? 1.0 : -1.0;
    float startY = sideSign*sin(40*ANTORAD)*workR;
    float scrubY = -sideSign*0.75*workR;

    int rotateCount = 0;
    while (ON&&(!currentOocyte->isPBDetected) && rotateCount < ROTATE_OUT_SHORT_MAX_COUNT)
    {
        // 接触点：针壁已经贴住细胞，不能只停在外侧。
        injet_move_to_rel(contactX,startY,safeZ,outPlaneVel);
        if(!ON) break;
        injet_move_to_rel(contactX,scrubY,safeZ,outPlaneVel);
        if(!ON) break;
        injet_move_to_rel(startX,scrubY,safeZ,outPlaneVel);
        if(!ON) break;
        injet_move_to_rel(startX,startY,safeZ,outPlaneVel);

        rotateCount++;
        if(currentOocyte->findPB(CurrentBGR))
            break;
        QThread::msleep(100);
    }
    injet_move_to_rel(startX,0,0,outPlaneVel);
}

void StepRotateOutPlane::rotate_xyz()
{
    // xyz：A到B的斜向进入段中途蹭到细胞，C点向外侧和正Z方向撤出。
    float r = oocytesRadius;
    float workR = r - penDepth;
    if(workR < 10)
        workR = 10;
    float contactX = safe_injection_side_x(0.25*r);
    float startX = iniDisRatio*r;
    float safeZ = 0.65*workR;
    if(safeZ < 20)
        safeZ = 20;
    float contactY = -0.65*workR;
    float retractZ = safeZ + 0.35*workR;

    int rotateCount = 0;
    while (ON&&(!currentOocyte->isPBDetected) && rotateCount < ROTATE_OUT_SHORT_MAX_COUNT)
    {
        // A：外侧、中轴线、正Z安全高位。
        injet_move_to_rel(startX,0,safeZ,outPlaneVel);
        if(!ON) break;
        // B：偏下并略越过细胞表面，A->B段中途产生接触。
        injet_move_to_rel(contactX,contactY,safeZ,outPlaneVel);
        if(!ON) break;
        // C：向外侧和更高Z方向撤出。
        injet_move_to_rel(startX,contactY,retractZ,outPlaneVel);
        if(!ON) break;
        // 回到A后再进行极体检测和下一轮判断。
        injet_move_to_rel(startX,0,safeZ,outPlaneVel);

        rotateCount++;
        if(currentOocyte->findPB(CurrentBGR))
            break;
        QThread::msleep(100);
    }
    injet_move_to_rel(startX,0,0,outPlaneVel);
}

float StepRotateOutPlane::safe_injection_side_x(float preferredX)
{
    // 吸持针在细胞左侧，平面外拨动不主动越过细胞右半侧，降低碰针风险。
    float r = oocytesRadius;
    float minX = 0.35*r;
    if(holdPipette->isDetected)
    {
        float holdTipRelX = (holdPipette->xM-currentOocyte->centerZP.x)*Pixel2Um;
        float holdSafeX = holdTipRelX + 0.5*r;
        if(minX < holdSafeX)
            minX = holdSafeX;
    }
    if(preferredX < minX)
        return minX;
    return preferredX;
}

void StepRotateOutPlane::injet_move_to_rel(float x,float y,float z,float v)
{
    moveInjWait(x-curInjectTipRel[0],y-curInjectTipRel[1],z-curInjectTipRel[2], v);
    curInjectTipRel.clear();
    curInjectTipRel.push_back(x);
    curInjectTipRel.push_back(y);
    curInjectTipRel.push_back(z);
}

void StepRotateOutPlane::inject_rot_arc_xz(int iniAngle,int smallRotatedAngle,int oneSmallAngle)
{
    injet_move_to_rel((oocytesRadius-penDepth)*cos((iniAngle+smallRotatedAngle+oneSmallAngle)*3.14/180),0,
                      (oocytesRadius-penDepth)*sin((iniAngle+smallRotatedAngle+oneSmallAngle)*3.14/180));
}

bool StepRotateOutPlane::isPBinPlace()
{
    if(currentOocyte->findPB(CurrentBGR))
        if(abs(atan2(currentOocyte->centerPB.y-currentOocyte->centerZP.y, currentOocyte->centerPB.x-currentOocyte->centerZP.x)) < 45*CV_PI/180)
        {
            qDebug()<<"PB in Place!";
            return true;
        }
    return false;
}
