#include "pservo.h"
using namespace std;
using namespace cv;

PServo::PServo() {
    pHoldController = new Controller_PID;
    pHoldController->uM[0]=0.01;
    pHoldController->um[0]=-0.01;
    pHoldController->saveParas();
    pHoldController->qParas["kp"]=5e-6;
    pHoldController->qParas["kd"]=1.5e-6;
    pHoldController->qParas["ki"]=1e-9;
    pHoldController->qParas["delta_t"]=0.1;
    pHoldController->setParas(pHoldController->qParas);
    pHoldController->initialize();

    pInjController = new Controller_PID;
    pInjController->uM[0]=0.3;
    pInjController->um[0]=-0.3;
    pInjController->saveParas();
    pInjController->qParas["kp"]=1e-3;
    pInjController->qParas["kd"]=0.0;
    pInjController->qParas["ki"]=0.0;
    pInjController->qParas["delta_t"]=0.1;
    pInjController->setParas(pInjController->qParas);
    pInjController->initialize();

    timerPressure = new QTimer(this);
    connect(timerPressure, SIGNAL(timeout()), this, SLOT(servoPressure()));
}
void PServo::servoPressure()
{
    vector<double> y, u;

    y.clear(); y.push_back(holder->pSensor->pressure);
    pHoldController->set_y(y);
    y.clear(); y.push_back(holdRotateP);
    pHoldController->set_yd(y);
    pHoldController->get_u(u);
    pHoldController->step();
    emit holdBlow(0, u[0], true);

    // y.clear(); y.push_back(injector->pSensor->pressure);
    // pInjController->set_y(y);
    // pInjController->get_yd();
    // pInjController->get_u(u);
    // pInjController->step();
    // emit injSuck(0, -u[0], true);
}
void PServo::start()
{
    timerPressure->start(100);
}
void PServo::stop()
{
    timerPressure->stop();
    emit holdBlow(0, 0, true);
    emit injSuck(0, 0, true);
}

void PServo::startCompare(Mat img, Rect rect)
{
    iniOocyte = img.clone();
    iniRectOocyte = rect;
}
void PServo::compareAndCalHoldP(Mat curImg)
{
    // if(!currentOocyte->isDetected || !holdPipette->isDetected)
    //     return;
    float pm = -1500;
    float pM = 300;
    Mat matchResult;
    matchTemplate(curImg, iniOocyte, matchResult, TM_CCOEFF_NORMED);
    double dMaxVal; //分数最大值
    Point ptMaxLoc; //最大值坐标
    minMaxLoc(matchResult, 0, &dMaxVal, 0, &ptMaxLoc); //寻找结果矩阵中的最大值
    Rect curRect(ptMaxLoc.x, ptMaxLoc.y, iniRectOocyte.width, iniRectOocyte.height);


    Mat curOocyte = curImg(curRect);
    Mat diffOocyte = abs(curOocyte-iniOocyte);
    imshow("ini", iniOocyte); imshow("cur", curOocyte); imshow("dif", diffOocyte);

    float dx, dy, dist;
    dx = (currentOocyte->centerZP.x - holdPipette->xM)*Pixel2Um;
    dy = (currentOocyte->centerZP.y - 0.5*(holdPipette->yM+holdPipette->ym))*Pixel2Um;
    dist = sqrt(dx*dx+dy*dy);
    if(dist > 90)
        holdRotateP=pm;
    else if(holdRotateP<-500)
        holdRotateP = -500;
    else
        holdRotateP+=dPinSuchHoldP(holdRotateP)*(0.5-rotatability(dMaxVal));
    holdRotateP = holdRotateP>pM? pM:holdRotateP;
    holdRotateP = holdRotateP<pm? pm:holdRotateP;
    qDebug()<<"holdRotateP: "<<holdRotateP<<" mean diff: "<<mean(diffOocyte).val[0]<<"dist: "<<dist<<"dMaxVal: "<<dMaxVal;
}
double PServo::rotatability(double x)
{
    double l, m, h, p, k;
    p = 3;

    l = 0.85;
    m = 0.90;
    h = 0.98;
    k = (h-m)/(m-l);
    if(x<=l)
        return 1;
    else if(x>=h)
        return 0;
    return pow(h-x, p)/(pow(h-x, p)+pow(k*(x-l), p));
}
double PServo::dPinSuchHoldP(double x)
{
    x = abs(x);
    double l, m, h, k, kx, p, dpm, dpM;
    dpm = 100;
    dpM = 2000;
    p=2;
    l = 500;
    m = 1500;
    h = 4000;
    k = (m-l)/(h-m);
    if(x<=l)
        kx = 0;
    else if(x>=h)
        kx = 1;
    else
        kx = pow(x-l, p)/(pow(x-l, p)+pow(k*(h-x), p));
    return dpm+kx*(dpM-dpm);
}
