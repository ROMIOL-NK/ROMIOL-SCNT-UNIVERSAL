#ifndef PSERVO_H
#define PSERVO_H
#include "public.h"
#include "Objects/objects.h"
#include "Controllers/controllers.h"
#include "Components/holder.h"

class PServo: public QObject
{
    Q_OBJECT
public:
    PServo();
    QTimer* timerPressure;
    Controller* pHoldController, *pInjController;
    cv::Mat iniOocyte;
    cv::Rect iniRectOocyte;

    double rotatability(double x);          // x为匹配程度(0-1)返回根据当前匹配程度计算的细胞可旋转性 x越接近1旋转性越接近0
    double dPinSuchHoldP(double x);         // 吸持压为p时每次调节的最大压强变化值

public slots:
    void servoPressure();
    void start();
    void stop();

    void startCompare(cv::Mat, cv::Rect);
    void compareAndCalHoldP(cv::Mat);
signals:
    void holdBlow(float, float, bool);
    void injSuck(float, float, bool);
};

#endif // PSERVO_H
