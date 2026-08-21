#ifndef STAGETRIO_H
#define STAGETRIO_H

#include "Base/stage.h"
#include "utils/triopc.h"

enum AxisNum{Axis_X=0, Axis_Y};
using namespace TrioPc;

class StageTrio : public Stage
{
    Q_OBJECT
public:
    StageTrio();
    ~StageTrio();
private:
    TrioPC* trioPc;     //Trio 实例 与驱动器相关
public slots:
    virtual void connectDevice(QString portName=strPortStage);
    virtual void setInchingSpeed(float speed);
    virtual void moveX(float x, bool CONTINUOUS=false, float vel=0);
    virtual void moveY(float y, bool CONTINUOUS=false, float vel=0);
    virtual void move(float x, float y, float vel=0);
    virtual void stop();
    virtual void close(){isConnected = false;trioPc->Close();}
};

#endif // STAGETRIO_H
