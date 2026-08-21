#ifndef STAGEZMOTION_H
#define STAGEZMOTION_H

#include "Base/stage.h"
#include "utils/zauxdll.h"

class StageZMotion : public Stage
{
    Q_OBJECT
public:
    StageZMotion();
    ZMC_HANDLE g_handle;
    float ratio = 0.405;
    ~StageZMotion();
public slots:
    virtual void connectDevice(QString portName=strPortStage);
    virtual void setInchingSpeed(float speed);
    virtual void moveX(float x, bool CONTINUOUS=false, float vel=0);
    virtual void moveY(float y, bool CONTINUOUS=false, float vel=0);
    virtual void move(float x, float y, float vel);
    virtual void stop();
    virtual void close(){isConnected = false;}
};

#endif // STAGEZMOTION_H
