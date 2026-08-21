#ifndef MICROSCOPE_H
#define MICROSCOPE_H
#include "Devices/focusdeviceprior.h"
#include "Devices/focusdevicenikon.h"
#include "Devices/stageprior.h"
#include "Devices/stagetrio.h"
#include "Devices/stagezmotion.h"
#include "Devices/camerausb.h"
#include "Devices/cameratoup.h"

class Microscope: public QObject
{
    Q_OBJECT
public:
    Microscope();
    ~Microscope();

    float oriz;     //z的自定义原点

    FocusDevice*    focusdevice;
    Stage*          stage;
    Camera*         camera;

private:
    TISCOPELib::INikonTiPtr pMicroscope;    // 显微镜指针
    bool isTiEConnected;

public slots:
    // 运动响应函数 相对运动
    void moveX(float x, bool CONTINUOUS=false, float vel=0);
    void moveY(float y, bool CONTINUOUS=false, float vel=0);
    void moveZ(float z, bool CONTINUOUS=false, float vel=0);
    void move(float dx, float dy, float dz);    //移动 (dx, dy, dz) 的距离
    void moveToABS(float x, float y, float z);  //移动到绝对坐标为 (x, y, z)的位置
    void moveToREL(float x, float y, float z);  //移动到相对坐标为 (x, y, z)的位置
    void stop();
    void close(){focusdevice->close();stage->close();camera->close();}
    void closeCamera();

    void captureFrame(QString frame_path);

    void  changeObjective(Objective targetObj);
    float getRelX(){return stage->getXPos()-stage->orix; }
    float getRelY(){return stage->getYPos()-stage->oriy; }
    float getRelZ(){return focusdevice->getZPos()-oriz;}
    float getAbsX(){return stage->getXPos();}
    float getAbsY(){return stage->getYPos();}
    float getAbsZ(){return focusdevice->getZPos();}
    float getStageInchingSpeed(){return stage->inchingSpeed;}

    bool  isCameraConnected(){return camera->isConnected;}
    bool  isStageConnected(){return stage->isConnected;}
    bool  isFocusDeviceConnected(){return focusdevice->isConnected;}
    bool  isConnected(){return focusdevice->isConnected&stage->isConnected&camera->isConnected;}//设备连接信息
};

#endif // MICROSCOPE_H
