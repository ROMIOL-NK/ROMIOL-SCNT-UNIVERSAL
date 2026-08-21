#ifndef CAMERATOUP_H
#define CAMERATOUP_H
#include "utils/toupcam.h"
#include "Devices/Base/camera.h"
//图谱相机 MTR3CCD01400KPA http://touptek.com/product/showproduct.php?lang=en&id=248
class CameraToup : public Camera
{
    Q_OBJECT
public:
    CameraToup();
    ~CameraToup();

    ToupcamDevice  m_allDevs[TOUPCAM_MAX];
    ToupcamDevice  m_curDev;
    HToupcam       m_hToupcam;

    cv::Rect       roi;
    uchar*         m_pData;
    short int      m_temperature;
    short int      m_tint;
    short int      m_imgWidth;
    short int      m_imgHeight;

public slots:
    virtual void connectDevice(int nID=0);          //连接相机
    virtual void readFarme();                       //保存图像
    virtual void close();                           //关闭相机

private:
    static void eventCallBack(unsigned nEvent, void* pCallbackCtx);
    void handleImageCallback();
};

#endif // CAMERATOUP_H
