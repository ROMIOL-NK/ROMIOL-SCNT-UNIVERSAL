/* 相机类，通过 VideoCapture打开，实现抓拍，录像（暂停）的功能
 * 对 UI 发送建立连接、失去连接、更新图像的信号
*/
#ifndef CAMERAUSB_H
#define CAMERAUSB_H

#include "Devices/Base/camera.h"
// USB接口 可直接通过 opencv打开的相机
class CameraUSB: public Camera
{
    Q_OBJECT
public:
    CameraUSB();
    ~CameraUSB();

public slots:
    virtual void connectDevice(int nID=0);          //连接相机
    virtual void readFarme();                       //保存图像
    virtual void close();                           //关闭相机

private:
    cv::VideoCapture    videoCapture;               //从相机读视频
};

#endif // CAMERAUSB_H
