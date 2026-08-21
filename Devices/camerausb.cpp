#include "camerausb.h"

using namespace cv;

CameraUSB::CameraUSB()
{
    DeviceName = "Camera USB";
};
CameraUSB::~CameraUSB()
{
    close();
};

void CameraUSB::connectDevice(int nID)
{
    videoCapture.open(nID);
    if (!videoCapture.isOpened())
    {
         qDebug() << "Can't connect " + DeviceName;
         emit noConnection();
         return;
    }
    isConnected = true;
    qDebug("Camera OK!");
    emit startConnection();
    videoCapture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    videoCapture.set(cv::CAP_PROP_FRAME_WIDTH, WIDTH);
    videoCapture.set(cv::CAP_PROP_FRAME_HEIGHT,HEIGHT);
    videoCapture.set(cv::CAP_PROP_FPS, frameFPS);

    frame_timer->start(1000/frameFPS);
}

void CameraUSB::readFarme()//(定时)读取图像
{
    videoCapture>>raw_frame; //Read current frame
    frame = raw_frame.clone();
    if (!frame.empty())
    {
        resize(frame, frame, Size(WIDTH, HEIGHT));
        if(frame.channels()==1)
        {
            grayImg = frame.clone();
            cvtColor(grayImg, bgrImg, COLOR_GRAY2BGR);
        }
        else if(frame.channels()==3)
        {
            bgrImg = frame.clone();
            cvtColor(bgrImg, grayImg, COLOR_BGR2GRAY);
        }
        else
        {
            qDebug("Wrong image format!!!");
            return;
        }
        m_qimage = QImage(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_BGR888);
        emit updateImage();
    }
    else
    {
        qDebug("can not read frame");
        frame_timer->stop();
        emit noConnection();
    }
}

void CameraUSB::close()
{
    videoCapture.release();
    emit noConnection();
}
