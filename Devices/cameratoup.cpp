#include "cameratoup.h"
#define COUNT_TIME_RANGE 500000

using namespace cv;
using namespace std;
CameraToup::CameraToup()
{
    m_hToupcam = NULL;
    DeviceName = "Camera ToupTech";
}
CameraToup::~CameraToup()
{
    close();
}

void CameraToup::connectDevice(int nID)
{
    int ret = Toupcam_Enum(m_allDevs);
    if(nID>=ret)
    {
        qDebug() << "Toup Camera Index Error!";
        return;
    }
    m_curDev = m_allDevs[nID];
    m_imgWidth = m_curDev.model->res[0].width;
    m_imgHeight = m_curDev.model->res[0].height;
    m_hToupcam = Toupcam_Open(m_curDev.id);

    if (!m_hToupcam)
    {
         qDebug() << "Can't connect " + DeviceName;
         emit noConnection();
         return;
    }
    else
    {
        Toupcam_put_Speed(m_hToupcam, 1);
        Toupcam_put_eSize(m_hToupcam, static_cast<unsigned>(0));
        m_pData = new uchar[TDIBWIDTHBYTES(m_imgWidth * 24) * m_imgHeight];
        Toupcam_put_ExpoAGain(m_hToupcam, 300);
        Toupcam_put_ExpoTime(m_hToupcam, 50000);
        Toupcam_put_TempTint(m_hToupcam, 6503, 1000);
        Toupcam_put_AutoExpoEnable(m_hToupcam, false);
        Toupcam_put_Contrast(m_hToupcam, 0);
        // Toupcam_put_AutoExpoEnable(m_hToupcam, true);


        Toupcam_put_Option(m_hToupcam, TOUPCAM_OPTION_TEC, 0);//关闭制冷
        Toupcam_put_Option(m_hToupcam, TOUPCAM_OPTION_FAN, 0);//关闭风扇
        Toupcam_put_Option(m_hToupcam, TOUPCAM_OPTION_BYTEORDER, 0);//设置 RGB/BGR顺序

        if (Toupcam_StartPullModeWithCallback(m_hToupcam, eventCallBack, this) >= 0)
        {
            isConnected = true;
            qDebug("Camera OK!");
            emit startConnection();
            frame_timer->start(1000/frameFPS);
            int nw = m_imgWidth/WIDTH;
            int nh = m_imgHeight/HEIGHT;
            int n = min(nw, nh);
            int wMargin = (m_imgWidth- n*WIDTH) /2;
            int hMargin = (m_imgHeight-n*HEIGHT)/2;
            roi = Rect(wMargin, hMargin, n*WIDTH, n*HEIGHT);
        }
    }
}

void CameraToup::readFarme()//(定时)读取图像
{
    if (!raw_frame.empty())
    {
        // frame = raw_frame.clone();
        frame = raw_frame(roi).clone();

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
}

void CameraToup::close()
{
    if(m_hToupcam)
    {
        Toupcam_Close(m_hToupcam);
        m_hToupcam = nullptr;
        delete[] m_pData;
        m_pData = nullptr;
    }
    emit noConnection();
    frame_timer->stop();
    video_timer->stop();
}


void CameraToup::eventCallBack(unsigned nEvent, void* pCallbackCtx)
{
    CameraToup* pThis = static_cast<CameraToup*>(pCallbackCtx);
    if (TOUPCAM_EVENT_IMAGE == nEvent)
        pThis->handleImageCallback();
}

void CameraToup::handleImageCallback()
{
    if(!m_hToupcam)
    {
        close();
        return;
    }
    unsigned width = 0, height = 0;
    int exitCode;
    try {
        exitCode = Toupcam_PullImage(m_hToupcam, m_pData, 24, &width, &height);
        if ( exitCode >= 0)
        {
            QImage image(m_pData, width, height, QImage::Format_RGB888);
            Mat tmp = Mat(image.height(), image.width(), CV_8UC3, image.bits(), image.bytesPerLine());
            cvtColor(tmp,raw_frame,COLOR_RGB2BGR);//COLOR_BGR2RGB
        }

    } catch (exception &e) {
        close();
        qDebug()<<e.what();
    }
}
