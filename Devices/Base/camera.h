/* 相机基类，实现连接、关闭，抓拍，录像（暂停）的功能
 * 对 UI 发送建立连接、失去连接、更新图像的信号 */
#ifndef CAMERA_H
#define CAMERA_H

#include "public.h"

class Camera: public QObject
{
    Q_OBJECT
public:
    Camera()
    {
        DeviceName = "Camera";
        isConnected = false;
        frame_timer = new QTimer(this);
        frame_timer->setTimerType(Qt::PreciseTimer);
        connect(frame_timer, SIGNAL(timeout()), this, SLOT(readFarme()));
        frameFPS = 50;

        video_timer = new QTimer(this);
        video_timer->setTimerType(Qt::PreciseTimer);
        connect(video_timer, SIGNAL(timeout()), this, SLOT(writeVideo()));
        videoFPS = 50;

        QFile file(":/images/demo.jpg");        file.copy("demo.jpg");
        grayImg = cv::imread("demo.jpg", 0);
        bgrImg  = cv::imread("demo.jpg");
    }
    ~Camera()
    {
        frame_timer->stop();
        video_timer->stop();
        videoWriter.release();
        delete  frame_timer;
        delete  video_timer;
    }
    friend class Microscope;
    cv::Mat             grayImg, bgrImg;                        //灰度图与彩色图

public slots:
    virtual void connectDevice(int nID=0)=0;                    //连接相机
    virtual void readFarme()=0;                                 //读取图像
    virtual void close()=0;                                     //关闭相机
    void captureFrame(QString frame_path)                       //保存图像
    {
        if(frame.empty())
        {
            qDebug("No picture available!");
            return;
        }
        //Use system time to name pictures
        QDateTime dateTime(QDateTime::currentDateTime());
        QString time = dateTime.toString("yyyy-MM-dd-hh-mm-ss");
        //Create picture save pathname
        QString filename = QString("/%1.jpg").arg(time);
        QString full_path = frame_path + filename;
        cv::imwrite(full_path.toStdString(),frame);
    };
    void startRecord(QString video_path)                        //开始录像
    {
        //Use system time to name videos
        QDateTime dateTime(QDateTime::currentDateTime());
        QString time = dateTime.toString("yyyy-MM-dd-hh-mm-ss");
        //Create video save pathname
        QString filename = QString("/%1.avi").arg(time);
        QString full_path = video_path + filename;
        videoWriter.open(full_path.toStdString(), cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), videoFPS, cv::Size(WIDTH, HEIGHT), true);
        video_timer->start(1000/videoFPS);
    }
    void pauseRecord()                                          //暂停录像
    {video_timer->stop();}
    void continueRecord()                                       //继续录像
    {video_timer->start(1000/videoFPS);}
    void endRecord()                                            //结束录像
    {
        video_timer->stop();
        videoWriter.release();
    }
    void writeVideo()                                           //写入视频
    {videoWriter.write(frame);}

public:
    QString             DeviceName;                 //设备名称
    QImage              m_qimage;                   //在界面中显示的图像
    cv::Mat             raw_frame, frame;           //用于存储图像和视频
    cv::VideoWriter     videoWriter;                //存储视频片段
    QTimer              *frame_timer, *video_timer; //从相机读取图像、存储视频的定时器
    float               frameFPS, videoFPS;         //读图像、存视频的帧率
    bool isConnected;                               //设备连接状态
signals:
    void startConnection(); //通知 UI 已建立与该设备的通讯
    void noConnection();    //通知 UI 失去与该设备的通讯
    void updateImage();     //通知 UI 图像已更新
};

#endif // CAMERA_H
