#ifndef TRACK_H
#define TRACK_H

#include "public.h"
#include "Objects/oocyte.h"

class Track: public QObject
{
    Q_OBJECT
public:
    Track();
    ~Track();

    int interval;
    QTimer* timerPB;
    QTimer* timerOocyte;

    std::vector<cv::Rect> rects;
    cv::Ptr<cv::Tracker> trackerPB;
    cv::Rect roiPB;
    cv::Ptr<cv::Tracker> trackerOocyte;
    cv::Rect roiOocyte;

public slots:
    void initPB(cv::Mat img, cv::Rect roi);
    void initOocyte(cv::Mat img, cv::Rect roi);
    void runPB();
    void runOocyte();
    void stopPB();
    void stopOocyte();
};

#endif // TRACK_H
