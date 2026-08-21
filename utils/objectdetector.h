#ifndef OBJECTDETECTOR_H
#define OBJECTDETECTOR_H

#include "public.h"
#include "Objects/objects.h"
#include "WorkFlow/vision.h"

class ObjectDetector: public QObject
{
    Q_OBJECT
public:
    ObjectDetector();
    ~ObjectDetector();
    std::string img_folder;
    std::map<std::string, std::vector<cv::Rect>> result;
    std::map<std::string, std::vector<cv::Rect>> getRects(cv::Mat input);
    bool inUse;
signals:
    void sgnVision();
    void resultReceived();
private slots:
    void handleResult(std::map<std::string, std::vector<cv::Rect>>);
};

#endif // OBJECTDETECTOR_H
