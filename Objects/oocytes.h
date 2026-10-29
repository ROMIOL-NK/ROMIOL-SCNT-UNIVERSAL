#ifndef OOCYTES_H
#define OOCYTES_H
#include "public.h"
#include "Components/components.h"
#include "Objects/oocyte.h"
#include "utils/neuralnetwork.h"
#include "utils/objectdetector.h"
// 全部的卵细胞
class Oocytes:public QObject
{
    Q_OBJECT
public:
    Oocytes();

    bool isDetected;                                //是否已经被检测到
    std::vector<Oocyte> oocytes;                    //卵细胞列表
signals:
    void finishFind();
public slots:
    void find(cv::Mat img, cv::Point2f left_top_um);
    void find(cv::Mat src, std::vector<cv::Rect> oocytesRects=std::vector<cv::Rect>(), bool firstDetection=true);
    void reset();
};

#endif // OOCYTES_H
