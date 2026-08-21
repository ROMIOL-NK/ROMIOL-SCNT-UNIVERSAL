#ifndef SOMAS_H
#define SOMAS_H
#include "public.h"
#include "Components/components.h"
#include "Objects/soma.h"
#include "utils/objectdetector.h"

// 全部的体细胞
class Somas:public QObject
{
    Q_OBJECT
public:
    Somas();

    bool isDetected;                    //是否已经被检测到
    int total_num;                  // 总数
    std::vector<Soma> somas;             //
signals:
    void finishFind();
public slots:
    void find(cv::Mat img, std::vector<cv::Rect> rects=std::vector<cv::Rect>(), bool firstDetection=true);
    void reset();
};

#endif // SOMAS_H
