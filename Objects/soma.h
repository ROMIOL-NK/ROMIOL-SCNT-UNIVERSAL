#ifndef SOMA_H
#define SOMA_H
#include "public.h"

// 体细胞
class Soma
{
public:
    Soma();
    ~Soma();
    Soma(float S, cv::Point2f p, std::vector<cv::Point> c):area(S), center(p), contour(c){};

    bool isDetected;                    //是否已经被检测到
    int xm, xM, ym, yM;                 //轮廓的最左/右/上/下端
    float area;                         //面积
    cv::Point2f center;                 //中心
    std::vector<cv::Point> contour;     //轮廓
    cv::Rect roi;                       //矩形标记的体细胞区域
    cv::Ptr<cv::Tracker> tracker;       //跟踪器
    std::vector<cv::Point2f> trackingPoints;

    void set(std::vector<cv::Point> contour);
    void find(cv::Mat);
    void findinPipette(cv::Mat);
    void track(cv::Mat);
    void reset();
};

#endif // SOMA_H
