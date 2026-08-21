#ifndef HOLDPIPETTE_H
#define HOLDPIPETTE_H
#include "public.h"
//吸持针
class HoldPipette
{
public:
    HoldPipette();
    HoldPipette(std::vector<cv::Point>);
    void reset();
    void find(cv::Mat src, std::vector<cv::Rect> rects=std::vector<cv::Rect>(), bool firstDetection=true);  //从图像中寻找吸持针
    void set(std::vector<cv::Point>);                                                                       //根据给定轮廓构建信息

    bool isDetected;                //是否已经被检测到
    int area;                       //面积
    int xm, xM, ym, yM;             //轮廓的最左/右/上/下端
    int diameter;                   //直径
    cv::Point tip;                  //针尖
    std::vector<cv::Point> contour; //轮廓
};

#endif // HOLDPIPETTE_H
