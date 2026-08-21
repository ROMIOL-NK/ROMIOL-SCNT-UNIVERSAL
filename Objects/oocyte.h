#ifndef OOCYTE_H
#define OOCYTE_H
#include "public.h"

// 卵细胞

class Oocyte
{
public:
    Oocyte();
    Oocyte(float S, cv::Point2f p, std::vector<cv::Point> c):area(S), center(p), contour(c){};
    void reset();
    void find(cv::Mat img, std::vector<cv::Rect> rects=std::vector<cv::Rect>(), bool firstDetection=true);
    bool findPB(cv::Mat img, std::vector<cv::Rect> rects=std::vector<cv::Rect>(), bool firstDetection=true);
    void findZP(cv::Mat src);                                                                   //获取透明带轮廓
    void set(std::vector<cv::Point> contour, cv::Point2f left_top_up);
    void setZP(std::vector<cv::Point> contour);

    void calcStrain(cv::Mat src);                                                               //计算卵细胞的应变

    void initInnerPoints();

    //TODO: 有些计算中需要锁定，阻止重新检测细胞；比如计算内应变
    cv::Ptr<cv::Tracker> tracker;       //跟踪器
    cv::Mat preImg, preSImg;            // preSImg: 用于计算应变的上一帧图像
    bool isDetected;                    //是否已经被检测到
    bool isPBDetected;                  //极体是否已经被检测到
    bool isZPDetected;                  //透明带是否已经被检测到
    int xm, xM, ym, yM;                 //轮廓的最左/右/上/下端
    int xmZP, xMZP, ymZP, yMZP;         //轮廓的最左/右/上/下端
    float area;                         //面积
    float radius;                       //半径(像素)
    float radiusZP;                     //透明带半径(像素)
    double strain;                      //卵细胞应变
    float l0;                           //应变采样点的初始间隔
    cv::Rect rectPB;                    //极体矩形框
    cv::Rect rectZP;                    //透明带矩形框
    cv::Point2f centerPB;               //极体中心
    cv::Point2f center;                 //中心
    cv::Point2f centerZP;               //透明带中心
    cv::Point2f centerUm;               //全局地图中的中心相对位置(单位：um)，平台移动到此位置时，该细胞位于视野中心
    std::vector<cv::Point> contour;     //细胞轮廓
    std::vector<cv::Point> contourZP;   //透明带轮廓
    std::vector<cv::Point2f>innerPoints;//卵细胞内点，用于计算内应变
    std::vector<cv::Point> pairs;       //点对
    std::vector<bool> isPairValid;      //点对有效性标志（是否离开细胞内）
    std::vector<double> weights;        //权重
    cv::RotatedRect eZP;                //透明带椭圆拟合；
    std::vector<cv::Point2f> newPoints; //临时记录
    std::vector<uchar> status;
    std::vector<float> err;
    cv::Size winSize;
    cv::TermCriteria termcrit;
};

#endif // OOCYTE_H
