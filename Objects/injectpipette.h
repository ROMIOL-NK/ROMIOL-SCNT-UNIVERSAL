#ifndef INJECTPIPETTE_H
#define INJECTPIPETTE_H
#include "public.h"
#include "utils/neuralnetwork.h"
#include "utils/custom_algorithms.h"

enum STATE_INTERFACE{NEAR_TIP, INJ_VIEW, OUT_VIEW};//胞质面位置的三个状态：未出透明带，视野中的注射针内，视野外
//注射针
class InjectPipette
{
public:
    InjectPipette();
    InjectPipette(std::vector<cv::Point>);
    void reset();
    void set(std::vector<cv::Point>);                                               //根据给定轮廓构建信息
    void find(cv::Mat src, std::vector<cv::Rect> rects=std::vector<cv::Rect>(), bool firstDetection=true);    //从图像中寻找注射针
    void findGLI(cv::Mat src, std::vector<cv::Rect> gliRects=std::vector<cv::Rect>(), bool firstDetection=true);      //从图像中寻找气液交界面 xGLI vGLI (在图像中的x位置：0-(w-1)) 失败则设置为 -1;假定注射针已经找到
    void findOCI(cv::Mat src, int x0, std::vector<cv::Rect> rects=std::vector<cv::Rect>(), bool firstDetection=true); //从图像中寻找胞质交界面 xOCI vOCI (在图像中的x位置：0-(w-1)) 失败则设置为 -1;假定注射针已经找到
    void findSomaInside(cv::Mat src);                                               //寻找针管内的体细胞
    void findSomaOutside(cv::Mat src);                                              //寻找针管外的体细胞
    void setBeforeBubble(cv::Mat src);                                              //产生气泡前保存正常情况的图片
    bool detectGasColumn(cv::Mat src);                                              //检测是否产生气泡
    bool detectBubble(cv::Mat src, float p=INFINITY);                              //检测是否产生气泡

    void separateFromOocyte(Oocyte *oocyte, cv::Mat img);                           //将注射针与卵细胞分离开
    void separateFromContaminants(cv::Mat img);                                     //将注射针与杂 质分离开

    STATE_INTERFACE stateOCI;
    Kalman_Filter filterGLI;
    Kalman_Filter filterOCI;
    MHI mhiOCI;
    std::vector<cv::Point2f> pointsGLI;
    std::vector<cv::Point2f> pointsOCI;
    cv::Mat preGLIimage;
    cv::Mat preOCIimage;
    cv::Mat preBubbleImage;         //由于检测气泡的上一帧图像
    cv::Mat templateGLI;
    cv::Mat templateOCI;
    bool wasGLIDetected;            //已经检测到过，运行追踪算法
    bool wasOCIDetected;
    bool wasSomaDetected;
    bool isDetected;                //是否已经被检测到
    int area;                       //面积
    int xm, xM, ym, yM;             //轮廓的最左/右/上/下端
    int diameter;                   //直径
    float x0GLI, xfGLI, xGLI, vGLI; //气液交界面(Gas-Liquid Interface) x 初始及期望位置 速度
    float x0OCI, xfOCI, xOCI, vOCI; //胞质交界面(Oocyte Cytoplasm Interface) x 初始及期望位置 速度
    float x0Soma,xfSoma,xSoma,vSoma;//针管内体细胞相对于注射针的 x 位置 速度
    float preBubbleValue;           //检测气泡前的平均像素值
    cv::Point xySoma, uvSoma;       //针管外附近体细胞相对于注射针的位置
    cv::Point tip;                  //针尖位置
    std::vector<cv::Point> contour; //轮廓
};

#endif // INJECTPIPETTE_H
