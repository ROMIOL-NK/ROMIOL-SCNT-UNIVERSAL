#include "somas.h"
using namespace cv;
using namespace std;
//获取所有的体细胞，前期要求：
//1. 完成方向和尺寸的标定
//2. 完成地图的构建
Somas::Somas()
{
    isDetected = false;
    somas.clear();
    total_num = 0;
}
void Somas::find(cv::Mat src, vector<Rect> rects, bool firstDetection)
{
    reset();
    if(src.empty())
        return;
    Mat bgrImg, img;
    if(src.channels()==1)
    {
        cvtColor(src, bgrImg, COLOR_GRAY2BGR);
        img = src.clone();
    }
    else
    {
        bgrImg = src.clone();
        cvtColor(src, img, COLOR_BGR2GRAY);
    }
    map<string, vector<Rect>> results;
    if(rects.empty() && firstDetection)
    {
        if(isUsingSocket)
            results = objectDetector->getRects(bgrImg);
        else
            results = yolo->getRects(bgrImg);
        rects = results["Soma"];
    }
    if(!rects.empty())
    {
        Soma soma;
        Rect rect;
        vector<Point> contour;
        for(int i=0; i<rects.size();++i)
        {
            rect = rects[i];
            rect &= Rect(0, 0, src.cols, src.rows);
            //设置透明带
            contour.clear();
            contour.push_back(Point(rect.x, rect.y));
            contour.push_back(Point(rect.x+rect.width, rect.y));
            contour.push_back(Point(rect.x+rect.width, rect.y+rect.height));
            contour.push_back(Point(rect.x, rect.y+rect.height));
            soma.roi = rect;
            soma.set(contour);
            somas.push_back(soma);
            total_num++;
        }
        isDetected = true;
        emit finishFind();
        return;
    }


    float S, C, roundness; // S: 面积  C: 周长  roundness：圆度
    cv::Mat gray, bImg;
    std::vector<std::vector<Point>> contours;
    std::vector<Vec4i> hierarchy;
    std::vector<Point> contour;
    Rect boundRect;
    Moments m;
    Point2f p;

    if(src.channels()==3)
        cvtColor(src, gray, COLOR_BGR2GRAY);
    else
        gray = src.clone();
    Laplacian(gray, gray, -1, 3, 1.0);
    // imshow("gray", gray);
    threshold(gray, bImg, 0, 255, THRESH_OTSU|THRESH_BINARY_INV);

    findContours(bImg,contours,hierarchy,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE,Point());

    // 显示结果
    cv::Mat showImg = gray.clone();
    cvtColor(showImg, showImg, COLOR_GRAY2BGR);
    drawContours(showImg, contours, -1, Scalar(0, 0, 255), 1);
    // imshow("Somas", showImg);

    for(size_t i=0; i<contours.size(); ++i)
    {
        contour = contours[i];
        S = contourArea(contour)*Pixel2Um*Pixel2Um;
        C = arcLength(contour, true)*Pixel2Um;
        roundness = 4*M_PI*S/(C*C);
        if(S>100 && S<6000 && (roundness>0.5))//TODO: adjust Paras
        {
            m = moments(contour);
            if(m.m00==0)
                continue;

            p = Point2f(m.m10/m.m00, m.m01/m.m00);
            somas.push_back(Soma(S, p, contour));
        }
    }
    total_num = somas.size();
    if(total_num>0)
        isDetected = true;
    emit finishFind();
}
void Somas::reset()
{
    isDetected = false;
    somas.clear();
    total_num = 0;
}
