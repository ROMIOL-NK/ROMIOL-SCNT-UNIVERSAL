#include "holdpipette.h"
#include "oocytes.h"
#include "utils/neuralnetwork.h"
#include "utils/objectdetector.h"

using namespace cv;
using namespace std;
HoldPipette::HoldPipette()
{
    isDetected = false;
    contour = {{}};
}
HoldPipette::HoldPipette(std::vector<cv::Point> curContour)
{
    set(curContour);
}

void HoldPipette::reset()
{
    isDetected = false;
    contour.clear();
}
//将轮廓 curContour 设置为吸持针的轮廓，计算面积、上下左右边界、直径以及针尖位置
void HoldPipette::set(std::vector<cv::Point> curContour)
{
    reset();
    if(curContour.empty())
        return;
    contour.assign(curContour.begin(), curContour.end());
    area = contourArea(contour);
    xm = min_element(contour.begin(), contour.end(), [](Point p1, Point p2){return p1.x<p2.x;})->x;
    xM = max_element(contour.begin(), contour.end(), [](Point p1, Point p2){return p1.x<p2.x;})->x;
    ym = min_element(contour.begin(), contour.end(), [](Point p1, Point p2){return p1.y<p2.y;})->y;
    yM = max_element(contour.begin(), contour.end(), [](Point p1, Point p2){return p1.y<p2.y;})->y;
    diameter = yM-ym;
    tip = Point(xM, int(ym+yM)/2);
    isDetected = true;
}
void HoldPipette::find(Mat src, vector<Rect> holdRects, bool firstDetection)
{
    reset();
    if(src.empty())
        return;
    Mat bgrImg, roi, biRoi, element;
    vector<vector<Point>> contours;         //提取出的所有轮廓
    vector<Point> contour;
    //    imshow("src", src);
    //    waitKey(1);
    if(src.channels()==1)
        cvtColor(src, bgrImg, COLOR_GRAY2BGR);
    else
        bgrImg = src.clone();
    map<string, vector<Rect>> results;
    if(holdRects.empty() && firstDetection)
    {
        if(isUsingSocket)
        {
            results = objectDetector->getRects(bgrImg);
            holdRects = results["holder_current"];
        }
        else
        {
            results = yolo->getRects(bgrImg);
            vector<Rect> injectRects = results["Injector"];
            holdRects = results["Holder"];//Holder
            holdRects.insert(holdRects.end(), injectRects.begin(), injectRects.end()); // TODO: 吸持针误检为注射针
        }
    }
    if(!holdRects.empty())
    {
        for(int i=0; i<holdRects.size(); ++i)
        {
            if(holdRects[i].x+holdRects[i].width > src.cols-200)
                continue;//更可能是注射针
            holdRects[i]&=Rect(0,0,src.cols, src.rows);
            //直接将矩形框作为轮廓
            contour.clear();
            contour.push_back(Point(holdRects[i].x, holdRects[i].y));
            contour.push_back(Point(holdRects[i].x+holdRects[i].width, holdRects[i].y));
            contour.push_back(Point(holdRects[i].x+holdRects[i].width, holdRects[i].y+holdRects[i].height));
            contour.push_back(Point(holdRects[i].x, holdRects[i].y+holdRects[i].height));
            set(contour);
            break;
        }
    }
}
////传统方法
//void HoldPipette::find(Mat src)
//{
//    Mat img, biImg, element;
//    int  min_y, max_y, w, h;
//    float S, L, roundness;
//    vector<vector<Point>> contours;
//    img = src.clone();
//    w = img.cols;
//    h = img.rows;

//    threshold(img, biImg, 0, 255, THRESH_OTSU);
//    bitwise_not(biImg, biImg);
//    for(int i=0;i<w;++i)//畜牧所得到的图像边缘有黑色细线，去除掉
//    {
//        img.at<uchar>(0, i) = 255;
//        img.at<uchar>(h-1, i) = 255;
//    }
//    findContours(biImg, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
//    sort(contours.begin(), contours.end(), [](vector<Point> c1, vector<Point> c2)
//    { return contourArea(c1) > contourArea(c2); });

//    for(size_t i=0; i<contours.size(); ++i)         //从大到小搜索
//    {
//        S = contourArea(contours[i]);   L = arcLength(contours[i], true);   if(L==0)   continue;   roundness = (4*CV_PI*S)/pow(L, 2);
//        xm = min_element(contours[i].begin(), contours[i].end(), [](Point p1, Point p2){ return p1.x < p2.x; })->x;
//        xM = max_element(contours[i].begin(), contours[i].end(), [](Point p1, Point p2){ return p1.x < p2.x; })->x;
//        yM = max_element(contours[i].begin(), contours[i].end(), [](Point p1, Point p2){ return p1.y < p2.y; })->y;
//        ym = min_element(contours[i].begin(), contours[i].end(), [](Point p1, Point p2){ return p1.y < p2.y; })->y;

//        if(xm<=0 && yM-ym >= 0.8*HOLD_DIA && roundness<0.7 && S>0.1*HOLD_DIA*HOLD_DIA && S<w*h/3)          //吸持针对应的轮廓条件满足
//        {
//            biImg = Mat::zeros(src.size(), CV_8UC1);//填充轮廓内部
//            drawContours(biImg, contours, i, 255, -1);
//            element = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
//            morphologyEx(biImg, biImg, MORPH_OPEN, element);
//            for (int x = 0; x<xM; ++x)
//            {
//                min_y = h-1;
//                max_y = 0;
//                for (int y = 0; y<h; ++y)
//                    if (biImg.at<uchar>(y, x) == 255)
//                    {
//                        min_y = y;
//                        break;
//                    }
//                for(int y=h-1; y>min_y; --y)
//                    if(biImg.at<uchar>(y, x) == 255)
//                    {
//                        max_y = y;
//                        break;
//                    }
//                for(int y=min_y; y<max_y; ++y)
//                    biImg.at<uchar>(y, x) = 255;
//            }
////            imshow("after OPEN: hold pipette", biImg);
//            waitKey(1);
//            findContours(biImg, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
//            sort(contours.begin(), contours.end(), [](vector<Point> c1, vector<Point> c2)
//            { return contourArea(c1) > contourArea(c2); });
//            if(!contours.empty())
//                set(contours[0]);
//            break;                                  //找到一个即停止搜索
//        }
//    }
//}
