#include "oocytes.h"

using namespace cv;
using namespace std;
Oocytes::Oocytes()
{
    isDetected = false;
    oocytes.clear();
}
//void Oocytes::find(Mat src, Point2f left_top_um)
//{
//    oocytes.clear();
//    if(src.empty())
//        return;
//    Mat bgrImg, roi, biRoi, element;
//    vector<vector<Point>> contours;         //提取出的所有轮廓
//    vector<Point> contour;
//    Oocyte oocyte;
//    if(src.channels()==1)
//        cvtColor(src, bgrImg, COLOR_GRAY2BGR);
//    else
//        bgrImg = src.clone();
//    map<string, vector<Rect>> results = yolo->run(bgrImg);
//    vector<Rect> oocytesRect = results["Oocyte"];
//    for(size_t i=0; i<oocytesRect.size(); ++i)
//    {
////        roi = src(oocytesRect[i]).clone();
////        threshold(roi, biRoi, 0, 255, THRESH_OTSU);
////        bitwise_not(biRoi, biRoi);
////        imshow("biImg", biRoi);
////        element = getStructuringElement(MORPH_ELLIPSE, Size(SOMA_DIA, SOMA_DIA));
////        morphologyEx(biRoi, biRoi, MORPH_CLOSE, element);
////        element = getStructuringElement(MORPH_ELLIPSE, Size(OOCYTE_DIA, OOCYTE_DIA));
////        morphologyEx(biRoi, biRoi, MORPH_OPEN, element);
////        imshow("morpho", biRoi);
////        waitKey(1);
////        findContours(biRoi, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
////        sort(contours.begin(), contours.end(), [](vector<Point> c1, vector<Point> c2)
////        {return contourArea(c1)>contourArea(c2);});
////        contour = contours[0];
////        for(size_t j=0; j<contour.size(); ++j)
////        {
////            contour[j].x += oocytesRect[i].x;
////            contour[j].y += oocytesRect[i].y;
////        }
//        //或者直接将矩形框作为轮廓
//        contour.clear();
//        contour.push_back(Point(oocytesRect[i].x, oocytesRect[i].y));
//        contour.push_back(Point(oocytesRect[i].x+oocytesRect[i].width, oocytesRect[i].y));
//        contour.push_back(Point(oocytesRect[i].x+oocytesRect[i].width, oocytesRect[i].y+oocytesRect[i].height));
//        contour.push_back(Point(oocytesRect[i].x, oocytesRect[i].y+oocytesRect[i].height));

//        oocyte.set(contour, left_top_um);
//        oocytes.push_back(oocyte);
//    }
//    sort(oocytes.begin(), oocytes.end(), [](Oocyte o1, Oocyte o2)
//    { return o1.area > o2.area; });
//}
void Oocytes::find(Mat src, vector<Rect> oocytesRects, bool firstDetection)
{
    reset();                                //清除信息、设置为未检测到
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
    if(oocytesRects.empty() && firstDetection)
    {
        if(isUsingSocket)
            results = objectDetector->getRects(bgrImg);
        else
            results = yolo->getRects(bgrImg);
        oocytesRects = results["Oocyte"];
    }
    if(oocytesRects.empty())
        return;

    Oocyte oocyte;
    Rect rect;
    for(int i=0; i<oocytesRects.size();++i)
    {
        rect = oocytesRects[i];
        rect &= Rect(0, 0, src.cols, src.rows);
        //设置透明带
        vector<Point> contourZP;
        contourZP.push_back(Point(rect.x, rect.y));
        contourZP.push_back(Point(rect.x+rect.width, rect.y));
        contourZP.push_back(Point(rect.x+rect.width, rect.y+rect.height));
        contourZP.push_back(Point(rect.x, rect.y+rect.height));
        oocyte.setZP(contourZP);

        oocyte.set(contourZP, cv::Point2f(microscope->getRelX(), microscope->getRelY()));
        oocytes.push_back(oocyte);
    }
    sort(oocytes.begin(), oocytes.end(), [](Oocyte o1, Oocyte o2)
         { return o1.center.y < o2.center.y; });
}
void Oocytes::find(Mat src, Point2f left_top_um)
{
    reset();
    if(src.empty())
        return;
    // 默认img为灰度图
    Oocyte oocyte;                          //一幅图中可能有多个卵细胞，临时存储
    Mat img, showImg, roiImg, element;      //交互图像、形态学变换元素
    Mat biImg, biRoi, closeBiImg;           //当前图像、二值图、闭运算后的二值图
    Mat biOocyteImg;                        //卵细胞区域二值图：去除左侧吸持针和右侧注射针
    int w, h, count;                        //图像的高度、宽度; 计数器
    int x0, y0, r, area;                    //圆的中心、半径
    float cut_ratioX = 1, cut_ratioY=1;     //精细检测时的裁剪区域放大比例
    vector<vector<Point>> contours;         //提取出的所有轮廓
    vector<Point> contour;
    vector<Vec3f> circles;                  //检测到的圆
    if(src.channels()==1)
    {
        cvtColor(src, showImg, COLOR_GRAY2BGR);
        img = src.clone();
    }
    else
    {
        showImg = src.clone();
        cvtColor(src, img, COLOR_BGR2GRAY);
    }
    h = img.rows;
    w = img.cols;
    //------------------------------------- 寻找卵细胞 -------------------------------------
    //卵细胞具有 呈球形、较背景颜色深的特点，据此可以用图像处理方法寻找
    threshold(img, biImg, 0, 255, THRESH_OTSU);
    bitwise_not(biImg, biImg);
    // imshow("binary", biImg);
    //霍夫圆检测粗定位                                最小距离                边缘点数阈值      最小半径        最大半径
    HoughCircles(img, circles, HOUGH_GRADIENT, 1, 0.5*OOCYTE_DIA, 100, 0.5*OOCYTE_DIA, 0.4*OOCYTE_DIA, 0.7*OOCYTE_DIA);// 一定大小范围内寻找
    if(circles.empty())
        return;
    sort(circles.begin(), circles.end(), [](Vec3f c1, Vec3f c2)
    {return c1[2]>c2[2];});
    for (size_t i = 0; i < circles.size(); i++)
    {//在圆中
        x0 = circles[i][0];
        y0 = circles[i][1];
        r = circles[i][2];
//        if(x0-r<0 || x0+r>WIDTH || y0-r<0 || y0+r>HEIGHT)//排除在视野中不完整的圆
//            continue;
        area = 2*r*r;               //内接正方形的面积
        count = 0;                  //内接正方形内的深色区域面积
        for(int d=0; d<=r; ++d)
            for(int d1=0; d1<=d; ++d1)
            {
                int d2 = d-d1;
                if(y0+d2<h  && x0+d1<w  && biImg.at<uchar>(y0+d2, x0+d1) > 0)
                    ++count;
                if(y0+d2<h  && x0-d1>=0 && biImg.at<uchar>(y0+d2, x0-d1) > 0)
                    ++count;
                if(y0-d2>=0 && x0+d1<w  && biImg.at<uchar>(y0-d2, x0+d1) > 0)
                    ++count;
                if(y0-d2>=0 && x0-d1>=0 && biImg.at<uchar>(y0-d2, x0-d1) > 0)
                    ++count;
            }
        if(count >= 0.7*area)       //筛选覆盖深色区域多的圆
        {
            //在小区域内寻找轮廓、去噪、平滑边缘（开运算）
            int x1 = x0 - r * cut_ratioX > 0 ? x0 - r * cut_ratioX : 0;
            int y1 = y0 - r * cut_ratioY > 0 ? y0 - r * cut_ratioY : 0;
            int x2 = x0 + r * cut_ratioX < w ? x0 + r * cut_ratioX : w - 1;
            int y2 = y0 + r * cut_ratioY < h ? y0 + r * cut_ratioY : h - 1;
            Rect rect(x1, y1, x2-x1, y2-y1);
            rect &= Rect(0, 0, src.cols, src.rows);
            roiImg = img(rect).clone();
            threshold(roiImg, biRoi, 0, 255, THRESH_OTSU);
            bitwise_not(biRoi, biRoi);
            findContours(biRoi, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
            if(contours.empty())
                continue;
            sort(contours.begin(), contours.end(), [](vector<Point> c1, vector<Point> c2)
            {return contourArea(c1)>contourArea(c2);});
            contour = contours[0];
            for(size_t j=0; j<contour.size(); ++j)
            {
                contour[j].x += x1;
                contour[j].y += y1;
            }

            contours.clear();
            contours.push_back(contour);
            drawContours(showImg, contours, 0, Scalar(0, 255, 255), 2);
            circle(showImg, Point(circles[i][0], circles[i][1]), circles[i][2], Scalar(0, 0, 255), 1, 8, 0);

            oocyte.set(contour, left_top_um);
            oocytes.push_back(oocyte);
            for(int i=0; i<oocytes.size()-1; i++)
                if(pointPolygonTest(oocytes[i].contour, oocyte.center, false)>0)
                {
                    oocytes.pop_back();
                }
        }
    }
    int rh, rw; //resized height & width;
    float k = min(0.8*1080/h, 0.8*1920/w);
    rh = h*k;rw = w*k;
    resize(showImg, showImg, Size(rw, rh));
    imshow("Oocytes Contour", showImg);
    waitKey(1);
    sort(oocytes.begin(), oocytes.end(), [](Oocyte o1, Oocyte o2)
    { return o1.center.y < o2.center.y; });
}
void Oocytes::reset()
{
    isDetected = false;
    oocytes.clear();
}
