#include "injectpipette.h"
#include "oocytes.h"
#include "utils/objectdetector.h"

using namespace cv;
using namespace std;
InjectPipette::InjectPipette()
{
    ym = 0;         yM = HEIGHT-1;
    x0GLI = -1;     x0OCI = -1;
    xfGLI = -1;     xfOCI = -1;
    xGLI = -1;      xOCI = -1;
    reset();
    mhiOCI = MHI(240, 30, 40);
}
InjectPipette::InjectPipette(vector<Point> curContour)
{
    set(curContour);
}

void mask_roi(Mat& mask, int x0, int y0, int x1, int y1)
{
    for(int x=x0; x<x1; ++x)
        for(int y=y0; y<y1; ++y)
            if(x>0 && (x<mask.cols) && y>0 && (y<mask.rows))
                mask.at<uchar>(y, x) = 255;
}
void putMethodText(Mat img, string str, int h)
{
    putText(img,str,Point(0,h-1), FONT_HERSHEY_SIMPLEX,1,Scalar(0,0,200), 2, LINE_8);
}
//检测新环境下的注射针，清除已有信息
void InjectPipette::reset()
{
    isDetected = false;
    wasGLIDetected = false;

    contour.clear();
    stateOCI = NEAR_TIP;
    mhiOCI.reset();
    templateGLI.release();
    templateOCI.release();
    pointsGLI.clear();
    pointsOCI.clear();
    preBubbleImage.release();
}
//将轮廓 curContour 设置为注射针的轮廓，计算面积、上下左右边界、直径以及针尖位置
void InjectPipette::set(vector<Point> curContour)
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
    if(curContour.size()==4)
    {
        vector<Point> tmpContour;
        tmpContour.assign(curContour.begin(), curContour.end());
        sort(tmpContour.begin(), tmpContour.end(), [](Point p1, Point p2)    { return p1.x<p2.x;});
        tip = Point((tmpContour[0].x+tmpContour[1].x)/2, (tmpContour[0].y+tmpContour[1].y)/2);
    }
    else
        tip = Point(*min_element(contour.begin(), contour.end(), [](Point p1, Point p2){return p1.x<p2.x;}));
    isDetected = true;
}
//注射针检测：在右半侧图像自适应选择阈值，获得整幅二值图像；根据注射针尺寸做闭运算，连接微针的上下端；寻找轮廓（右端在图像最右端、圆度足够小、面积不太大也不太小、宽长比足够大
void InjectPipette::find(Mat src, vector<Rect> injectRects, bool firstDetection)
{
    reset();
    if(src.empty())
        return;
    Mat bgrImg, roi, biRoi, element;
    vector<vector<Point>> contours;         //提取出的所有轮廓
    vector<Point> curContour, tmpContour;
    Mat img, biImg, roiInj;
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
    if(injectRects.empty() && firstDetection)
    {
        if(isUsingSocket)
        {
            results = objectDetector->getRects(bgrImg);
            injectRects = results["injector_current"];//Injector
        }
        else
        {
            results = yolo->getRects(bgrImg);
            injectRects = results["Injector"];//Injector
        }
    }
    if(!injectRects.empty())
    {
        for(int i=0; i<injectRects.size(); ++i)
        {
            if(injectRects[i].x < 30)
                continue;//更可能是吸持针
            if(isDetected && injectRects[i].x >= xm)
                continue;//注射针更小的部分
            injectRects[i].width = INT_MAX;
            injectRects[i]&=Rect(0,0,src.cols, src.rows);
            roiInj = img(injectRects[i]).clone();
            int tmp = threshold(roiInj, biImg, 0, 255, THRESH_OTSU);
            threshold(roiInj, biImg, tmp, 255, THRESH_BINARY_INV);
            // imshow("biImg", biImg);

            vector<Point> unionContour;
            vector<Point> hull;
            findContours(biImg, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
            for(size_t j=0; j<contours.size(); ++j)
                for(size_t k=0; k<contours[j].size(); ++k)
                    unionContour.push_back(contours[j][k]);
            convexHull(unionContour, hull);
            for(size_t j=0; j<hull.size(); ++j)
            {
                hull[j].x += injectRects[i].x;
                hull[j].y += injectRects[i].y;
            }
            set(hull);
        }
    }
    // if(!injectRects.empty())
    // {
    //     injectRects[0]&=Rect(0,0,src.cols, src.rows);
    //     curContour.clear();
    //     curContour.push_back(Point(injectRects[0].x, injectRects[0].y));
    //     curContour.push_back(Point(injectRects[0].x+injectRects[0].width, injectRects[0].y));
    //     curContour.push_back(Point(injectRects[0].x+injectRects[0].width, injectRects[0].y+injectRects[0].height));
    //     curContour.push_back(Point(injectRects[0].x, injectRects[0].y+injectRects[0].height));
    //     set(curContour);
    // }
}
void InjectPipette::findGLI(Mat src, vector<Rect> gliRects, bool firstDetection)
{
    if(!isDetected || src.empty())
        return;
    //直接使用目标检测结果
    if(!gliRects.empty())
    {
        Rect rect = gliRects[0];
        for(int i=0;i<gliRects.size();++i)
            rect |= gliRects[i];
        xGLI = rect.x;
        return;
    }
    else if(isUsingSocket)
    {
        if(firstDetection)
        {
            map<string, vector<Rect>> results = objectDetector->getRects(src);
            gliRects = results["GL-Interface"];
            if(!gliRects.empty())
            {
                Rect rect = gliRects[0];
                for(int i=0;i<gliRects.size();++i)
                    rect |= gliRects[i];
                xGLI = rect.x;
                return;
            }
        }
        xGLI = WIDTH-1;
        return;
    }
    float matchTH = 0.8;
    int w=src.cols, h=src.rows, dxTH_H = 40;
    Mat img, showImg, element, roiImg;
    vector<vector<Point>> contours;
    Rect rect(xm, ym-3, w-xm, diameter+6);//略大于微针的区域作为ROI
    rect &= Rect(0,0,w,h);
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
    roiImg = img(rect).clone();

    int tmp_w = diameter, tmp_h = diameter;   double minVal, maxVal, matchVal;
    Point minLoc, maxLoc, matchLoc;
    //-------------------------------------水平梯度-------------------------------------
    int kw=15, posDx;   float dxVal, tipLen=0;  Mat roiDxImg, dxImg, kernel=Mat::ones(roiImg.rows, 2*kw, CV_32FC1);
    for(int y=0; y<kernel.rows; ++y)
        for(int x=0; x<kw; ++x)
            kernel.at<float>(y,x) = -1;
    roiDxImg = roiImg(Rect(tipLen, 0, w-rect.x-tipLen, rect.height)).clone();
    bitwise_not(roiDxImg, roiDxImg);
    equalizeHist(roiDxImg, roiDxImg);
    // GaussianBlur(roiDxImg, roiDxImg, Size(9, diameter-diameter%2+1), 3, 3);
    // element = getStructuringElement(MORPH_ELLIPSE, Size(1, 3));
    // morphologyEx(roiDxImg, roiDxImg, MORPH_CLOSE, element);
    // imshow("roiDxImg", roiDxImg);

    addWeighted(kernel, 1.0/(roiDxImg.rows*kw), kernel, 0, 0, kernel);
    filter2D(roiDxImg, dxImg, CV_32F, kernel, Point(kw, 0));    /*dxImg = abs(dxImg);*/
    minMaxLoc(dxImg, &minVal, &maxVal, &minLoc, &maxLoc);
    dxVal = maxVal;
    posDx = maxLoc.x+rect.x+tipLen;
    normalize(dxImg, dxImg, 0, 1, NORM_MINMAX);
    // imshow("dxImg", dxImg);
    //**************************************************************初始化**************************************************************
    if(!wasGLIDetected)
    {
        vGLI = 0;
        xGLI = posDx;
        putMethodText(showImg,"Dx", h);

        templateGLI = img(Rect(int(xGLI)-tmp_w/2, ym, tmp_w, tmp_h)&Rect(0,0,w,h)).clone();
        wasGLIDetected = true;
    }
    //*************************************************************初始化结束*************************************************************
    else
    {
        //-------------------------------------模板匹配-------------------------------------
        Mat matchedImg, matchResult;
        if(!templateGLI.empty())
        {
            matchTemplate(roiImg, templateGLI, matchResult, TM_CCOEFF_NORMED);
            minMaxLoc(matchResult, &minVal, &maxVal, &minLoc, &maxLoc);
            matchLoc = maxLoc;   matchVal = maxVal;
            matchedImg = roiImg(Rect(matchLoc.x, matchLoc.y, templateGLI.cols, templateGLI.rows)).clone();

            rectangle(showImg, Rect(rect.x+matchLoc.x, rect.y+matchLoc.y, templateGLI.cols, templateGLI.rows), Scalar(255, 0, 0), 2, 8, 0);
            // imshow("match-template", templateGLI);
            // imshow("match-result", matchedImg);
        }
        //----------------------------------近邻扫描--------------------------------------
        int posAround;
        if(!wasGLIDetected) xGLI=posDx;
        for(int x=0; x<dxImg.cols; ++x)
            if(x+rect.x<xGLI-tmp_w/2 || x+rect.x>xGLI+tmp_w/2)
                for(int y=0; y<dxImg.rows; ++y)
                    dxImg.at<float>(y, x) = 0;
        minMaxLoc(dxImg, &minVal, &maxVal, &minLoc, &maxLoc);
        posAround = maxLoc.x+rect.x;
        //***************************************汇总结果***************************************
        if(matchVal > matchTH)//模板匹配重合度高，采用模板匹配结果
        {
            if(round(xGLI)==rect.x+matchLoc.x+tmp_w/2)//运动缓慢可能造成静态漂移（位移<0.5pixel），采用邻近扫描法
            {
                vGLI = posAround-xGLI;
                xGLI = posAround;
                putMethodText(showImg,"Scan Around", h);
            }
            else
            {
                vGLI = rect.x+matchLoc.x+tmp_w/2-xGLI;
                xGLI = xGLI + vGLI;
                putMethodText(showImg,"Match Template", h);
            }
        }
        else if(dxVal>dxTH_H)//梯度十分明显，采用全局水平梯度检测结果
        {
            vGLI = posDx - xGLI;
            xGLI = posDx;
            putMethodText(showImg,"Dx", h);
        }
        else
        {
            vGLI = ((xGLI>(rect.x+w)/2)?w-1:rect.x) - xGLI;
            xGLI += vGLI;
            putMethodText(showImg,"Edge", h);
        }
    }
    templateGLI = img(Rect(int(xGLI)-tmp_w/2, ym, tmp_w, tmp_h)&Rect(0,0,w,h)).clone();
    preGLIimage = img.clone();

    line(showImg, Point(posDx, ym), Point(posDx, yM), Scalar(0, 0, 255), 1);
    line(showImg, Point(xGLI, ym+diameter/4), Point(xGLI, yM-diameter/4), Scalar(0, 0, 255), 2);
    putText(showImg,"Pos:"+to_string(int(xGLI))+";Vel:"+to_string(int(vGLI)),Point(1*w/4,3*h/4), FONT_HERSHEY_SIMPLEX,1,Scalar(0,0,0),2,LINE_8);
    putText(showImg,"Match Value:"+to_string(matchVal)+";dxVal: "+to_string(dxVal),Point(0,1*h/4), FONT_HERSHEY_SIMPLEX,1,Scalar(0,0,0),2,LINE_8);
    // imshow("showImg", showImg);
}
void InjectPipette::findOCI(Mat src, int x0, vector<Rect> ociRects, bool firstDetection)
{
    if(!isDetected || !currentOocyte->isDetected || src.empty())
        return;

    int count, w=src.cols, h=src.rows;
    Mat img, showImg, element, roiImg;
    vector<vector<Point>> contours;
    Rect rect(x0, ym-3, w-x0, diameter+6);//略大于微针的区域作为ROI
    int thAround = 150;                     //近邻扫描的梯度阈值

    Mat mask;       vector<Point2f> newPoints;      float velOPTF=0;
    vector<uchar> status;    vector<float> err;    Size winSize = Size(15,15);
    TermCriteria termcrit = TermCriteria(TermCriteria::COUNT|TermCriteria::EPS,10,0.03);

    int tmp_w = diameter/2, tmp_h = diameter;   double minVal, maxVal;
    Point minLoc, maxLoc, matchLoc;                 Mat matchedImg, matchResult;
    //直接使用目标检测结果
    if(!ociRects.empty())
    {
        Rect rect = ociRects[0];
        for(int i=0;i<ociRects.size();++i)
            rect |= ociRects[i];
        xOCI = rect.x+rect.width;
        return;
    }
    else if(isUsingSocket && firstDetection)
    {
        map<string, vector<Rect>> results = objectDetector->getRects(src);
        ociRects = results["Cytoplasm-Interface"];
        if(!ociRects.empty())
        {
            Rect rect = ociRects[0];
            for(int i=0;i<ociRects.size();++i)
                rect |= ociRects[i];
            xOCI = rect.x+rect.width;
            return;
        }
        else
        {
            xOCI = currentOocyte->xM;
            return;
        }
    }
    else
    {
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
        rect &= Rect(0,0, WIDTH,HEIGHT);
        roiImg = img(rect).clone();
        // imshow("roiImg", roiImg);
        //-------------------------------------模板匹配-------------------------------------
        if(!templateOCI.empty())
        {
            matchTemplate(roiImg, templateOCI, matchResult, TM_CCOEFF_NORMED);
            minMaxLoc(matchResult, &minVal, &maxVal, &minLoc, &maxLoc);        matchLoc = maxLoc;
            matchedImg = roiImg(Rect(matchLoc.x, matchLoc.y, templateOCI.cols, templateOCI.rows)).clone();

            rectangle(showImg, Rect(rect.x+matchLoc.x, rect.y+matchLoc.y, templateOCI.cols, templateOCI.rows), Scalar(255, 0, 0), 2, 8, 0);
            // imshow("match-template", templateOCI);   imshow("match-result", matchedImg);
        }
        //-------------------------------------光流检测-------------------------------------
        if(!pointsOCI.empty())
        {
            calcOpticalFlowPyrLK(preOCIimage, img, pointsOCI, newPoints, status, err, winSize, 2, termcrit, 0);
            int n = pointsOCI.size();
            for (size_t i = 0; i < pointsOCI.size(); ++i)
            {
                velOPTF += 1.0 * (newPoints[i].x - pointsOCI[i].x) / n;
                circle(showImg, newPoints[i], 2, Scalar(0, 255, 0));
            }
        }
        //-------------------------------------扫描线-------------------------------------
        Mat biRoi;      float posScan = x0;
        threshold(roiImg, biRoi, 0, 255, THRESH_OTSU);    bitwise_not(biRoi, biRoi);
        element = getStructuringElement(MORPH_ELLIPSE, Size(diameter/2, diameter/4));
        morphologyEx(biRoi, biRoi, MORPH_CLOSE, element);
        // imshow("biRoi", biRoi);
        for(int x=0; x<biRoi.cols; ++x)
        {
            count = 0;
            for(int y=0; y<biRoi.rows; ++y)
                if(biRoi.at<uchar>(y, x) == 255)
                    count ++;
            if(count < 0.6*injectPipette->diameter)		//宽度变小到阈值以下的位置认为是胞质面位置
            {
                posScan = x+rect.x;
                break;
            }
            if(x==biRoi.cols-1)
                posScan = w-1;			//宽度一直没有减小到阈值以下，认为是胞质面位置为最大值
        }
        //-------------------------------------水平梯度-------------------------------------
        int kw=20;   Mat dxImg, kernel=Mat::ones(roiImg.rows, 2*kw, CV_32FC1);
        for(int y=0; y<kernel.rows; ++y)
            for(int x=kw; x<2*kw; ++x)
                kernel.at<float>(y,x) = -1;
        addWeighted(kernel, 1.0/roiImg.rows*kw, kernel, 0, 0, kernel);
        filter2D(roiImg,dxImg, CV_32F, kernel, Point(kw, 0));
        //----------------------------------近邻扫描--------------------------------------
        float posAround = xOCI;
        for(int x=0; xOCI>=rect.x && round(xOCI-rect.x+x)<dxImg.cols && round(xOCI-rect.x-x)>=0; ++x)
        {
            if(dxImg.at<float>(0,round(xOCI-rect.x+x)) > thAround)		//宽度变小到阈值以下的位置认为是胞质面位置
            {
                posAround = round(xOCI+x);
                break;
            }
            if(dxImg.at<float>(0,round(xOCI-rect.x-x)) > thAround)
            {
                posAround = round(xOCI-x);
                break;
            }
            if(round(xOCI-rect.x+x)==dxImg.cols-1)
                posAround = w-1;			//宽度一直没有减小到阈值以下，认为是胞质面位置为最大值
        }
        // normalize(dxImg, dxImg, 0, 1, NORM_MINMAX);     imshow("dxImg", dxImg);
        //--------------------------------------MHI--------------------------------------
        Mat mhiImg;     int mhiXM = xOCI;
        if(mhiOCI.run(roiImg, mhiImg))
        {
            // imshow("MHI OCI before", mhiImg);
            element = getStructuringElement(MORPH_ELLIPSE, Size(diameter, 3));
            morphologyEx(mhiImg, mhiImg, MORPH_CLOSE, element);
            element = getStructuringElement(MORPH_ELLIPSE, Size(3, diameter/2));
            morphologyEx(mhiImg, mhiImg, MORPH_OPEN, element);
            // imshow("MHI OCI after", mhiImg);
            findContours(mhiImg, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
            sort(contours.begin(), contours.end(), [](vector<Point> c1, vector<Point> c2)
                 {return contourArea(c1)>contourArea(c2);});
            if(!contours.empty())
            {
                for(size_t j=0; j<contours[0].size(); ++j)
                {
                    contours[0][j].x += rect.x;
                    contours[0][j].y += rect.y;
                }
                mhiXM = max_element(contours[0].begin(), contours[0].end(), [](Point p1, Point p2){return p1.x<p2.x;})->x;
                drawContours(showImg, contours, 0, Scalar(0, 128, 200), 2);
            }
        }
        //***************************************汇总结果***************************************
        Rect rectMHIRight(xOCI-rect.x-3*diameter, ym+diameter/5-rect.y, 4*diameter, 0.6*diameter);
        Rect rectOCI(xOCI-rect.x-diameter/2, ym+diameter/5-rect.y, diameter, 0.6*diameter);
        Mat mhiRightImg, mhiOCIimg;
        if(!mhiImg.empty())
        {
            mhiRightImg = mhiImg(rectMHIRight&Rect(0,0,rect.width,rect.height)).clone();
            mhiOCIimg = mhiImg(rectOCI&Rect(0,0,rect.width,rect.height)).clone();
        }
        if(stateOCI == NEAR_TIP)//第一次进行检测
        {
            if(!mhiImg.empty() && mean(mhiRightImg)[0]>50)
            {
                vOCI = mhiXM - xOCI;
                xOCI = mhiXM;
                putMethodText(showImg,"MHI", h);
            }
            else
            {
                vOCI = posScan - xOCI;
                xOCI = posScan;
                putMethodText(showImg,"Scan", h);
            }
            if(xOCI<0)
                vOCI = 0;
        }
        else if(stateOCI == INJ_VIEW)//在针管内合适位置（离开透明带，特征较为稳定）
        {
            if(maxVal > 0.8)//模板匹配重合度高，采用模板匹配结果
            {
                if(mean(mhiOCIimg)[0]<10 && round(xOCI)==rect.x+matchLoc.x+tmp_w/2)//运动过于缓慢，为了避免每次运动小于0.5pixel 产生静态有偏误差，使用邻近搜索的结果
                {
                    vOCI = posAround-xOCI;
                    xOCI = posAround;
                    putMethodText(showImg,"Scan Around", h);
                }
                else
                {
                    vOCI = rect.x+matchLoc.x+tmp_w/2-xOCI;
                    xOCI = xOCI + vOCI;
                    putMethodText(showImg,"Match Template", h);
                }
            }
            else if(!contours.empty() && mean(mhiRightImg)[0]>50)//运动剧烈，使用 MHI检测的结果
            {
                vOCI = mhiXM - xOCI;
                xOCI = mhiXM;
                putMethodText(showImg,"MHI", h);
            }
            else
            {
                vOCI = velOPTF;
                xOCI = xOCI + vOCI;
                putMethodText(showImg,"Optical Flow", h);
                // vOCI = posScan-xOCI;
                // xOCI = posScan;
                // putMethodText(showImg,"Scan", h);
            }
            //一些初始化
            if(templateOCI.empty())
                templateOCI = img(Rect(int(xOCI)-tmp_w/2, ym, tmp_w, tmp_h)&Rect(0,0,w,h)).clone();
            if(pointsOCI.empty())
            {
                mask = Mat::zeros(img.size(), CV_8UC1);
                mask_roi(mask, xOCI-diameter, ym+diameter/4, xOCI, yM-diameter/4);
                goodFeaturesToTrack(img, pointsOCI, 5, 0.5, 1, mask);
            }
        }
        else if(stateOCI == OUT_VIEW)//在视野外，只能检测速度
        {
            if(maxVal > 0.75)
            {
                vOCI = rect.x+matchLoc.x - (xm+xM)/2;
                xOCI = xOCI + vOCI;
                putMethodText(showImg,"Match Template", h);
            }
            else
            {
                vOCI = velOPTF;
                xOCI = xOCI + vOCI;
                putMethodText(showImg,"Optical Flow", h);
            }
        }
    }

    templateOCI = img(Rect(int(xOCI)-tmp_w/2, ym, tmp_w, tmp_h)&Rect(0,0,w,h)).clone();
    mask = Mat::zeros(img.size(), CV_8UC1);
    mask_roi(mask, xOCI-diameter, ym+diameter/4, xOCI, yM-diameter/4);
    goodFeaturesToTrack(img, pointsOCI, 5, 0.5, 1, mask);
    //更新胞质面状态
    if(stateOCI != OUT_VIEW && xOCI>currentOocyte->xMZP && xOCI+diameter<w)
    {
        stateOCI = INJ_VIEW;
    }
    else if(xOCI+diameter>w)
    {
        stateOCI = OUT_VIEW;
        templateOCI = img(Rect((xm+xM)/2, ym, tmp_w, tmp_h)&Rect(0,0,w,h)).clone();
    }
    preOCIimage = img.clone();
    line(showImg, Point(xOCI, ym+diameter/4), Point(xOCI, yM-diameter/4), Scalar(0, 0, 255), 2);
    // putText(showImg,"Pos:"+to_string(xOCI),Point(w/2,h/2), FONT_HERSHEY_SIMPLEX,1,Scalar(0,0,0), 1, LINE_8);
    putText(showImg,"Pos:"+to_string(int(xOCI))+";Vel:"+to_string(int(vOCI)),Point(1*w/4,3*h/4), FONT_HERSHEY_SIMPLEX,1,Scalar(0,0,0),2,LINE_8);
    putText(showImg,"Match Value:"+to_string(maxVal),Point(0,1*h/4), FONT_HERSHEY_SIMPLEX,1,Scalar(0,0,0),2,LINE_8);
    // imshow("showImg", showImg);
}
void InjectPipette::findSomaInside(Mat src)
{
    if(!isDetected)
        return;
    Mat gray, roiImg, biRoi;
    Rect roiRect;
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    float S, C, roundness;
    if(src.channels()==3)
        cvtColor(src, gray, COLOR_BGR2GRAY);
    else
        gray = src.clone();
    if(tip.y<0.5*(ym+yM))//◥
        roiRect = Rect(tip.x-diameter, ym, 2*diameter, 2*diameter);
    else//◢
        roiRect = Rect(tip.x-diameter, yM-2*diameter, 2*diameter, 2*diameter);
    roiRect &= Rect(0, 0, src.cols, src.rows);
    roiImg = gray(roiRect);
    float th = threshold(roiImg, biRoi, 0, 255, THRESH_OTSU);
    threshold(roiImg, biRoi, th*1.05, 255, THRESH_BINARY_INV);

    // imshow("biRoi", biRoi);
    Mat element = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
    morphologyEx(biRoi, biRoi, MORPH_CLOSE, element);
    // imshow("after close", biRoi);
    findContours(biRoi,contours,hierarchy,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE,Point());
    for(int i=contours.size()-1; i>=0; --i)
    {
        S = contourArea(contours[i])*Pixel2Um*Pixel2Um;
        C = arcLength(contours[i], true)*Pixel2Um;
        roundness = 4*M_PI*S/(C*C);
        // qDebug()<<roundness;
        if(!(roundness>0.3 && S>40 && S<1000))
        {
            contours.erase(contours.begin()+i);
            // m = moments(contour);
            // if(m.m00==0)
            //     continue;

            // p = Point2f(m.m10/m.m00, m.m01/m.m00);
        }
    }
    sort(contours.begin(), contours.end(), [](vector<Point> c1, vector<Point> c2)
         { return contourArea(c1) > contourArea(c2); });
    if(contours.empty())
        return;

    // 显示结果
    Mat showImg = roiImg.clone();
    cvtColor(showImg, showImg, COLOR_GRAY2BGR);
    drawContours(showImg, contours, 0, Scalar(0, 0, 255), 1);
    // imshow("inj-soma", showImg);
}
void InjectPipette::findSomaOutside(Mat src)
{

}
//检测 气压变化过程中 是否产生了气泡
void InjectPipette::setBeforeBubble(Mat src)//产生气泡前保存正常情况的图片与平均像素值，要求输入灰度图
{
    preBubbleImage = src.clone();
    preBubbleValue = mean(src)[0];
}
bool InjectPipette::detectGasColumn(Mat src)//检测是否产生气柱
{
    assert(src.rows==HEIGHT&&src.cols==WIDTH);
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
    if(!isDetected)
        find(src);
    vector<Rect> gasRects;
    if(isUsingSocket)
    {
        map<string, vector<Rect>> results = objectDetector->getRects(src);
        gasRects = results["GL-Interface"];
        Rect injRect(xm, ym, xM-xm, yM-ym);
        for(auto gasRect: gasRects)
        {
            Rect intersection;
            double interArea, gasRatio;
            intersection = gasRect & injRect;
            interArea = intersection.area();
            if(gasRect.area()<=0)
                gasRatio = 0;
            else
                gasRatio = interArea/gasRect.area();
            qDebug()<<gasRatio<<interArea<<gasRect.area();
            if(gasRatio>0.02)
                return true;
        }
    }
    else if(isDetected)
    {
        Rect roi(WIDTH-31, ym, 30, diameter);
        Mat roiImg = img(roi).clone();
        int meanValue = mean(roiImg).val[0];
        imshow("gasVolumn", roiImg);
        qDebug()<<meanValue;
        if(meanValue<55)
            return true;
        else
            return false;
    }
    return false;
}
bool InjectPipette::detectBubble(Mat src, float p)//检测是否产生气泡
{
    if(preBubbleImage.empty())
    {
        setBeforeBubble(src);
        return false;
    }
    Mat img, showImg;
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
    if(p==INFINITY)//压强值无效
    {
        if(preBubbleValue-mean(img)[0]>5)
        {
            qDebug()<<"Notice the Bubbles!";
            preBubbleImage.release();
            return true;
        }
    }
    else if(p>1000)//压强较大时才可能产生气泡
    {

    }
    return false;
}
//删除卵细胞 oocyte 右端左侧的轮廓点
void InjectPipette::separateFromOocyte(Oocyte* oocyte, Mat src)
{
    if(!isDetected)
        return;
    vector<Point> tmpContour;                                   //删除轮廓点时使用的临时变量
    tmpContour.assign(contour.begin(), contour.end());

    xm = oocyte->xM > xm? oocyte->xM:xm;
    for(int j = tmpContour.size()-1; j>=0; --j)
        if(tmpContour[j].x<=xm)                                 //删除掉该轮廓中细胞左侧的点
            tmpContour.erase(tmpContour.begin()+j);
    if(tmpContour.empty())                                      //轮廓被清空，判定为未检测到
    {
        isDetected = false;
        return;
    }
    set(tmpContour);
    separateFromContaminants(src);
}
//检测注射针的上下两条直线，删除直线外的点
void InjectPipette::separateFromContaminants(Mat src)
{
    Mat biImg, showImg, isSmall;                                //二分类中位于上侧(Small，离原点近)的标志
    vector<vector<Point>> contours;
    vector<Point> tmpContour;
    vector<Vec3f> lines;                                        //清除针尖两侧的杂质
    vector<float> lineYVals;                                    //直线在x=w处的y值
    vector<bool>  isReserved;                                   //保留轮廓点的标志
    Vec3f l1, l2;
    int min_y, max_y, x, y, w = src.cols, h = src.rows;
    float rho, theta, cTheta, sTheta;
    Mat centers(2, 1, CV_32F);
    bool isBelowHigh, isAboveLow;                               //当前轮廓点在注射针上侧轮廓直线下方、下侧轮廓直线上方的标志
    cvtColor(src, showImg, COLOR_GRAY2BGR);
    biImg = Mat::zeros(src.size(), CV_8UC1);
    contours.push_back(contour);
    drawContours(biImg, contours, 0, 255);
    HoughLines(biImg, lines, 1, CV_PI/180, WIDTH/10);           //TODO
    for(int i=lines.size()-1; i>=0; --i)
        if(abs(lines[i][1]-CV_PI/2)>CV_PI/36)                   //删除不够平的线条 TODO
            lines.erase(lines.begin()+i);


    sort(lines.begin(), lines.end(), [](Vec3f l1, Vec3f l2){return l1[0] < l2[0];});
    for( size_t i = 0; i < lines.size(); i++ )//画线
    {
        rho = lines[i][0], theta = lines[i][1];
        Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        line( showImg, pt1, pt2, Scalar(0,0,255), 1, LINE_AA);
    }
    //    imshow("line Inj", showImg);

    //对直线进行二分类聚类
    //计算 x=w处的y值
    if(lines.size()>=2)
    {
        for(size_t i=0; i<lines.size(); ++i)
            lineYVals.push_back((lines[i][0]-w*cos(lines[i][1]))/sin(lines[i][1]));//由于前面保证了角度在 π/2-π/36 ~ π/2+π/36之间，sin(theta)≠0
        TermCriteria criteria = TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 10, 0.1);
        kmeans(lineYVals, 2, isSmall, criteria, 3, KMEANS_PP_CENTERS, centers);
        //删除直线外侧的点

        tmpContour.assign(contour.begin(), contour.end());
        for(size_t i = 0; i<tmpContour.size(); ++i)
        {
            isBelowHigh = false;
            isAboveLow  = false;
            for(size_t j=0; j<lines.size(); ++j)
            {
                rho = lines[j][0];      theta = lines[j][1];    cTheta = cos(theta);    sTheta = sin(theta);
                x = tmpContour[i].x;    y = tmpContour[i].y;
                if((isSmall.at<int>(j)^(centers.at<int>(0)>centers.at<int>(1)))&&rho-(x*cTheta+y*sTheta)>0) isBelowHigh = true;// isSmall[j]=0 centers[0]<centers[1]  || isSmall[j]=1 centers[0]>centers[1]
                if((isSmall.at<int>(j)^(centers.at<int>(0)<centers.at<int>(1)))&& x*cTheta+y*sTheta-rho >0) isAboveLow  = true;// isSmall[j]=0 centers[0]>centers[1]  || isSmall[j]=1 centers[0]<centers[1]
            }
            isReserved.push_back(isBelowHigh&&isAboveLow);
        }
        for(int i = tmpContour.size()-1; i>=0; --i)
            if(!isReserved[i])
                tmpContour.erase(tmpContour.begin()+i);
    }
    if(tmpContour.empty())                                      //轮廓被清空，判定为未检测到
    {
        isDetected = false;
        return;
    }
    contours.clear();
    contours.push_back(tmpContour);
    biImg = Mat::zeros(src.size(), CV_8UC1);                    //填充针尖内部，使上下轮廓中间没有缝隙
    drawContours(biImg, contours, 0, 255, -1);
    for (int x = xm; x<w; ++x)
    {
        min_y = h-1;
        max_y = 0;
        for (int y = 0; y<h; ++y)
            if (biImg.at<uchar>(y, x) == 255)
            {
                min_y = y;
                break;
            }
        for(int y=h-1; y>=0; --y)
            if(biImg.at<uchar>(y, x) == 255)
            {
                max_y = y;
                break;
            }
        for(int y=min_y; y<max_y; ++y)
            biImg.at<uchar>(y, x) = 255;
    }
    //    imshow("inject pipette", biImg);
    findContours(biImg, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    sort(contours.begin(), contours.end(), [](vector<Point> c1, vector<Point> c2)
         { return contourArea(c1) > contourArea(c2); });
    if(!contours.empty())
        set(contours[0]);
}
