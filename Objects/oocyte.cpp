#include "oocyte.h"
#include "holdpipette.h"
#include "injectpipette.h"
#include "Components/microscope.h"
#include "utils/neuralnetwork.h"
#include "utils/objectdetector.h"
using namespace cv;
using namespace std;


Oocyte::Oocyte()
{
    reset();
    tracker = TrackerMIL::create();
    winSize = Size(5,5);
    termcrit = TermCriteria(TermCriteria::COUNT|TermCriteria::EPS,10,0.03);
}
void Oocyte::find(Mat src, vector<Rect> oocytesRects, bool firstDetection)
{
    reset();                                //清除信息、设置为未检测到
    if(src.empty())
        return;
    Mat bgrImg, img, biRoi;                 //当前图像、二值图、闭运算后的二值图
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
        {
            results = objectDetector->getRects(bgrImg);
            oocytesRects = results["oocyte_current"];
        }
        else
        {
            results = yolo->getRects(bgrImg);
            oocytesRects = results["Oocyte"];
        }
    }
    if(oocytesRects.empty())
        return;

    Mat showImg, roiImg, element;           //交互图像、形态学变换元素
    vector<vector<Point>> contours;         //提取出的所有轮廓
    vector<Point> curContour;
    int idx=0, max_count=-1, area;
    showImg = bgrImg.clone();
    for(size_t i=0; i<oocytesRects.size(); ++i)
    {
        oocytesRects[i] &= Rect(0, 0, src.cols, src.rows);
        // if(holdPipette->isDetected)
        // {
        //     oocytesRects[i].width -= (max(oocytesRects[i].x, holdPipette->xM)-oocytesRects[i].x);
        //     oocytesRects[i].x = max(oocytesRects[i].x, holdPipette->xM);
        // }
        if(holdPipette->isDetected)
        {

        }
        area = oocytesRects[i].width*oocytesRects[i].height;
        if(area>max_count)
        {
            idx = i;
            max_count = area;
        }
    }
    if(holdPipette->isDetected)
    {
        int idx_near_hold=0;
        float min_dis_hold=1e9, dis_hold;
        for(size_t i=0; i<oocytesRects.size(); ++i)
        {
            oocytesRects[i] &= Rect(0, 0, src.cols, src.rows);
            dis_hold = sqrt((oocytesRects[i].x-holdPipette->xM)*(oocytesRects[i].x-holdPipette->xM) +
                            (oocytesRects[i].y-holdPipette->ym)*(oocytesRects[i].y-holdPipette->ym));
            if(dis_hold<min_dis_hold)
            {
                idx_near_hold = i;
                min_dis_hold = dis_hold;
            }
        }
        idx = idx_near_hold;
    }
    Rect rect = oocytesRects[idx];
    rect &= Rect(0, 0, src.cols, src.rows);
    rectZP = rect;
    //设置透明带
    vector<Point> contourZP;
    contourZP.push_back(Point(rect.x, rect.y));
    contourZP.push_back(Point(rect.x+rect.width, rect.y));
    contourZP.push_back(Point(rect.x+rect.width, rect.y+rect.height));
    contourZP.push_back(Point(rect.x, rect.y+rect.height));
    setZP(contourZP);
    isDetected = true;

    roiImg = CurrentGray(oocytesRects[idx]).clone();
    threshold(roiImg, biRoi, 0, 255, THRESH_OTSU);
    bitwise_not(biRoi, biRoi);
    findContours(biRoi, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    sort(contours.begin(), contours.end(), [](vector<Point> c1, vector<Point> c2)
         {return contourArea(c1)>contourArea(c2);});
    if(contours.size()>0)
        curContour = contours[0];
    for(size_t i=0; i<curContour.size(); ++i)
    {
        curContour[i].x += oocytesRects[idx].x;
        curContour[i].y += oocytesRects[idx].y;
    }
    contours.clear();
    contours.push_back(curContour);
    drawContours(showImg, contours, 0, Scalar(0, 255, 255), 2);
    //平滑轮廓 TODO 修改阈值 自适应细胞亮度 面积
    Mat biOocyte = Mat::zeros(img.size(), CV_8UC1);
    drawContours(biOocyte, contours, 0, Scalar(255), -1);
    element = getStructuringElement(MORPH_ELLIPSE, Size(0.2*OOCYTE_DIA, 0.2*OOCYTE_DIA));//TODO
    morphologyEx(biOocyte, biOocyte, MORPH_OPEN, element);
    findContours(biOocyte, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    if(contours.empty())
    {
        set(contourZP, cv::Point2f(microscope->getRelX(), microscope->getRelY()));
        qDebug()<<"This oocyte is bad!";
        // QMessageBox* msgBox = new QMessageBox(QMessageBox::Icon::Information, "info", "这个卵检测不到细胞质！");//参数 Icon icon, const QString &title, const QString &text,
        // msgBox->setModal(false);//设置为非模态
        // msgBox->setAttribute(Qt::WA_DeleteOnClose); //关掉消息框后删除指针
        // msgBox->show();
        return;
    }
    sort(contours.begin(), contours.end(), [](vector<Point> c1, vector<Point> c2)
         { return contourArea(c1) > contourArea(c2); });
    curContour = contours[0];
    drawContours(showImg, contours, 0, Scalar(255, 0, 255), 2);
    // imshow("biOocyte", biOocyte);
    set(contours[0], cv::Point2f(microscope->getRelX(), microscope->getRelY()));
}
bool Oocyte::findPB(Mat src, vector<Rect> PolarBodyRects, bool firstDetection)
{
    isPBDetected = false;
    if(src.empty()||(!isDetected))
        return false;
    Mat bgrImg, img, cellImg;                 //当前图像、二值图、闭运算后的二值图
    // Rect cell_roi(xm - 0.2*abs(xM - xm), ym-0.2*abs(yM - ym), 1.4*abs(xM - xm), 1.4*abs(yM - ym));
    Rect cell_roi(0,0,0,0);
    map<string, vector<Rect>> results;
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
    if(PolarBodyRects.empty() && firstDetection)
    {
        if(isUsingSocket)
        {
            results = objectDetector->getRects(bgrImg);
            PolarBodyRects = results["polar_body"];
        }
        else
        {
            results = yoloPB->getRects(bgrImg);
            PolarBodyRects = results["Polarbody"];
        }

        // cell_roi &= Rect(0,0, w, h);
        // cellImg = bgrImg(cell_roi);
        // if(cellImg.empty())
        //     return false;
        // // imshow("roi", cellImg);
        // results = yoloPB->runPolar(cellImg);
    }
    if(PolarBodyRects.empty())
        return false;
    else
    {
        int x0,y0,x1,y1;
        for(int i=0; i<PolarBodyRects.size(); ++i)
        {
            x0 = PolarBodyRects[i].x;
            y0 = PolarBodyRects[i].y;
            x1 = x0 + PolarBodyRects[i].width;
            y1 = y0 + PolarBodyRects[i].height;
            if(x0>=xmZP && x1<=xMZP && y0>=ymZP && y1<=yMZP)
            {
                isPBDetected = true;
                centerPB.x = cell_roi.x + PolarBodyRects[i].x + 0.5*PolarBodyRects[i].width;
                centerPB.y = cell_roi.y + PolarBodyRects[i].y + 0.5*PolarBodyRects[i].height;
                rectPB = Rect(cell_roi.x + PolarBodyRects[i].x, cell_roi.y + PolarBodyRects[i].y, PolarBodyRects[i].width, PolarBodyRects[i].height);
                // circle(bgrImg, centerPB, 2, Scalar(0, 0, 255), 2);        imshow("PB", bgrImg);
                return true;
            }
        }
    }
}
void Oocyte::findZP(Mat src)
{

}
void Oocyte::reset()
{
    isDetected = false;
    isPBDetected = false;
    isZPDetected = false;

    contour.clear();
    contourZP.clear();

    innerPoints.clear();
    pairs.clear();
    isPairValid.clear();
    weights.clear();
}
void Oocyte::set(std::vector<cv::Point> curContour, Point2f left_top_up)
{
    if(curContour.empty())
        return;
    contour.assign(curContour.begin(), curContour.end());
    area = contourArea(contour);
    xm = min_element(contour.begin(), contour.end(), [](Point p1, Point p2){return p1.x<p2.x;})->x;
    xM = max_element(contour.begin(), contour.end(), [](Point p1, Point p2){return p1.x<p2.x;})->x;
    ym = min_element(contour.begin(), contour.end(), [](Point p1, Point p2){return p1.y<p2.y;})->y;
    yM = max_element(contour.begin(), contour.end(), [](Point p1, Point p2){return p1.y<p2.y;})->y;
    radius = (xM-xm+yM-ym)/4;
    preImg = CurrentGray.clone();
    Moments m = moments(contour);
    if(m.m00!=0)
        center = Point2f(m.m10/m.m00, m.m01/m.m00);
    else
        center = Point2f((xm+xM)/2, (ym+yM)/2);
    centerUm = Point2f(left_top_up.x+(center.x-WIDTH/2)*Pixel2Um, left_top_up.y+(center.y-HEIGHT/2)*Pixel2Um);

    isDetected = true;
}
void Oocyte::setZP(std::vector<cv::Point> contour)
{
    if(contour.empty())
        return;
    this->contourZP = contour;
    xmZP = min_element(contour.begin(), contour.end(), [](Point p1, Point p2){return p1.x<p2.x;})->x;
    xMZP = max_element(contour.begin(), contour.end(), [](Point p1, Point p2){return p1.x<p2.x;})->x;
    ymZP = min_element(contour.begin(), contour.end(), [](Point p1, Point p2){return p1.y<p2.y;})->y;
    yMZP = max_element(contour.begin(), contour.end(), [](Point p1, Point p2){return p1.y<p2.y;})->y;
    radiusZP = (xMZP-xmZP+yMZP-ymZP)/4;
    centerZP = Point2f((xmZP+xMZP)/2, (ymZP+yMZP)/2);
    isZPDetected = true;
}

void Oocyte::initInnerPoints()//初始化卵细胞内的跟踪点；要求卵细胞轮廓已经被检测到
{
    innerPoints.clear();
    pairs.clear();
    isPairValid.clear();
    weights.clear();
    if(contour.empty())
        return;
    l0 = int(sqrt(area/100));
    int preId = -1, curId=-1; //上一个|当前x值中，innerPoints中索引最小的
    bool isIdSet;
    int idBias;
    for(int x=xm; x<=xM; x+=l0)
    {
        isIdSet = false;//每一个x值进行一次设置
        for(int y=ym; y<=yM; y+=l0)
            if (pointPolygonTest(contour, Point2f(x,y), false)>=0)
            {
                innerPoints.push_back(Point2f(x, y));
                if(!isIdSet)
                {
                    isIdSet = true;
                    curId = innerPoints.size()-1;
                }
                else
                {
                    pairs.push_back(Point(innerPoints.size()-2, innerPoints.size()-1));//与上一个点相连
                    isPairValid.push_back(true);
                    weights.push_back(1);//TODO 修改权重
                }
                if(preId>=0)
                {
                    idBias = round((y-innerPoints[preId].y)/l0);
                    if(idBias>=0 && idBias<(curId-preId))//与轮廓内左侧点相连
                    {
                        pairs.push_back(Point(preId+idBias, innerPoints.size()-1));
                        isPairValid.push_back(true);
                        weights.push_back(1);//TODO 修改权重
                    }
                }
            }
        preId = curId;
    }
    Mat showImg = CurrentBGR.clone();
    for(int i=0; i<pairs.size(); ++i)
    {
        weights[i] = 1.0*weights[i]/pairs.size();
        line(showImg, innerPoints[pairs[i].x], innerPoints[pairs[i].y], 255*weights[i]);
    }
    imshow("Init Strain Points", showImg);waitKey(1);
}

void Oocyte::calcStrain(Mat src)
{
    if(innerPoints.empty()||preSImg.empty())
    {
        initInnerPoints();
        preSImg = src.clone();
        strain = 0;
        return;
    }
    calcOpticalFlowPyrLK(preSImg, src, innerPoints, newPoints, status, err, winSize, 5, termcrit, 0);
    // 对每一对边，计算工程应变
    float dx, dy, dist, sumWeights = 1.0;
    Point2f tmpP;
    strain = 0;
    for(int i=0; i<pairs.size(); ++i)
    {
        if(!isPairValid[i])
            continue;
        if(pointPolygonTest(contour, newPoints[pairs[i].x], false)<0 || pointPolygonTest(contour, newPoints[pairs[i].y], false)<0)
        {
            sumWeights -= weights[i];
            isPairValid[i] = false;
        }
        dx = newPoints[pairs[i].x].x - newPoints[pairs[i].y].x;
        dy = newPoints[pairs[i].x].y - newPoints[pairs[i].y].y;
        dist = sqrt(dx*dx+dy*dy);
        strain += weights[i]*abs(dist-l0)/l0;
    }
    strain /= sumWeights;
    for(int i=0; i<pairs.size(); ++i)
        if(isPairValid[i])
            weights[i]/=sumWeights;
    //更新点和历史图像
    innerPoints.assign(newPoints.begin(), newPoints.end());
    preSImg = src.clone();
}
