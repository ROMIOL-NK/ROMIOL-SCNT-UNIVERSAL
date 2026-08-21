#include "stepbuildmap.h"
using namespace std;
using namespace cv;
StepBuildMap::StepBuildMap(){}

WORKSTEP StepBuildMap::run()
{
    isWorking = true;

    //添加处理代码: 在 4 倍镜下：首先检测针尖位置，下移得到距底部的高度，设置安全范围，将其移出视野 移动平台构建地图 检测卵母细胞，排序；TODO: 怎样排序吸细胞时对其它细胞影响最小
    //先将微针移出视野
    injectPipette->find(CurrentBGR);
    if(injectPipette->isDetected)
    {
        moveInjWait(0, 0, -areaMoveHeight, MANIP_VEL);
        moveInjWait(0, (0.5*HEIGHT-injectPipette->tip.y-1.0*injectPipette->diameter)*Pixel2Um, 0, MANIP_VEL);
        moveInjWait((WIDTH-injectPipette->xm)*Pixel2Um, 0, 0, MANIP_VEL);
    }
    holdPipette->find(CurrentBGR);
    if(holdPipette->isDetected)
    {
        // emit moveHold(0, 0, -areaMoveHeight, MANIP_VEL);
        // emit moveHold(0, (0.6*HEIGHT-holdPipette->tip.y)*Pixel2Um, 0, MANIP_VEL);
        // emit moveHold(-holdPipette->xM*Pixel2Um-200, 0, 0, MANIP_VEL);
        moveHoldWait(0, 0, -areaMoveHeight, MANIP_VEL);
        moveHoldWait(0, (0.6*HEIGHT-holdPipette->tip.y)*Pixel2Um, 0, MANIP_VEL);
        moveHoldWait(-holdPipette->xM*Pixel2Um-200, 0, 0, MANIP_VEL);
    }
    //起始位置设置为卵细胞区域
    oocyteArea = Point2f(microscope->getRelX(), microscope->getRelY());
    globalMap->pixel2Um = Pixel2Um;                         //记录当前镜头下的像素尺寸
    build();
    moveStageWait(0, 0.8*HEIGHT*Pixel2Um,  STAGE_VEL);
    //将微针移入视野
    if(holdPipette->isDetected)
    {
        // emit moveHold(0.42*WIDTH_UM+200, 0, 0, MANIP_VEL);
        // emit moveHold(0, 0, areaMoveHeight, MANIP_VEL);
        moveHoldWait(0.32*WIDTH_UM+200, 0, 0, MANIP_VEL);
        moveHoldWait(0, 0, areaMoveHeight, MANIP_VEL);
    }
    if(injectPipette->isDetected)
    {
        moveInjWait(-0.44*WIDTH_UM, 0, 0, MANIP_VEL);        //移动到高倍镜视野 TODO: 0.42为注射针刚好能从低倍镜下显示在高倍镜下的宽度
        moveInjWait(0, 0, areaMoveHeight, MANIP_VEL);
    }
    allOocytes->find(globalMap->mapImage, oocyteArea);  //寻找卵细胞
    // DEAL_FAULT("请手动切换到高倍镜，完成后关闭该窗口！！！");  //切换高倍镜
    if(mode==SCNT_SINGLE || !areAllOocytesEnucleated)
        nextStep = FETCHOOCYTE;
    else
        nextStep = FETCHOOCYTEFORSOMA;
    isWorking = false;
    emit finish();
    return nextStep;
}
void StepBuildMap::build()
{
    //保存全局地图，用于神经网络训练
    QDateTime dateTime(QDateTime::currentDateTime());
    QString time = dateTime.toString("yyyy-MM-dd-hh-mm-ss");
    QString path = FILE_PATH+QString("/%1/").arg(time);
    if(!QDir(path).exists())
        QDir().mkpath(path);

    GMAP_WIDTH  = GMAP_WIDTH_UM /Pixel2Um;
    GMAP_HEIGHT = GMAP_HEIGHT_UM/Pixel2Um;

    float ratio_ov = 0.2;
    float ovw = WIDTH*ratio_ov, ovh = HEIGHT*ratio_ov;//重叠区域占0.2
    float dw = WIDTH-ovw, dh = HEIGHT-ovh;
    int waitInterval_ms = 2.0*1000*max(dh, dw)/PIXEL_VEL + 500;          //2倍时间 ms
    int nw = ceil((GMAP_WIDTH_UM /Pixel2Um-ovw)/dw); //每一行多少张图片
    int nh = ceil((GMAP_HEIGHT_UM/Pixel2Um-ovh)/dh);
    float rw = GMAP_WIDTH_UM /Pixel2Um - ovw-(nw-1)*dw ;
    float rh = GMAP_HEIGHT_UM /Pixel2Um - ovh-(nh-1)*dh ;
    globalMap->mapImage = cv::Mat::zeros(Size(GMAP_WIDTH, GMAP_HEIGHT), CV_8UC1);//初始化全局地图
    Mat img = CurrentGray.clone();
    for(int i=0; i<img.rows; ++i)
        for(int j=0; j<img.cols; ++j)
            globalMap->mapImage.at<uchar>(i, j) = img.at<uchar>(i, j);
    int i=0, j=0;
    float ox = 0, oy = 0;
    cv::imwrite(path.toStdString()+to_string(i)+"_"+to_string(j)+".jpg", img);
    ON = true;
    while(!((i==nh-1)&&((j==nw-1&&nh%2==1)||(j==0&&nh%2==0))) && ON)//扫描结束的条件不满足
    {
        if(i%2==0)//i为偶数行，向右或向下(j==nw-1)运动
        {
            if(j==nw-1)//向下
            {
                if(i!=nh-2)//移动dh
                {
                    moveStageWait(0, dh*Pixel2Um,  STAGE_VEL);
                    oy+=dh;
                }
                else//移动rh
                {
                    moveStageWait(0, rh*Pixel2Um,  STAGE_VEL);
                    oy+=rh;
                }
                i+=1;
                img = CurrentGray.clone();
                extendMap(Point2f(ox, oy), img);
            }
            else//向右
            {
                if(j!=nw-2)//移动dw
                {
                    moveStageWait(dw*Pixel2Um, 0,  STAGE_VEL);
                    ox+=dw;
                }
                else//移动rw
                {
                    moveStageWait(rw*Pixel2Um, 0,  STAGE_VEL);
                    ox+=rw;
                }
                j+=1;
                img = CurrentGray.clone();
                extendMap(Point2f(ox, oy), img);
            }
        }
        else//i为奇数行，向左或向下(j==0)运动
        {
            if(j==0)//向下
            {
                if(i!=nh-2)//移动dh
                {
                    moveStageWait(0, dh*Pixel2Um,  STAGE_VEL);
                    oy+=dh;
                }
                else//移动rh
                {
                    moveStageWait(0, rh*Pixel2Um,  STAGE_VEL);
                    oy+=rh;
                }
                i+=1;
                img = CurrentGray.clone();
                extendMap(Point2f(ox, oy), img);
            }
            else//向左
            {
                if(j!=nw-1)//移动dw
                {
                    moveStageWait(-dw*Pixel2Um, 0,  STAGE_VEL);
                    ox-=dw;
                }
                else//移动rw
                {
                    moveStageWait(-rw*Pixel2Um, 0,  STAGE_VEL);
                    ox-=rw;
                }
                j-=1;
                img = CurrentGray.clone();
                extendMap(Point2f(ox, oy), img);
            }
        }
        cv::imwrite(path.toStdString()+to_string(i)+"_"+to_string(j)+".jpg", img);
    }
    cv::imwrite(path.toStdString()+"globalMap.jpg",globalMap->mapImage);
}
void StepBuildMap::extendMap(Point2f point, Mat img)
{
    int ox = point.x, oy = point.y;

    cv::Mat preMap; //当前图像、已构建的地图
    img = CurrentGray.clone();     preMap = globalMap->mapImage.clone();

    //图像拼接，需要加入裂缝处理
    cv::Mat tmpMap = cv::Mat::zeros(max(preMap.rows, oy+HEIGHT), max(preMap.cols, ox+WIDTH), CV_8UC1);      //更新后的局部大地图
    double dImg, dMap, alpha;             //离边界的距离和像素取值的权重
    cv::Mat bTmpMap, showMap;
    std::vector<std::vector<Point>> contoursMap;
    std::vector<Point> contourMap;
    preMap.copyTo(tmpMap(Range(0, preMap.rows), Range(0, preMap.cols)));

    threshold(tmpMap, bTmpMap, 1, 255, THRESH_BINARY);
    showMap = tmpMap.clone(); cvtColor(showMap, showMap, COLOR_GRAY2BGR);
    findContours(bTmpMap, contoursMap, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    contourMap = contoursMap[0];

    //        drawContours(showMap, contoursMap, -1, Scalar(0, 0, 255), 1);
    //        imshow("showMap", showMap);

    for(int i=0;i<img.rows;i++)
        for(int j=0;j<img.cols;j++)
            if(i+oy<0 ||j+ox<0)
                continue;
            else if(tmpMap.at<uchar>(i+oy,j+ox)==0)//原图中的空白地方，直接拷贝
                tmpMap.at<uchar>(i+oy,j+ox) = img.at<uchar>(i, j);
            else
            {
                Point2f p(j+ox, i+oy);
                dImg = min(i, min(j, min(HEIGHT-i, WIDTH-j)));
                dMap = abs(pointPolygonTest(contourMap, p, true));
                alpha = dMap/(dImg+dMap);
                tmpMap.at<uchar>(i+oy,j+ox) = (1-alpha) * img.at<uchar>(i, j) + alpha * tmpMap.at<uchar>(i+oy,j+ox);
            }

    globalMap->mapImage = tmpMap;
}
