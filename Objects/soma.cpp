#include "soma.h"

using namespace cv;
using namespace std;
Soma::Soma()
{
    isDetected = false;
    tracker = TrackerMIL::create();
}
Soma::~Soma()
{

}
void Soma::set(vector<Point> curContour)
{
    if(curContour.empty())
        return;
    contour.assign(curContour.begin(), curContour.end());
    area = contourArea(contour);
    xm = min_element(contour.begin(), contour.end(), [](Point p1, Point p2){return p1.x<p2.x;})->x;
    xM = max_element(contour.begin(), contour.end(), [](Point p1, Point p2){return p1.x<p2.x;})->x;
    ym = min_element(contour.begin(), contour.end(), [](Point p1, Point p2){return p1.y<p2.y;})->y;
    yM = max_element(contour.begin(), contour.end(), [](Point p1, Point p2){return p1.y<p2.y;})->y;
    Moments m = moments(contour);
    if(m.m00!=0)
        center = Point2f(m.m10/m.m00, m.m01/m.m00);
    else
        center = Point2f((xm+xM)/2, (ym+yM)/2);

    isDetected = true;
}
void Soma::find(Mat src)
{
    reset();
    tracker = TrackerMIL::create();

    roi=selectROI("track", src);
    destroyWindow("track");

    if(roi.width>0 && roi.height>0)
    {
        isDetected = true;
        xm = roi.x;
        ym = roi.y;
        xM = roi.x+roi.width;
        yM = roi.y+roi.height;
        area = roi.width*roi.height;
        center = Point2f(0.5*(xm+xM), 0.5*(ym+yM));
        contour.push_back(Point(roi.x, roi.y));
        contour.push_back(Point(roi.x+roi.width, roi.y));
        contour.push_back(Point(roi.x+roi.width, roi.y+roi.height));
        contour.push_back(Point(roi.x, roi.y+roi.height));
        tracker->init(src, roi);
    }
}
void Soma::findinPipette(Mat src)
{
    isDetected = true;
    if(isDetected)
    {
        tracker = TrackerMIL::create();
        tracker->init(src, roi);
    }
}
void Soma::track(Mat src)
{
    tracker->update(src,roi);
    xm = roi.x;
    ym = roi.y;
    xM = roi.x+roi.width;
    yM = roi.y+roi.height;
}
void Soma::reset()
{
    isDetected = false;
    contour.clear();
    trackingPoints.clear();
    tracker.reset();
}
