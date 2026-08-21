#include "track.h"
using namespace cv;
using namespace std;

Track::Track()
{
    trackerPB = TrackerMIL::create();
    trackerOocyte = TrackerMIL::create();
    interval = 200;
    timerPB = new QTimer(this);
    timerOocyte = new QTimer(this);
    connect(timerPB, SIGNAL(timeout()), this, SLOT(runPB()));
    connect(timerOocyte, SIGNAL(timeout()), this, SLOT(runOocyte()));
}
Track::~Track() {}
void Track::initPB(Mat img, Rect init_roiPB)
{
    trackerPB->init(img, init_roiPB);
    roiPB = init_roiPB;
    timerPB->start(interval);
}
void Track::initOocyte(Mat img, Rect init_roiOocyte)
{
    trackerOocyte->init(img, init_roiOocyte);
    roiOocyte = init_roiOocyte;
    timerOocyte->start(interval);
}
void Track::runPB()
{
    trackerPB->update(CurrentBGR, roiPB);
    currentOocyte->rectPB = roiPB;
    currentOocyte->centerPB.x = currentOocyte->rectPB.x + 0.5*currentOocyte->rectPB.width;
    currentOocyte->centerPB.y = currentOocyte->rectPB.y + 0.5*currentOocyte->rectPB.height;
}
void Track::runOocyte()
{
    currentOocyte->find(CurrentBGR);
    // trackerOocyte->update(CurrentBGR, roiOocyte);
    // currentOocyte->rectZP = roiOocyte;
    // vector<Point> contourZP;
    // contourZP.push_back(Point(roiOocyte.x, roiOocyte.y));
    // contourZP.push_back(Point(roiOocyte.x+roiOocyte.width, roiOocyte.y));
    // contourZP.push_back(Point(roiOocyte.x+roiOocyte.width, roiOocyte.y+roiOocyte.height));
    // contourZP.push_back(Point(roiOocyte.x, roiOocyte.y+roiOocyte.height));
    // currentOocyte->setZP(contourZP);
}
void Track::stopPB()
{
    timerPB->stop();
}
void Track::stopOocyte()
{
    timerOocyte->stop();
}
