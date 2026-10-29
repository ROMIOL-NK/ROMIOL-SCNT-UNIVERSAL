#ifndef STEPROTATEINPLANE_H
#define STEPROTATEINPLANE_H
#include "workflow.h"
#include "track.h"

class StepRotateInPlane: public WorkFlow
{
    Q_OBJECT
public:
    StepRotateInPlane();
    void injet_move_to_rel(float x,float y,float z,int v=500);
    void inject_rot_arc_xy(int iniAngle,int smallRotatedAngle,int oneSmallAngle);

    bool isPBinPlace();
    std::vector<float> curInjectTipRel;//相对于细胞中心的注射针针尖位置（单位微米）

    bool ON;
    bool isPBdetected;
    int oocytesRadius; //细胞半径，单位微米
    int penDepth; //刺入细胞的深度
    float iniDisRatio;
    int iniAngle;
    int oneBigAngle;
    int oneSmallAngle;
    int rotatedAngle;
    int maxRotateAngle;
    float ANTORAD;
public slots:
    WORKSTEP run();
signals:
    void sgnInitTrackerPB(cv::Mat, cv::Rect);
    void sgnInitTrackerOocyte(cv::Mat, cv::Rect);
    void sgnStopTrackPB();
    void sgnStopTrackOocyte();

    void sgnStartCompare(cv::Mat, cv::Rect);
    void sgnCompareAndCalHoldP(cv::Mat);
};
#endif // STEPROTATEINPLANE_H
