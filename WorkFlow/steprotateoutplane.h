#ifndef STEPROTATEOUTPLANE_H
#define STEPROTATEOUTPLANE_H
#include "workflow.h"

class StepRotateOutPlane: public WorkFlow
{
    Q_OBJECT
public:
    StepRotateOutPlane();
    bool isPBinPlace();
    void injet_move_to_rel(float x,float y,float z);
    void inject_rot_arc_xz(int iniAngle,int smallRotatedAngle,int oneSmallAngle);
    std::vector<float> curInjectTipRel;//相对于细胞中心的注射针针尖位置（单位微米）

    bool ON;
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
    void sgnStartPServo();
    void sgnStopPServo();
    void sgnInitTrackerOocyte(cv::Mat, cv::Rect);
    void sgnStopTrackOocyte();
    void sgnStartCompare(cv::Mat, cv::Rect);
    void sgnCompareAndCalHoldP(cv::Mat);
};

#endif // STEPROTATEOUTPLANE_H
