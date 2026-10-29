#ifndef STEPROTATEOUTPLANE_H
#define STEPROTATEOUTPLANE_H
#include "workflow.h"

class StepRotateOutPlane: public WorkFlow
{
    Q_OBJECT
public:
    enum RotateOutPlaneMode {
        ROTATE_OUT_ORIGINAL_3D,
        ROTATE_OUT_X_LINE_DOWN,
        ROTATE_OUT_X_LINE_UP,
        ROTATE_OUT_XYZ
    };

    StepRotateOutPlane();
    bool isPBinPlace();
    void injet_move_to_rel(float x,float y,float z,float v = 0);
    void inject_rot_arc_xz(int iniAngle,int smallRotatedAngle,int oneSmallAngle);
    void rotate_original_3d(); /* 原始3D弧线拨法 */
    void rotate_x_line_down(); /* 针壁从down侧沿X向直线蹭 */
    void rotate_x_line_up();   /* 针壁从up侧沿X向直线蹭 */
    void rotate_xyz();         /* 针尖斜向蹭 */
    void rotate_x_line(bool isDown);
    float safe_injection_side_x(float preferredX); /* 避开吸持针侧的最小安全X */
    std::vector<float> curInjectTipRel;//相对于细胞中心的注射针针尖位置（单位微米）

    bool ON;
    RotateOutPlaneMode rotateOutPlaneMode; // 平面外拨动方式，默认XYZ斜蹭
    float outPlaneVel; // 新拨法的注射针移动速度
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
