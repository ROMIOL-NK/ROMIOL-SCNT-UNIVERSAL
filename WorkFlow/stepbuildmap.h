//在体细胞区域构建地图，提取所有卵细胞的位置信息，排序，为后续操作做准备
#ifndef STEPBUILDMAP_H
#define STEPBUILDMAP_H
#include "workflow.h"
#include "Objects/globalmap.h"
class StepBuildMap: public WorkFlow
{
    Q_OBJECT
public:
    StepBuildMap();

    bool ON;
    cv::Mat mapImg;
public slots:
    WORKSTEP run();
    void build();
    void extendMap(cv::Point2f, cv::Mat);
};

#endif // STEPBUILDMAP_H
