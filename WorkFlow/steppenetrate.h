#ifndef STEPPENETRATE_H
#define STEPPENETRATE_H
#include "workflow.h"

class StepPenetrate: public WorkFlow
{
    Q_OBJECT
public:
    StepPenetrate();

public slots:
    WORKSTEP run();

    void getxy(cv::Mat img, int& x, int& y);
    int get_center(cv::Mat img, int cy_x, int cy_y, int cy_r);
};

#endif // STEPPENETRATE_H
