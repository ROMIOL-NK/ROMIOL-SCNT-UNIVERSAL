#ifndef STEPPENETRATEFORSOMA_H
#define STEPPENETRATEFORSOMA_H
#include "workflow.h"

class StepPenetrateForSoma: public WorkFlow
{
    Q_OBJECT
public:
    StepPenetrateForSoma();

public slots:
    WORKSTEP run();

    void getxy(cv::Mat img, int& x, int& y);
    int get_center(cv::Mat img, int cy_x, int cy_y, int cy_r);
};

#endif // STEPPENETRATEFORSOMA_H
