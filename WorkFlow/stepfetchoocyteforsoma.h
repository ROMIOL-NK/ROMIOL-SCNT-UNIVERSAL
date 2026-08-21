#ifndef STEPFETCHOOCYTEFORSOMA_H
#define STEPFETCHOOCYTEFORSOMA_H
#include "workflow.h"

class StepFetchOocyteForSoma: public WorkFlow
{
    Q_OBJECT
public:
    StepFetchOocyteForSoma();
    bool ON;

    void findOocyteAround();                    //在附近寻找卵细胞
    void findNextOocyte();                      //在附近寻找卵细胞
    bool holdOocyte();                          //将卵细胞吸持住
    bool moveToNextOocyteInView();

public slots:
    WORKSTEP run();
};

#endif // STEPFETCHOOCYTEFORSOMA_H
