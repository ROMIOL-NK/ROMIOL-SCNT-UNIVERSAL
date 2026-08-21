#ifndef STEPFETCHSOMA_H
#define STEPFETCHSOMA_H
#include "workflow.h"
#include "Controllers/controllers.h"

class StepFetchSoma: public WorkFlow
{
    Q_OBJECT
public:
    StepFetchSoma();
    ~StepFetchSoma();
    Controller* somaController;
    bool ON;
    bool isReady;       //已经到达供体区域，调整后焦距和注射针高度
public slots:
    WORKSTEP run();
};

#endif // STEPFETCHSOMA_H
