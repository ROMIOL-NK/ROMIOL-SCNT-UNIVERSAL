#ifndef STEPEXPEL_H
#define STEPEXPEL_H
#include "workflow.h"
#include "Controllers/controllers.h"

class StepExpel: public WorkFlow
{
    Q_OBJECT
public:
    StepExpel();
    ~StepExpel();

public slots:
    WORKSTEP run();
};

#endif // STEPEXPEL_H
