#ifndef STEPRELEASEOOCYTE_H
#define STEPRELEASEOOCYTE_H
#include "workflow.h"

class StepReleaseOocyte: public WorkFlow
{
    Q_OBJECT
public:
    StepReleaseOocyte();
    bool output_Oocyte();

public slots:
    WORKSTEP run();
};

#endif // STEPRELEASEOOCYTE_H
