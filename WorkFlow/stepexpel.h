#ifndef STEPEXPEL_H
#define STEPEXPEL_H
#include "workflow.h"
#include "Controllers/controllers.h"
#undef slots
#include <Python.h>
#define slots Q_SLOTS

class StepExpel: public WorkFlow
{
    Q_OBJECT
public:
    StepExpel();
    ~StepExpel();

    bool ON;

public slots:
    WORKSTEP run();
};

#endif // STEPEXPEL_H
