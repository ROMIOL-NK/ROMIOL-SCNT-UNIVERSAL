#ifndef STEPINJECTSOMA_H
#define STEPINJECTSOMA_H
#include "workflow.h"
#include "steppenetrate.h"

class StepInjectSoma: public WorkFlow
{
    Q_OBJECT
public:
    StepInjectSoma();
    bool ON;
public slots:
    WORKSTEP run();
};

#endif // STEPINJECTSOMA_H
