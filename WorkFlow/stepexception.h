#ifndef STEPEXCEPTION_H
#define STEPEXCEPTION_H
#include "workflow.h"

class StepException: public WorkFlow
{
    Q_OBJECT
public:
    StepException();
public slots:
    WORKSTEP run();
};

#endif // STEPEXCEPTION_H
