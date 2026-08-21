#ifndef STEPENUCLEATION_H
#define STEPENUCLEATION_H
#include "workflow.h"
#include "Controllers/controllers.h"
#undef slots
#include <Python.h>
#define slots Q_SLOTS

class StepEnucleation: public WorkFlow
{
    Q_OBJECT
public:
    StepEnucleation();
    ~StepEnucleation();

    Controller* enuController;
    Controller* injP_Controller;
    bool ON;
private:
    std::string path;                                                   //文件保存路径

public slots:
    WORKSTEP run();
};

#endif // STEPENUCLEATION_H
