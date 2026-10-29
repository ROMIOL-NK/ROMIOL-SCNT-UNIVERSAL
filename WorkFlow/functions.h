#ifndef FUNCTIONS_H
#define FUNCTIONS_H
#include "public.h"
#include "Objects/objects.h"
#include "Components/components.h"
#include "Controllers/controllers.h"
#include "WorkFlow/workflow.h"

class Functions: public WorkFlow
{
    Q_OBJECT
public:
    Functions();
    ~Functions();

    std::atomic_bool ON;
    Controller* injGLI_Controller;
    Controller* holdP_Controller;
    Controller* injP_Controller;
public slots:
    void adjInjGLI();       //调节注射针液面位置
    void adjHoldP(float);   //调节吸持压
    void adjInjP(float);    //调节注射压
    void withdrawInj();     //注射针撤针
    void co_up();           //吸持针&注射针共同上升
    void co_down();         //吸持针&注射针共同下降
    void nextOocyte();      //到下一个卵的位置
    void findOocyte();      //附近区域搜索卵细胞
    void holdOocyte();      //吸持卵细胞（前提：吸持针、卵细胞在视野中，相对位置合适，可以直接调节气压进行吸持
    void releaseOocyte();   //吸持针释放卵细胞
    void focus(QString);    //调焦
    void test();
    void run(QString);
    WORKSTEP run(){return END;}
signals:
    void finish(QString);

    void sgnPlotLine(std::vector<double>, std::vector<double>, QString, QString);
    void sgnPlotLineMat(std::vector<double>, std::vector<cv::Mat>, std::vector<cv::Mat>, QString, QString);
    void sgnTest(std::vector<cv::Mat>);
};

#endif // FUNCTIONS_H
