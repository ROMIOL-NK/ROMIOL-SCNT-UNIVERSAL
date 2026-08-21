#include "controller_pid.h"
using namespace std;

Controller_PID::Controller_PID(int n_in, int n_out):Controller(n_in, n_out)
{
    name = "PID";
    kp = 0.0;
    ki = 0.0;
    kd = 0.0;
    uM[0] = 0.01;
    um[0] = -0.01;
    saveParas();
    initialize();
}
void Controller_PID::saveParas()
{
    Controller::saveParas();
    qParas["kp"] = kp;
    qParas["ki"] = ki;
    qParas["kd"] = kd;
}
void Controller_PID::setParas(map<string, QVariant> q)
{
    Controller::setParas(q);
    // 自定义参数值
    if(qParas.count("kp")>0)
        kp = qParas["kp"].toFloat();
    if(qParas.count("ki")>0)
        ki = qParas["ki"].toFloat();
    if(qParas.count("kd")>0)
        kd = qParas["kd"].toFloat();
}

void Controller_PID::initialize()
{
    Controller::initialize();
    eI = 0;
    ePre = 0;
    eCur = 0;
}
void Controller_PID::get_yd(double t)
{
    Controller::get_yd(t);
}
void Controller_PID::set_y(vector<double> y_observe)
{
    assert(y_observe.size()>0);

    Controller::set_y(y_observe);
}
void Controller_PID::get_u(vector<double>& u)
{
    u.clear();
    eCur = yd[0] - y[0];
    eI += eCur*delta_t;
    u.push_back(kp*eCur+ki*eI+kd*(eCur-ePre)/delta_t);
    ePre = eCur;

    Controller::get_u(u);
}
