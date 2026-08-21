#include "controller_pneumatic.h"
using namespace cv;
using namespace std;


Controller_PNEUMATIC::Controller_PNEUMATIC(int n_in, int n_out):Controller(n_in, n_out)
{
    name = "PNEUMATIC";
    //默认参数值
    U_name = "spd";
    uM[0] = 0.3;
    um[0] = -0.3;
    delta = 10000.0;
    p0 = 101300.0;
    V0 = 2000.0;
    kx = 600.0;
    k = 2;
    b = 0.008;

    xm0 = -1.0978;
    xp = 3000.0;
    saveParas();
    initialize();
}
void Controller_PNEUMATIC::saveParas()
{
    //保存参数到 qParas 中
    Controller::saveParas();
    qParas["delta"] = delta;
    qParas["xm0"] = xm0;
    qParas["xp"] = xp;
    qParas["p0"] = p0;
    qParas["V0"] = V0;
    qParas["kx"] = kx;
    qParas["k"] = k;
    qParas["b"] = b;
}
void Controller_PNEUMATIC::setParas(map<string, QVariant> q)
{
    Controller::setParas(q);
    // 自定义参数值
    if(qParas.count("delta")>0)
        delta = qParas["delta"].toFloat();
    if(qParas.count("xm0")>0)
        xm0 = qParas["xm0"].toFloat();
    if(qParas.count("delta")>0)
        delta = qParas["delta"].toFloat();
    if(qParas.count("p0")>0)
        p0 = qParas["p0"].toFloat();
    if(qParas.count("V0")>0)
        V0 = qParas["V0"].toFloat();
    if(qParas.count("kx")>0)
        kx = qParas["kx"].toFloat();
    if(qParas.count("k")>0)
        k = qParas["k"].toFloat();
    if(qParas.count("b")>0)
        b = qParas["b"].toFloat();
}
void Controller_PNEUMATIC::initialize()
{
    Controller::initialize();
    a = 0;
    isInitialized = false;//用做初次控制量起作用的判断
}
void Controller_PNEUMATIC::get_yd(double t)
{
    Controller::get_yd(t);
}
void Controller_PNEUMATIC::set_y(vector<double> y_observe)
{
    assert(y_observe.size()>0);
    Controller::set_y(y_observe);
}

void Controller_PNEUMATIC::get_u(std::vector<double>&u)
{
    assert(z.size()>=2);// z[0]: x_motor, z[1]: dp
    u.clear();
    double e, u_var;
    e = yd[0]-y[0];
    if(abs(e)>delta)
        u_var = (e>0?uM[0]:um[0]);
    else
        u_var = k*e*(V0+xp+kx*(z[1]-xm0))/(kx*(p0+z[0]));

    if(isInitialized && abs(e)>100 && abs(a)<b && (u_var>0&&!wasPlusU||u_var<0&&wasPlusU))
    {
        double v;
        if(u_var>0)
            v = b;
        else if(u_var<0)
            v = -b;
        U_name = "pos";
        u.push_back(v);
    }
    else
    {
        U_name = "spd";
        u.push_back(u_var);
    }
    if(u[0]>0)
        wasPlusU = true;
    else if(u[0]<0)
        wasPlusU = false;
    Controller::get_u(u);
    if(!isInitialized)
        isInitialized = true;
}
