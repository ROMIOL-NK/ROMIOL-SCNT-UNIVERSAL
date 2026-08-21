#include "controller_adrc.h"
using namespace std;

Controller_ADRC::Controller_ADRC(int n_in, int n_out):Controller(n_in, n_out)
{
    name = "ADRC";
    //设置默认参数
    r = 10.0;
    c = 2.0;
    b0 = 0.001;
    h1 = 0.5;
    delta = 30;

    U_name = "p";
    uM[0] = 5000;
    um[0] = -1000;
    saveParas();
    initialize();
}
void Controller_ADRC::saveParas()
{
    //保存参数到 qParas 中
    Controller::saveParas();
    qParas["r"] = r;
    qParas["c"] = c;
    qParas["b0"] = b0;
    qParas["h1"] = h1;
    qParas["delta"] = delta;
}
void Controller_ADRC::setParas(map<string, QVariant> q)
{
    Controller::setParas(q);
    // 自定义参数值
    if(qParas.count("r")>0)
        r = qParas["r"].toFloat();
    if(qParas.count("c")>0)
        c = qParas["c"].toFloat();
    if(qParas.count("b0")>0)
        b0 = qParas["b0"].toFloat();
    if(qParas.count("h1")>0)
        h1 = qParas["h1"].toFloat();
    if(qParas.count("delta")>0)
        delta = qParas["delta"].toFloat();
}

void Controller_ADRC::initialize()
{
    Controller::initialize();
    // 计算次级变量
    beta1 = 1;  beta2 = 0.5/pow(delta_t, 0.5);  beta3 = 0.08/pow(delta_t, 1.2);
    v1 = 0; v2 = 0;
    z1 = 0; z2 = 0; z3 = 0;
}
void Controller_ADRC::get_yd(double t)
{
    Controller::get_yd(t);
}
void Controller_ADRC::set_y(vector<double> y_observe)
{
    assert(y_observe.size()>0);
    Controller::set_y(y_observe);
}
void Controller_ADRC::get_u(vector<double>& u)
{
    u.clear();
    double u_var = 0;

    double v, fv, h, e, fe, fe1, e1, e2;
    v =  yd[0];
    h = delta_t;
    fv = fhan(v1-v, v2, h);
    v1 += h*v2;
    v2 += h*fv;
    e = z1-y[0];
    fe = fal(e,0.5,delta);  fe1 = fal(e, 0.25, delta);

    e1=v1-z1;   e2 = v2-z2;
    u_var = -(fhan(e1, c*e2, h1)+z3)/b0;

    z1 += h*z2-beta1*e;
    z2 += h*(z3+b0*u_var)-beta2*fe;
    z3 -= beta3*fe1;

    qDebug()<<z1<<z2<<z3;
    u.push_back(u_var);
    Controller::get_u(u);
}
double Controller_ADRC::fhan(double v1, double v2, double h0)
{
    double d, a0, y, a1, a2, sy, a, sa;
    d = h0*r*r;    a0 = h0*v2;    y = v1+a0;
    a1 = sqrt(d*(d+8*abs(y)));
    a2 = a0+sign(y)*(a1-d)/2;
    sy = (sign(y+d)-sign(y-d))/2;
    a = (a0+y-a2)*sy+a2;
    sa = (sign(a+d)-sign(a-d))/2;
    return -r*(a/d-sign(a))*sa-r*sign(a);
}
double Controller_ADRC::fal(double e, double alpha, double delta)
{
    if(abs(e)<=delta)
        return e/pow(delta, 1-alpha);
    else
        return pow(abs(e), alpha)*sign(e);
}
