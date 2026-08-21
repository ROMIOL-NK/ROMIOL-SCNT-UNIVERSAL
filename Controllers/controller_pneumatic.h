#ifndef CONTROLLER_PNEUMATIC_H
#define CONTROLLER_PNEUMATIC_H
#include "controller.h"
// 反步自抗扰控制 （BackStepping Active Disturbance Rejection Control)
class Controller_PNEUMATIC: public Controller
{
    Q_OBJECT
public:
    Controller_PNEUMATIC(int n_in=1, int n_out=1);
    virtual void saveParas();                                   //保存控制器参数
    virtual void setParas(std::map<std::string, QVariant>);     //设置参数
    virtual void initialize();                                  //初始化控制器参数、相关变量
    virtual void get_yd(double t=-1);                           //根据当前时刻，轨迹参数paras，获取系统参考值   r，维数 m;
    virtual void set_y(std::vector<double> y);                  //获取系统输出值   y，维数 m
    virtual void get_u(std::vector<double>&u);                  //获取控制器输出   u，维数 p
    double delta, xm0, xp, p0, V0, kx, k;                       // delta: 误差边界； xm0, xp：表压为0时的电机位置和注射器中气体体积（uL），p0：大气压；V0: 气管中的体积（uL）；kx气泵系数 (uL/r); k: 阻尼
    double a, b;                                                //死区记忆变量 (a) 死区阈值 (b)
    bool wasPlusU;
};
#endif // CONTROLLER_PNEUMATIC_H
