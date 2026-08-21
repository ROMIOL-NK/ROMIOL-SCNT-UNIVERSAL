#ifndef CONTROLLER_ADRC_H
#define CONTROLLER_ADRC_H
#include "Controllers/controller.h"

class Controller_ADRC: public Controller
{
    Q_OBJECT
public:
    Controller_ADRC(int n_in=1, int n_out=1);

    virtual void saveParas();                                       //保存控制器参数
    virtual void setParas(std::map<std::string, QVariant>);         //设置参数
    virtual void initialize();                                      //初始化控制器参数、相关变量
    virtual void get_yd(double t=-1);                               //根据当前时刻，轨迹参数paras，获取系统参考值   r，维数 m;
    virtual void set_y(std::vector<double> y);                      //获取系统输出值   y，维数 m
    virtual void get_u(std::vector<double>&u);                      //获取控制器输出   u，维数 p

    double r, b0, c, h1, delta;
    double beta1, beta2, beta3;

    double v1, v2, z1, z2, z3;

    // 辅助函数
    double sign(double x){if(x>0) return 1; else if(x<0) return -1; return 0;}
    double fhan(double v1, double v2, double h0);
    double fal(double e, double alpha, double delta);
};

#endif // CONTROLLER_ADRC_H
