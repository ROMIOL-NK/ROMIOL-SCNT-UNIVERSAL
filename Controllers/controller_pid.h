#ifndef CONTROLLER_PID_H
#define CONTROLLER_PID_H
#include "Controllers/controller.h"
// PID控制器
class Controller_PID: public Controller
{
    Q_OBJECT
public:
    Controller_PID(int n_in=1, int n_out=1);                //仅支持单输入单输出 SISO 系统的控制
    virtual void saveParas();                                       //保存控制器参数
    virtual void setParas(std::map<std::string, QVariant>);         //设置参数
    virtual void initialize();                                      //初始化控制器参数、相关变量
    virtual void get_yd(double t=-1);                               //根据当前时刻，轨迹参数paras，获取系统参考值   r，维数 m;
    virtual void set_y(std::vector<double> y);                      //获取系统输出值   y，维数 m
    virtual void get_u(std::vector<double>&u);                      //获取控制器输出   u，维数 p
    double kp, ki, kd;                                              //比例、积分、微分系数
    double eI, ePre, eCur;                                          //误差积分、上一时刻、当前时刻的误差
};
#endif // CONTROLLER_PID_H
