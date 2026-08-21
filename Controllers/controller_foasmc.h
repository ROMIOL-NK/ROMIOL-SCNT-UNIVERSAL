#ifndef CONTROLLER_FOASMC_H
#define CONTROLLER_FOASMC_H
#include "Controllers/controller.h"
// 分数阶自适应滑模控制器 (Fractional Order Adaptive Sliding Mode Controller)
class Controller_FOASMC: public Controller
{
    Q_OBJECT
public:
    Controller_FOASMC(int n_in=1, int n_out=1);                     //仅支持单输入单输出 SISO 系统的控制

    virtual void saveParas();                                       //保存控制器参数
    virtual void setParas(std::map<std::string, QVariant>);         //设置参数
    virtual void initialize();                                      //初始化控制器参数、相关变量
    virtual void get_yd(double t=-1);                               //根据当前时刻，轨迹参数paras，获取系统参考值   r，维数 m;
    virtual void set_y(std::vector<double> y);                      //获取系统输出值   y，维数 m
    virtual void get_u(std::vector<double>&u);                      //获取控制器输出   u，维数 p

    double TF;                               //预计控制终止时刻
    // 控制器参数
    int order;                               //阶数
    std::vector<double> alpha;               //阶次
    std::vector<std::vector<double>> W;      //权重矩阵
    double sumAlpha;                         //最高阶次
    std::vector<double> K;                   //系数数组
    std::vector<double> t;                   //时间数组
    std::vector<double> r;                   //参考轨迹
    std::vector<double> x;                   //状态数组
    std::vector<double> e;                   //误差数组
    std::vector<double> GAMMA;               //参数更新增益
    std::vector<double> theta_hat;           //参数估计值
    std::vector<double> theta_hat0;          //参数初始估计值
    std::vector<double> drAlpha;             // r的sumAlpha阶导数
    std::vector<std::vector<double>> E;      // 误差 e（E[0]) 及 e_i^<α_i> (E[i+1]) 的历史数据
    std::vector<std::vector<double>> X;      // 状态 x（X[0]) 及 x_i^<α_i> (X[i+1]) 的历史数据
    double bu;                               //控制器输入 u 系数的估计值
    double k;                                //反馈增益
    double M;                                //最大扰动的绝对值
    double km;                               //M_dot = km|s|
    bool isM_Adapt;                          //M是否自适应调整

    // 辅助函数
    std::vector<double> glfdiff(std::vector<double> y, double gam);
    double ite_glfdiff(std::vector<double> y, int i);
};

#endif // CONTROLLER_FOASMC_H
