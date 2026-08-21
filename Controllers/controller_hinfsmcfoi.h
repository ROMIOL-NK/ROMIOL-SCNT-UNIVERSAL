#ifndef CONTROLLER_HINFSMCFOI_H
#define CONTROLLER_HINFSMCFOI_H
#include "Controllers/controller.h"

// H∞ 滑模控制器 分数阶非相称系统 (H∞ sliding mode control for fractional order incommensurate systems)
class Controller_HinfSMCFOI: public Controller
{
    Q_OBJECT
public:
    Controller_HinfSMCFOI(int n_in=1, int n_out=1);                 //仅支持单输入单输出 SISO 系统的控制

    virtual void saveParas();                                   //保存控制器参数
    virtual void setParas(std::map<std::string, QVariant>);         //设置参数
    virtual void initialize();                                      //初始化控制器参数、相关变量
    virtual void get_yd(double t=-1);                               //根据当前时刻，轨迹参数paras，获取系统参考值   r，维数 m;
    virtual void set_y(std::vector<double> y);                      //获取系统输出值   y，维数 m
    virtual void get_u(std::vector<double>&u);                      //获取控制器输出   u，维数 p

    cv::Mat A, B, Bw, S, xMat;
    double phi, w, mu, zeta;
    double SB, SBw, SBinv;

    double TF;                               //预计控制终止时刻
    // 控制器参数
    int order;                               //阶数
    std::vector<double> alpha;               //阶次
    std::vector<std::vector<double>> W;      //权重矩阵
    std::vector<double> t;                   //时间数组
    std::vector<double> x;                   //状态数组
    std::vector<double> drAlpha;             // r的sumAlpha阶导数
    std::vector<std::vector<double>> X;      // 状态 x（X[0]) 及 x_i^<α_i> (X[i+1]) 的历史数据

    // 辅助函数
    std::vector<double> glfdiff(std::vector<double> y, double gam);
    double ite_glfdiff(std::vector<double> y, int i);
};

#endif // CONTROLLER_HINFSMCFOI_H
