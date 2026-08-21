#ifndef CONTROLLER_FUZZY_FOASMC_RBF_PI_H
#define CONTROLLER_FUZZY_FOASMC_RBF_PI_H
#include "Controllers/controller.h"
// 模糊分数阶自适应滑模控制器 RBF&PI (Fuzzy Fractional Order Adaptive Sliding Mode Controller with RBF and PI)
class Controller_FUZZY_FOASMC_RBF_PI: public Controller
{
    Q_OBJECT
public:
    Controller_FUZZY_FOASMC_RBF_PI(int n_in=1, int n_out=1);        //仅支持单输入单输出 SISO 系统的控制

    virtual void saveParas();                                   //保存控制器参数
    virtual void setParas(std::map<std::string, QVariant>);         //设置参数
    virtual void initialize();                                      //初始化控制器参数、相关变量
    virtual void get_yd(double t=-1);                               //根据当前时刻，轨迹参数paras，获取系统参考值   r，维数 m;
    virtual void set_y(std::vector<double> y);                      //获取系统输出值   y，维数 m
    virtual void get_u(std::vector<double>&u);                      //获取控制器输出   u，维数 p

    double l1, l2, l3, ksi, psi;                // λ1,2,3; ξ, ψ
    std::vector<double> alpha;                  //阶次

    double beta1, beta2, mu;
    cv::Mat Ah, Bh, Kh;
    cv::Mat K1, K2;
    int order;                                  //阶数
    double TF;                                  //预计控制终止时刻
    double sumAlpha;                            //最高阶次
    double s, si;                               //s和 s的积分

    cv::Mat xMat, dxMat, sMat;
    std::vector<double> x;                      //状态数组

    // RBF神经网络参数
    std::vector<double> omega0;
    std::vector<double> omega;
    std::vector<std::vector<double>> cRBF;
    std::vector<double> sigmaRBF;

    std::vector<std::vector<double>> Wa;     //权重矩阵 α (W), α-1 (WI)
    std::vector<std::vector<double>> X;      // 状态     x（X[0]) 及 x_i^<α_i> (X[i+1]) 的历史数据
public:
    // 辅助函数
    std::vector<double> glfdiff(std::vector<double> y, double gam, double h=-1);
    std::vector<cv::Mat> glfdiffMat(std::vector<cv::Mat> y, std::vector<double> gam, double h=-1);
    double ite_glfdiff(std::vector<double> y, int i);
    double est_boundary(cv::Mat s);
};

#endif // CONTROLLER_FUZZY_FOASMC_RBF_PI_H
