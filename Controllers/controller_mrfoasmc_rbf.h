#ifndef CONTROLLER_MRFOASMC_RBF_H
#define CONTROLLER_MRFOASMC_RBF_H
#include "Controllers/controller.h"
// 分数阶自适应滑模控制器 (Fractional Order Adaptive Sliding Mode Controller)
class Controller_MRFOASMC_RBF: public Controller
{
    Q_OBJECT
public:
    Controller_MRFOASMC_RBF(int n_in=1, int n_out=1);               //仅支持单输入单输出 SISO 系统的控制

    virtual void saveParas();                                   //保存控制器参数
    virtual void setParas(std::map<std::string, QVariant>);         //设置参数
    virtual void initialize();                                      //初始化控制器参数、相关变量
    virtual void get_yd(double t=-1);                               //根据当前时刻，轨迹参数paras，获取系统参考值   r，维数 m;
    virtual void set_y(std::vector<double> y);                      //获取系统输出值   y，维数 m
    virtual void get_u(std::vector<double>&u);                      //获取控制器输出   u，维数 p

    int order;                               //阶数
    double sumAlpha;                         //最高阶次
    double bu;                               //控制器输入 u 系数的估计值
    std::vector<double> alpha;               //阶次
    std::vector<double> betaa;               //阶次 βi = Σ_{0}{i-1}αi
    std::vector<double> K;                   //参考模型系数列表
    cv::Mat A, B, Am, Bm;
    cv::Mat K1, K1_star;

    cv::Mat xMat, xmMat, sMat, eMat, deMat;  // deMat: D^α e
    std::vector<double> x;                   //状态数组
    std::vector<double> e;                   //误差数组
    std::vector<double> xm;                  //状态数组

    // RBF神经网络参数
    std::vector<double> omega0;
    std::vector<double> omega;
    std::vector<std::vector<double>> cRBF;
    std::vector<double> sigmaRBF;
    double eta;
    //
    double TF;                               //预计控制终止时刻
    // 控制器参数
    std::vector<std::vector<double>> W;      //权重矩阵 β (W) = Σ α
    std::vector<std::vector<double>> Wa;     //权重矩阵 α (W), α-1 (WI)
    std::vector<double> theta_hat;           //参数估计值
    std::vector<std::vector<double>> E;      // 误差 e（E[0]) 及 e_i^<α_i> (E[i+1]) 的历史数据
    std::vector<std::vector<double>> X;      // 状态     x（X[0]) 及 x_i^<α_i> (X[i+1]) 的历史数据
public:
    std::vector<std::vector<double>> Xm;     // 参考模型 xm（E[0]) 及 e_i^<α_i> (E[i+1]) 的历史数据
    // 辅助函数
    std::vector<double> glfdiff(std::vector<double> y, double gam, double h=-1);
    std::vector<cv::Mat> glfdiffMat(std::vector<cv::Mat> y, std::vector<double> gam, double h=-1);
    double ite_glfdiff(std::vector<double> y, int i);
    double est_boundary(cv::Mat s);
    void sim_y(std::vector<double> u);       //利用 D^α y = A y + B(u+f) 仿真得到 y
};

#endif // CONTROLLER_MRFOASMC_RBF_H
