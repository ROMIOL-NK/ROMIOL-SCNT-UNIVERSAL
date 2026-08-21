#ifndef CONTROLLER_CGPC_H
#define CONTROLLER_CGPC_H
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include "controller.h"
#include "utils/custom_algorithms.h"
// Constrained Generalized Predictive Control (有约束的 GPC |多变量自校正)
// 一种模式下最开始的CARIMA模型参数是固定的（isSaveLastPara=false），另一种模式下上一次实验结束后的辨识结果会作为下一次的初始参数（isSaveLastPara=true）
class Controller_CGPC: public Controller
{
    Q_OBJECT
public:
    Controller_CGPC(int n_in=1, int n_out=1);

    virtual void saveParas();                                   //保存控制器参数
    virtual void setParas(std::map<std::string, QVariant>);     //设置参数
    virtual void initialize();                                  //初始化控制器参数、相关变量
    virtual void get_yd(double t=-1);                           //根据当前时刻，轨迹参数paras，获取系统参考值   r，维数 m;
    virtual void set_y(std::vector<double> y);                  //获取系统输出值   y，维数 m
    virtual void get_u(std::vector<double>&u);                  //获取控制器输出   u，维数 p

    int n_steps;                                                //仿真步数

    int na, nb, N, Nu;                                          //控制器&仿真系统 输出阶数：na  输入阶数：nb  预测步数：N  控制步数：Nu
    double lambda;                                              //输入加权因子
    double softenCoe;                                           //柔化因子
    std::vector<double> duM;                        			//输入微分最大阈值
    std::vector<double> dum;                        			//输入微分最小阈值
    std::vector<double> thetaDefault;                     		//theta的预设值
    std::vector<double> theta0exp;                    			//本次实验中theta的初始值
    cv::Mat A, B, Aa;                                           //辨识参数、Aa = ΔA
    cv::Mat E, G, ej, Ei, EiB, HiDelta;                         //丢番图方程变量
    std::vector<cv::Mat> F, H, Yk_, Uk_;                        //丢番图方程变量、历史输出、历史输入
    cv::Mat yr, yk;                                             //期望输出、当前输出
    cv::Mat Gm, Omega, Yd, Y0;                                  //二次型优化问题中的矩阵
    cv::Mat duN, inv, uk;                                       //求解最优化问题的中间变量
    cv::Mat phi, theta_iy, theta;                               //自校正 数据、参数
    cv::Mat duMin, duMax, Ast;                                  //计算约束 duMin <= AstΔU <= duMax;
    std::vector<cv::Mat> P;                                     //协方差矩阵

};

#endif // CONTROLLER_CGPC_H
