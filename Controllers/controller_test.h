#ifndef CONTROLLER_TEST_H
#define CONTROLLER_TEST_H
#include "Controllers/controller.h"

class Controller_TEST: public Controller
{
    Q_OBJECT
public:
    Controller_TEST(int n_in=1, int n_out=1);

    virtual void saveParas();                                       //保存控制器参数
    virtual void setParas(std::map<std::string, QVariant>);         //设置参数
    virtual void initialize();                                      //初始化控制器参数、相关变量
    virtual void get_yd(double t=-1);                               //根据当前时刻，轨迹参数paras，获取系统参考值   r，维数 m;
    virtual void set_y(std::vector<double> y);                      //获取系统输出值   y，维数 m
    virtual void get_u(std::vector<double>&u);                      //获取控制器输出   u，维数 p

    double TF;                                                      //预计控制终止时刻
    double alpha;
    double M;                                                       //扰动误差上界
    double p_plus,p_minus;
    double beta_plus, beta_minus;

    double gamma, K, s;
    double GammaAlpha_1;                                            //Γ(α-1)

    double lambda0, lambda1, eta;                                   //λ0, λ1, η
    double k0, k1;

    std::vector<double> e, De, xd, int_e_1_Alpha, intTerm;         //intTerm:积分项
    std::map<double, std::vector<double>> W;                        //分数阶微积分权重

    double lambda, mu;              //滑模面参数
    double k;

    // 辅助函数
    double ite_glfdiff(std::vector<double> y, double a);
};

#endif // CONTROLLER_TEST_H
