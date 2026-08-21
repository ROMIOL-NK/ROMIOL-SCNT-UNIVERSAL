#ifndef CONTROLLER_ZDT_H
#define CONTROLLER_ZDT_H
#include <vector>
#include <cmath>
#include <algorithm>
#include "Controllers/controller.h"
// 分数阶自适应滑模控制器类
class Controller_ZDT: public Controller
{
    Q_OBJECT
private:
    double dt;         // 时间步长
    double alpha_r;    // D^{1.13}r(t) 的阶数
    double alpha_y1;   // D^{0.24}y(t) 的阶数
    double alpha_e;    // D^{0.13}e(t) 的阶数
    double gamma;      // 自适应增益
    double lam;        // 误差增益
    double eps;        // 符号函数增益
    double k;          // 滑模增益
    double M;          // 干扰上界
    double phi;        // 饱和函数边界层

    std::vector<double> history_r; // 存储 r(t) 的历史值
    std::vector<double> history_y; // 存储 y(t) 的历史值
    std::vector<double> history_e; // 误差历史存储
    double theta_hat;              // 参数估计

    // 计算分数阶导数 (Grünwald-Letnikov 近似)
    double fractional_derivative(double alpha, const std::vector<double>& history);

    // 饱和函数代替符号函数以减少抖振
    double sign_function(double s);

public:
    // 构造函数
    Controller_ZDT(int n_in=1, int n_out=1);
    virtual void saveParas();                                       //保存控制器参数
    virtual void setParas(std::map<std::string, QVariant>);         //设置参数
    virtual void initialize();                                      //初始化控制器参数、相关变量
    virtual void get_yd(double t=-1);                               //根据当前时刻，轨迹参数paras，获取系统参考值   r，维数 m;
    virtual void set_y(std::vector<double> y);                      //获取系统输出值   y，维数 m
    virtual void get_u(std::vector<double>&u);                      //获取控制器输出   u，维数 p

    // 更新控制输入
    double update(double r, double y) {
        // 计算误差并存储历史数据
        double e = r - y;
        history_r.push_back(r);
        history_y.push_back(y);
        history_e.push_back(e);

        // 计算分数阶导数
        double Dr = fractional_derivative(alpha_r, history_r);
        double Dy1 = fractional_derivative(alpha_y1, history_y);
        double De = fractional_derivative(alpha_e, history_e);

        // 计算滑模面 s(t)
        double integral_e = 0.0;
        for (double val : history_e) {
            integral_e += val;
        }
        integral_e *= lam * dt;  // 积分项

        double s = integral_e + De;  // 滑模面

        // 更新参数估计
        theta_hat += gamma * Dy1 * s * dt;

        // 计算控制输入并限幅
        double u = 40.39 * (lam * e + Dr + theta_hat * Dy1 + eps * sign_function(s) + k * s);
        u = std::max(-1000.0, std::min(4000.0, u));

        // 限制历史数据长度（避免内存爆炸）
        const int max_history = 1000;
        if (history_r.size() > max_history) {
            history_r.erase(history_r.begin());
            history_y.erase(history_y.begin());
            history_e.erase(history_e.begin());
        }

        return u;
    }
};

#endif // CONTROLLER_ZDT_H
