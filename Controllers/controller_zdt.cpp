#include "controller_zdt.h"
using namespace std;
Controller_ZDT::Controller_ZDT(int n_in, int n_out):Controller(n_in, n_out)
{
    name = "ZDT";
    U_name = "p";
    dt = delta_t;
    alpha_r = 1.87;    // D^{1.13}r(t) 的阶数
    alpha_y1 = 0.66;   // D^{0.24}y(t) 的阶数
    alpha_e = 0.87;    // D^{0.13}e(t) 的阶数
    gamma = 0.1;      // 自适应增益
    lam = 5.0;        // 误差增益
    eps = 0.5;        // 符号函数增益
    k = 2.0;          // 滑模增益
    M = 10;          // 干扰上界 TODO
    phi = 0.1;        // 饱和函数边界层
    uM[0] = 8000;
    um[0] = -3000;
    saveParas();
    initialize();
    trajParas[0]["MODE"] = TRAJ_MODE_SLOPE;
    trajParas[0]["VM"]=20;
    trajParas[0]["XF"]=200;
}

// 二项式系数计算（广义，支持非整数阶）
double binom(double n, int k) {
    if (k < 0 || k > n) return 0.0;
    if (k == 0 || k == n) return 1.0;
    if (k > n - k) k = n - k; // 取较小的k以减少计算量

    double result = 1.0;
    for (int i = 0; i < k; ++i) {
        result *= (n - i) / (i + 1);
    }
    return result;
}
double Controller_ZDT::fractional_derivative(double alpha, const std::vector<double>& history) {
    int m = history.size() - 1;
    if (m < 1) return 0.0;

    double gl = 0.0;
    for (int k = 0; k <= m; ++k) {
        double coeff = std::pow(-1, k) * binom(alpha, k);
        gl += coeff * history[history.size() - k - 1];
    }

    return gl / std::pow(dt, alpha);
}
double Controller_ZDT::sign_function(double s) {
    if (std::abs(s) > phi) {
        return (s > 0) ? 1.0 : -1.0;
    }
    else {
        return s / phi;
    }
}
void Controller_ZDT::saveParas()
{
    //保存参数到 qParas 中
    Controller::saveParas();
    QVariant qVar;
    if(qParas.count("alpha_r")==0)
        qParas["alpha_r"] = alpha_r;
    if(qParas.count("alpha_y1")==0)
        qParas["alpha_y1"] = alpha_y1;
    if(qParas.count("alpha_e")==0)
        qParas["alpha_e"] = alpha_e;
    if(qParas.count("gamma")==0)
        qParas["gamma"] = gamma;
    if(qParas.count("lam")==0)
        qParas["lam"] = lam;
    if(qParas.count("eps")==0)
        qParas["eps"] = eps;
    if(qParas.count("k")==0)
        qParas["k"] = k;
    if(qParas.count("M")==0)
        qParas["M"] = M;
    if(qParas.count("phi")==0)
        qParas["phi"] = phi;
}
void Controller_ZDT::setParas(map<string, QVariant> q)
{
    Controller::setParas(q);
    // 自定义参数值
    if(qParas.count("alpha_r")>0)
        alpha_r = qParas["alpha_r"].toFloat();
    if(qParas.count("alpha_y1")>0)
        alpha_y1 = qParas["alpha_y1"].toFloat();
    if(qParas.count("alpha_e")>0)
        alpha_e = qParas["alpha_e"].toFloat();
    if(qParas.count("gamma")>0)
        gamma = qParas["gamma"].toFloat();
    if(qParas.count("lam")>0)
        lam = qParas["lam"].toFloat();
    if(qParas.count("eps")>0)
        eps = qParas["eps"].toFloat();
    if(qParas.count("k")>0)
        k = qParas["k"].toFloat();
    if(qParas.count("M")>0)
        M = qParas["M"].toFloat();
    if(qParas.count("phi")>0)
        phi = qParas["phi"].toFloat();
}

void Controller_ZDT::initialize()
{
    theta_hat = 0;
    Controller::initialize();
}
void Controller_ZDT::get_yd(double t)
{
    Controller::get_yd(t);
}
void Controller_ZDT::set_y(vector<double> y_observe)
{
    assert(y_observe.size()>0);

    Controller::set_y(y_observe);
}
void Controller_ZDT::get_u(vector<double>& u)
{
    u.clear();

    // 计算误差并存储历史数据
    double e = yd[0] - y[0];
    history_r.push_back(yd[0]);
    history_y.push_back(y[0]);
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
    double u_var = 40.39 * (lam * e + Dr + theta_hat * Dy1 + eps * sign_function(s) + k * s);

    // 限制历史数据长度（避免内存爆炸）
    const int max_history = 1000;
    if (history_r.size() > max_history) {
        history_r.erase(history_r.begin());
        history_y.erase(history_y.begin());
        history_e.erase(history_e.begin());
    }
    u.push_back(u_var);
    Controller::get_u(u);
}
