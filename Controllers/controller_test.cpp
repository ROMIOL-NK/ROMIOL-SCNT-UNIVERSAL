#include "controller_test.h"
using namespace std;

Controller_TEST::Controller_TEST(int n_in, int n_out):Controller(n_in, n_out)
{
    name = "TEST";
    U_name = "p";
    TF = 20;
    //设置默认参数
    alpha = 0.98;
    p_plus = 20;
    p_minus = -20;
    beta_plus = 0.1;
    beta_minus = 0.1;
    lambda = 1; mu = 0.6;
    k = 0;
    M = 10;          //最大扰动的绝对值
    lambda0 = 1e-4;
    lambda1 = 1e-5;
    eta = 10;

    uM[0] = 5000;
    um[0] = -2000;
    saveParas();
    initialize();
}
void Controller_TEST::saveParas()
{
    //保存参数到 qParas 中
    Controller::saveParas();
    qParas["alpha"] = alpha;
    qParas["M"] = M;
    qParas["k"] = k;
    qParas["lambda"] = lambda;
    qParas["mu"] = mu;
    qParas["p_plus"] = p_plus;
    qParas["p_minus"] = p_minus;
    qParas["beta_plus"] = beta_plus;
    qParas["beta_minus"] = beta_minus;

    qParas["lambda0"] = lambda0;
    qParas["lambda1"] = lambda1;
    qParas["eta"] = eta;
}
void Controller_TEST::setParas(map<string, QVariant> q)
{
    Controller::setParas(q);
    // 自定义参数值
    if(qParas.count("alpha")>0)
        alpha = qParas["alpha"].toFloat();
    if(qParas.count("M")>0)
        M = qParas["M"].toFloat();
    if(qParas.count("k")>0)
        k = qParas["k"].toFloat();
    if(qParas.count("lambda")>0)
        lambda = qParas["lambda"].toFloat();
    if(qParas.count("mu")>0)
        mu = qParas["mu"].toFloat();
    if(qParas.count("p_plus")>0)
        p_plus = qParas["p_plus"].toFloat();
    if(qParas.count("p_minus")>0)
        p_minus = qParas["p_minus"].toFloat();
    if(qParas.count("beta_plus")>0)
        beta_plus = qParas["beta_plus"].toFloat();
    if(qParas.count("beta_minus")>0)
        beta_minus = qParas["beta_minus"].toFloat();

    if(qParas.count("lambda0")>0)
        lambda0 = qParas["lambda0"].toFloat();
    if(qParas.count("lambda1")>0)
        lambda1 = qParas["lambda1"].toFloat();
    if(qParas.count("eta")>0)
        eta = qParas["eta"].toFloat();
}

void Controller_TEST::initialize()
{
    Controller::initialize();
    // 计算次级变量
    double beta = min(beta_plus, beta_minus);
    if(beta!=0)
        gamma = 1.0/beta;
    else
        gamma = 1;
    GammaAlpha_1 = std::tgamma(alpha-1);

    s = 0;
    W.clear();
    W[alpha]={1};
    W[alpha-1] = {1};
    for(int j=1; j*delta_t<=TF; j++)//计算 TF 时间内的权重
    {
        W[alpha].push_back(W[alpha][j - 1] * (1 - (alpha + 1) / j));
        W[alpha-1].push_back(W[alpha-1][j - 1] * (1 - alpha / j));
    }

    k0 = 0;
    k1 = 0;
}
void Controller_TEST::get_yd(double t)
{
    Controller::get_yd(t);
    xd.push_back(yd[0]);
}
void Controller_TEST::set_y(vector<double> y_observe)
{
    assert(y_observe.size()>0);

    Controller::set_y(y_observe);
}
void Controller_TEST::get_u(vector<double>& u)
{
    u.clear();
    double u_var = 0;

    e.push_back(y.back() - xd.back());

    // double coe = 0;
    // // if(k_step!=0)
    // //     coe = pow(k_step*delta_t, alpha-2)/GammaAlpha_1;
    // double tmp = ite_glfdiff(e, alpha-1)*coe, eps = 1e-4;
    // if(abs(e.back())>eps)
    //     tmp += (pow(e.back()*e.back(), -mu)+lambda/mu)*e.back();
    // intTerm.push_back(tmp);
    // s += delta_t*(ite_glfdiff(e, alpha)+ite_glfdiff(intTerm, alpha-1));
    // K = abs(ite_glfdiff(xd, alpha))+M+abs(ite_glfdiff(intTerm, alpha-1))+k;
    // if(s>0)
    //     u_var = -gamma*K+p_minus;
    // else if(s<0)
    //     u_var = gamma*K+p_plus;
    double K = k0+k1*abs(e.back());
    s+=delta_t*(eta*e.back()+ite_glfdiff(e, alpha));
    if(s>0)
        u_var = -gamma*K+p_minus;
    else if(s<0)
        u_var = gamma*K+p_plus;
    k0+=delta_t*lambda0*abs(s);
    k1+=delta_t*lambda1*abs(s)*abs(e.back());

    u.push_back(u_var);
    Controller::get_u(u);
}

// 辅助函数
double Controller_TEST::ite_glfdiff(vector<double> y, double a)//返回 y 的alpha[i] 阶导数的最后一项
{
    // int n = min(int(y.size()), 50);
    int n = y.size();
    double tmp = 0;
    for(int j=0; j<n;j++)
    {
        if(W[a].size()<=j)
            W[a].push_back(W[a][j-1]*(1-(a+1)/j));
        tmp += W[a][j]*y[n-1-j];
    }
    tmp/=pow(delta_t, alpha );
    return tmp;
}
