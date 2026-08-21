#include "controller_foasmc_rbf.h"
#include "controller_foasmc.h"
using namespace std;

Controller_FOASMC_RBF::Controller_FOASMC_RBF(int n_in, int n_out):Controller(n_in, n_out)
{
    name = "FOASMC_RBF";
    // 默认参数值
    order = 5;
    alpha = {0.5, 0.5, 0.5, 0.5, 0.5};
    K = {2, 10, 20, 10, 10};
    theta_hat0 = {0.0833, 0.1839, -3.9540, 9.5057, -5.6954};
    theta_hat = theta_hat0;
    GAMMA = {1e-10, 1e-06, 1e-06, 0.0001, 1e-10};
    bu = 0.005747;
    omega0 = vector<double>(10, 1);
    cRBF = vector<vector<double>>(2*order, vector<double>(order, 0));
    sigmaRBF = vector<double>(2*order, 0);
    for(int i=0; i<order; ++i)
    {
        cRBF[i][i] = 100;
        cRBF[order+i][i] = -100;
        sigmaRBF[i] = pow(10, 6);         //高阶导数数值偏大
        sigmaRBF[order+i] = pow(10, 6);
    }
    eta = 0.01;
    k = 10;
    M = 10;          //最大扰动的绝对值
    uM[0] = 0.2;
    um[0] = -0.2;
    saveParas();
    initialize();
}
void Controller_FOASMC_RBF::saveParas()
{
    //保存参数到 qParas 中
    Controller::saveParas();
    QList<double> qlist;
    QVariant qVar;
    qParas["order"] = order;
    qlist = QList<double>::fromVector(QVector<double>(alpha.begin(), alpha.end()));
    qVar.setValue(qlist);
    qParas["alpha"] = qVar;
    qlist = QList<double>::fromVector(QVector<double>(K.begin(), K.end()));
    qVar.setValue(qlist);
    qParas["K"] = qVar;
    qlist = QList<double>::fromVector(QVector<double>(theta_hat0.begin(), theta_hat0.end()));
    qVar.setValue(qlist);
    qParas["theta_hat0"] = qVar;
    qlist = QList<double>::fromVector(QVector<double>(GAMMA.begin(), GAMMA.end()));
    qVar.setValue(qlist);
    qParas["GAMMA"] = qVar;
    qParas["bu"] = bu;
    qParas["k"] = k;
    qParas["M"] = M;
}
void Controller_FOASMC_RBF::setParas(map<string, QVariant> q)
{
    Controller::setParas(q);
    // 自定义参数值
    if(qParas.count("order")>0)
        order = qParas["order"].toInt();
    QList<double> qlist;
    if(qParas.count("alpha")>0)
    {
        qlist = qParas["alpha"].value<QList<double>>();
        copy(qlist.begin(), qlist.end(), alpha.begin());
    }
    if(qParas.count("K")>0)
    {
        qlist = qParas["K"].value<QList<double>>();
        copy(qlist.begin(), qlist.end(), K.begin());
    }
    if(qParas.count("theta_hat0")>0)
    {
        qlist = qParas["theta_hat0"].value<QList<double>>();
        copy(qlist.begin(), qlist.end(), theta_hat0.begin());
    }
    if(qParas.count("GAMMA")>0)
    {
        qlist = qParas["GAMMA"].value<QList<double>>();
        copy(qlist.begin(), qlist.end(), GAMMA.begin());
    }

    if(qParas.count("bu")>0)
        bu = qParas["bu"].toFloat();
    if(qParas.count("k")>0)
        k = qParas["k"].toFloat();
    if(qParas.count("M")>0)
        M = qParas["M"].toFloat();
}

void Controller_FOASMC_RBF::initialize()
{
    // 计算辅助变量
    Controller::initialize();
    sumAlpha = accumulate(alpha.begin(), alpha.end(), 0.0);
    //计算对于不同的 gamma， 对应的权重 W 数组
    X.clear();
    for(int i=0; i<order; ++i)
        X.push_back(vector<double>(0));
    E.clear();
    for(int i=0; i<=order; ++i)
        E.push_back(vector<double>());
    W = vector<vector<double>>(order+2, vector<double>(0, 0));
    for(int i=0; i<order; i++)
            W[i].push_back(1);
    W[order].push_back(1);
    W[order+1].push_back(1);

    for(int j=1; j*delta_t<=TF; j++)//计算 TF 时间内的权重
    {
        for (int i = 0; i < order; i++)
            W[i].push_back(W[i][j - 1] * (1 - (alpha[i] + 1) / j));
        W[order].   push_back(W[order]  [j - 1] * (1 - (sumAlpha    + 1) / j));
        W[order+1]. push_back(W[order+1][j - 1] * (1 - (-sumAlpha   + 1) / j));
    }
    omega.assign(omega0.begin(), omega0.end());
}
void Controller_FOASMC_RBF::get_yd(double t)
{
    Controller::get_yd(t);
    r.push_back(yd[0]);
}
void Controller_FOASMC_RBF::set_y(vector<double> y_observe)
{
    assert(y_observe.size()>0);
    x.push_back(y_observe[0]-y_bias[0]);

    Controller::set_y(y_observe);
}
void Controller_FOASMC_RBF::get_u(vector<double>& u)
{
    u.clear();
    e.push_back(r.back() - x.back());
    E[0].push_back(e.back());
    for (int i = 0; i < order; i++)
        E[i+1].push_back(ite_glfdiff(E[i], i));
    X[0].push_back(x.back());
    for (int i = 0; i < order - 1; i++)
        X[i+1].push_back(ite_glfdiff(X[i], i));

    double s = 0;
    double u_var = 0;
    vector<double> tmp;
    for(int i=0; i<order; ++i)
    {
        s += K[i]*accumulate(E[i].begin(), E[i].end(), 0.0)*delta_t;
        u_var += K[i]*E[i].back();                  // Σ ki e_i
    }
    s += accumulate(E[order].begin(), E[order].end(), 0.0)*delta_t;
//    u_var += (s>0?M:-M);                            // sgn(s) M
    double M_est = est_boundary(s);
    u_var += (s>0?M_est:-M_est);
    drAlpha.push_back(ite_glfdiff(r, order));
    u_var += drAlpha.back();                        // dr_Alpha
    for(size_t i=0; i<theta_hat.size(); ++i)
    {
        theta_hat[i] -= GAMMA[i]*s*X[i].back()*delta_t;
        u_var -= theta_hat[i]*X[i].back();          // theta X
    }
    u_var += k*s;                                   // ks
    u.push_back(u_var/bu);
//    u_idea.push_back(u);
//    u_real.push_back(max(min(u, uM), um));

    Controller::get_u(u);
}

// 辅助函数
double Controller_FOASMC_RBF::ite_glfdiff(vector<double> y, int i)//返回 y 的alpha[i] 阶导数的最后一项
{
    int n = y.size();
    double beta;
    if (i < order)
        beta = alpha[i];
    else if (i == order)
        beta = sumAlpha;
    else if (i == order + 1)
        beta = -sumAlpha;
    double tmp = 0;
    for(int j=0; j<n;j++)
    {
        if(W[i].size()<=j)
            W[i].push_back(W[i][j-1]*(1-(beta+1)/j));
        tmp += W[i][j]*y[n-1-j];
    }
    tmp/=pow(delta_t, beta);
    return tmp;
}
vector<double> Controller_FOASMC_RBF::glfdiff(vector<double> y, double alpha)// 计算 Grünwald–Letnikov 分数阶微积分的函数
{
    int n = y.size();
    vector<double> w(n);  w[0]  = 1;
    vector<double> dy(n); dy[0] = y[0]/pow(delta_t, alpha);
    for(int i=1; i<n; i++)
    {
        w[i] = w[i-1]*(1-(alpha+1)/i);
        dy[i] = 0;
        for(int j=0; j<=i ;j++)
            dy[i] += w[j]*y[i-j];
        dy[i]/=pow(delta_t, alpha);
    }
    return dy;
}

double Controller_FOASMC_RBF::est_boundary(double s)
{
    int n=omega.size();
    vector<double> h(n, 0);
    for(int i=0; i<n; ++i)
        for(int j=0; j<order; ++j)
        {
            double tmp = pow(X[j].back()-cRBF[i][j], 2)/(2*sigmaRBF[i]*sigmaRBF[i]);
            h[i] += omega[i]*exp(tmp);
        }
    double M_est = 0;
    for(int i=0; i<n; ++i)
    {
        omega[i] += delta_t*eta*abs(s)*h[i];
        M_est += omega[i]*h[i];
    }
    return M_est;
}
