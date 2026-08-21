#include "controller_foasmc.h"
using namespace std;

Controller_FOASMC::Controller_FOASMC(int n_in, int n_out):Controller(n_in, n_out)
{
    name = "FOASMC";

    order = 5;
    // alpha = {0.5, 0.5, 0.5, 0.5, 0.5};
    alpha = {0.4, 0.8, 0.6, 0.4, 0.6};
    K = {2, 10, 20, 10, 10};
    // theta_hat0 = {1, 0.5, -1, 1, 1};
    // theta_hat0 = {14.5/174, 32.0/174, -688/174, 1654/174, -991/174};
    theta_hat0 = {0.0833, 0.1839, -3.9540, 9.5057, -5.6954};
    GAMMA = {1e-10, 1e-06, 1e-06, 0.0001, 1e-10};

    // order = 2;
    // alpha = {1, 1};
    // K = {2, 2};
    // theta_hat = {1, 1};
    // theta_hat0 = {1, 1};
    // GAMMA = {0.01,0.01};

    // order = 1;
    // alpha = {1.4};
    // K = {2};
    // theta_hat = {0};
    // theta_hat0 = {0};
    // GAMMA = {0.01};
    bu = 16666;

    k = 10;
    M = 30;          //最大扰动的绝对值
    uM[0] = 0.2;
    um[0] = -0.2;

    km = 100;
    isM_Adapt = true;
    saveParas();
    initialize();
}
void Controller_FOASMC::saveParas()
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
    qParas["km"] = km;
    qParas["isM_Adapt"] = isM_Adapt;
}
void Controller_FOASMC::setParas(map<string, QVariant> q)
{
    Controller::setParas(q);
    // 自定义参数值
    if(qParas.count("order")>0)
        order = qParas["order"].toInt();
    QList<double> qlist;
    if(qParas.count("alpha")>0)
    {
        qlist = qParas["alpha"].value<QList<double>>();
        alpha.resize(qlist.size());
        copy(qlist.begin(), qlist.end(), alpha.begin());
    }
    if(qParas.count("K")>0)
    {
        qlist = qParas["K"].value<QList<double>>();
        K.resize(qlist.size());
        copy(qlist.begin(), qlist.end(), K.begin());
    }
    if(qParas.count("theta_hat0")>0)
    {
        qlist = qParas["theta_hat0"].value<QList<double>>();
        theta_hat0.resize(qlist.size());
        copy(qlist.begin(), qlist.end(), theta_hat0.begin());
    }
    if(qParas.count("GAMMA")>0)
    {
        qlist = qParas["GAMMA"].value<QList<double>>();
        GAMMA.resize(qlist.size());
        copy(qlist.begin(), qlist.end(), GAMMA.begin());
    }

    if(qParas.count("bu")>0)
        bu = qParas["bu"].toFloat();
    if(qParas.count("k")>0)
        k = qParas["k"].toFloat();
    if(qParas.count("M")>0)
        M = qParas["M"].toFloat();
    if(qParas.count("km")>0)
        km = qParas["km"].toFloat();
    if(qParas.count("isM_Adapt")>0)
        isM_Adapt = qParas["isM_Adapt"].toBool();
}

void Controller_FOASMC::initialize()
{
    Controller::initialize();
    // 计算辅助变量
    if(!isInitialized||!isSaveLastPara)
    {
        theta_hat = theta_hat0;
        isInitialized = true;
    }
    sumAlpha = accumulate(alpha.begin(), alpha.end(), 0.0);
    W = vector<vector<double>>(order, vector<double>(0, 0));
    //计算对于不同的 gamma， 对应的权重 W 数组
    X.clear();
    for(int i=0; i<order; ++i)
        X.push_back(vector<double>(0));
    E.clear();
    for(int i=0; i<=order; ++i)
        E.push_back(vector<double>());
    for(int i=0; i<order; i++)
        W[i].push_back(1);
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

    //初始状态
    for(int i=0; i<order; ++i)
        X.push_back(vector<double>(0));
    for(int i=0; i<=order; ++i)
        E.push_back(vector<double>());
}
void Controller_FOASMC::get_yd(double t)
{
    Controller::get_yd(t);
    r.push_back(yd[0]);
}
void Controller_FOASMC::set_y(vector<double> y_observe)
{
    assert(y_observe.size()>0);
    x.push_back(y_observe[0]-y_bias[0]);

    Controller::set_y(y_observe);
}
void Controller_FOASMC::get_u(vector<double>& u)
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
    if(isM_Adapt)
        M += km*abs(s);
    u_var += (s>0?M:-M);                            // sgn(s) M
    drAlpha.push_back(ite_glfdiff(r, order));
    u_var += drAlpha.back();                        // dr_Alpha
    for(size_t i=0; i<theta_hat.size(); ++i)
    {
        theta_hat[i] -= GAMMA[i]*s*X[i].back()*delta_t;
        u_var -= theta_hat[i]*X[i].back();          // theta X
    }
    u_var += k*s;                                   // ks
    u.push_back(u_var/bu);

    Controller::get_u(u);
}

// 辅助函数
double Controller_FOASMC::ite_glfdiff(vector<double> y, int i)//返回 y 的alpha[i] 阶导数的最后一项
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
vector<double> Controller_FOASMC::glfdiff(vector<double> y, double alpha)// 计算 Grünwald–Letnikov 分数阶微积分的函数
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
