#include "controller_fuzzy_foasmc_rbf_pi.h"
using namespace std;
using namespace cv;

Controller_FUZZY_FOASMC_RBF_PI::Controller_FUZZY_FOASMC_RBF_PI(int n_in, int n_out):Controller(n_in, n_out)
{
    name = "Fuzzy_FOASMC_RBF_PI";
    // 默认参数值
    alpha = {0.36, 0.98, 0.66, 0.42, 0.36};
    order = alpha.size();
    uM[0] = 0.2;
    um[0] = -0.2;
    //    iuM = 0;
    //    iuM = -300;
    l1 = 0.1;    l2 = 0.15;    l3 = 0.01;    ksi = 0.01;
    TF = 5;     si = 0;     beta1 = 0;  beta2 = 0;  mu = 0;

    Ah = (Mat_<double>(5, 5) << 0,1,0,0,0, 0,0,1,0,0, 0,0,0,1,0, 0,0,0,0,1, 0,24/708,-244/708,0,507/708);
    Bh = (Mat_<double>(5, 1) << 0.8, -3.4, 2.5, 3.0, -4.5);
    Kh = 1e2*(Mat_<double>(1, 5) << 0,0,0,0,100/708);
    saveParas();
    initialize();
}
void Controller_FUZZY_FOASMC_RBF_PI::saveParas()
{
    //保存参数到 qParas 中
    Controller::saveParas();
    QList<double> qlist;
    QVariant qVar;
    qlist = QList<double>::fromVector(QVector<double>(alpha.begin(), alpha.end()));
    qVar.setValue(qlist);
    qParas["alpha"] = qVar;
    qParas["l1"] = l1;
    qParas["l2"] = l2;
    qParas["l3"] = l3;
    qParas["ksi"] = ksi;
}
void Controller_FUZZY_FOASMC_RBF_PI::setParas(map<string, QVariant> q)
{
    Controller::setParas(q);
    // 自定义参数值
    QList<double> qlist;
    if(qParas.count("alpha")>0)
    {
        qlist = qParas["alpha"].value<QList<double>>();
        copy(qlist.begin(), qlist.end(), alpha.begin());
    }
    if(qParas.count("l1")>0)
        l1 = qParas["l1"].toFloat();
    if(qParas.count("l2")>0)
        l2 = qParas["l2"].toFloat();
    if(qParas.count("l3")>0)
        l3 = qParas["l3"].toFloat();
    if(qParas.count("ksi")>0)
        ksi = qParas["ksi"].toFloat();
    order = alpha.size();
}
void Controller_FUZZY_FOASMC_RBF_PI::initialize()
{
    Controller::initialize();
    order = alpha.size();
    // RBF
    omega0 = vector<double>(2*order, 1);
    cRBF = vector<vector<double>>(2*order, vector<double>(order, 0));
    sigmaRBF = vector<double>(2*order, 0);
    for(int i=0; i<order; ++i)
    {
        cRBF[i][i] = 100;
        cRBF[order+i][i] = -100;
        sigmaRBF[i] = pow(10, 6);         //高阶导数数值偏大
        sigmaRBF[order+i] = pow(10, 6);
    }

    xMat = Mat::zeros(order, 1, CV_64F);
    dxMat = Mat::zeros(order, 1, CV_64F);
    sMat = Mat::zeros(1, 1, CV_64F);

    x.clear();    X.clear();
    for(int i=0; i<=order; ++i)
        X.push_back(vector<double>(0));

    // 计算辅助变量
    //计算对于不同的 gamma， 对应的权重 Wa 数组
    Wa = vector<vector<double>>(order+2, vector<double>(0, 0));
    for(int i=0; i<order; i++)
        Wa[i].push_back(1);
    Wa[order].push_back(1);
    Wa[order+1].push_back(1);
    for(int j=1; j*delta_t<=TF; j++)//计算 TF 时间内的权重
    {
        for (int i = 0; i < order; i++)
            Wa[i].push_back(Wa[i][j - 1] * (1 - (alpha[i] + 1) / j));
        Wa[order].push_back(Wa[order][j - 1] * (1 - (sumAlpha    + 1) / j));
        Wa[order+1].push_back(Wa[order+1][j - 1] * (1 - (-sumAlpha + 1) / j));
    }
    omega.assign(omega0.begin(), omega0.end());
}
void Controller_FUZZY_FOASMC_RBF_PI::get_yd(double t)
{
    yd.clear();
    yd.push_back(0);
    yd_history.push_back(yd);
}
void Controller_FUZZY_FOASMC_RBF_PI::set_y(vector<double> y_observe)//y 的期望位置是 0，外部程序默认传入的初始值是 0
{
    assert(y_observe.size()>0);
    x.push_back(y_observe[0]-y_bias[0]);
    Controller::set_y(y_observe);
}
void Controller_FUZZY_FOASMC_RBF_PI::get_u(vector<double>& u)
{
    u.clear();
    double u_var = 0;
    // 假设 x 已经观测得到
    X[0].push_back(x.back());
    for (int i = 0; i < order; i++)
        X[i+1].push_back(ite_glfdiff(X[i], i));
    for(int i=0; i<order; ++i)
    {
        xMat.at<double>(i,0) = X[i].back();
        dxMat.at<double>(i,0) = X[i+1].back();
    }

    double f_est = est_boundary(sMat);
    sMat = sMat + delta_t*(dxMat-(Ah+Bh*Kh)*xMat);
    s = sMat.at<double>(0,0);
    double p = beta1*s+beta2*si;
    double M = mu+ksi;
    if(s>psi)
        u_var = (s>0? -M: M);
    else
        u_var = - M*p;
    Mat tmpMat = Kh*xMat-f_est;
    u_var+= tmpMat.at<double>(0,0);
    u.push_back(u_var);
    Controller::get_u(u);
    // update law
    beta1 += delta_t*M*s*s;
    beta2 += delta_t*M*s*si;
    mu += l2*abs(s);

    si+= delta_t*s;
}

// 辅助函数
double Controller_FUZZY_FOASMC_RBF_PI::ite_glfdiff(vector<double> y, int i)//返回 y 的alpha[i] 阶导数的最后一项
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
        if(Wa[i].size()<=j)
            Wa[i].push_back(Wa[i][j-1]*(1-(beta+1)/j));
        tmp += Wa[i][j]*y[n-1-j];
    }
    tmp/=pow(delta_t, beta);
    return tmp;
}
vector<double> Controller_FUZZY_FOASMC_RBF_PI::glfdiff(vector<double> y, double alpha, double h)// 计算 Grünwald–Letnikov 分数阶微积分的函数
{
    int n = y.size();
    vector<double> w(n);  w[0]  = 1;
    vector<double> dy(n); dy[0] = y[0]/pow(h, alpha);
    if(h<=0)
        h = delta_t;
    for(int i=1; i<n; i++)
    {
        w[i] = w[i-1]*(1-(alpha+1)/i);
        dy[i] = 0;
        for(int j=0; j<=i ;j++)
            dy[i] += w[j]*y[i-j];
        dy[i]/=pow(h, alpha);
    }
    return dy;
}
vector<Mat> Controller_FUZZY_FOASMC_RBF_PI::glfdiffMat(vector<Mat> x, vector<double> alpha, double h)
{
    int n = x.size();
    vector<Mat> w(n);
    vector<Mat> dy(n);
    if(h<=0)
        h = delta_t;
    for(int i=0; i<n; ++i)
    {
        w[i] = Mat::ones(order, 1, CV_64F);
        dy[i] = Mat::zeros(order, 1, CV_64F);
    }
    for(int ix=0; ix<order; ++ix)//遍历向量 x 中的每一个维度
        dy[0].at<double>(ix,0) = x[0].at<double>(ix,0)/pow(h, alpha[ix]);
    for(int ix; ix<order; ++ix)
        for(int i=1; i<n; i++)
        {
            w[i].at<double>(ix,0) = w[i-1].at<double>(ix,0)*(1-(alpha[ix]+1)/i);
            dy[i].at<double>(ix,0) = 0;
            for(int j=0; j<=i ;j++)
                dy[i].at<double>(ix,0) += w[j].at<double>(ix,0)*x[i-j].at<double>(ix,0);
            dy[i].at<double>(ix,0)/=pow(h, alpha[ix]);
        }
    return dy;
}
double Controller_FUZZY_FOASMC_RBF_PI::est_boundary(Mat s)
{
    int n=omega.size();
    vector<double> h(n, 0);
    for(int i=0; i<n; ++i)
    {
        double tmp = 0;
        for(int j=0; j<order; ++j)
            tmp += pow(X[j].back()-cRBF[i][j], 2);
        h [i] = exp(tmp/(2*sigmaRBF[i]*sigmaRBF[i]));
    }
    double f_est = 0;
    for(int i=0; i<n; ++i)
    {
        f_est += omega[i]*h[i];
        omega[i] += delta_t*l1*norm(s)*h[i];
    }
    return f_est;
}
