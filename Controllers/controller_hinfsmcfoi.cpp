#include "controller_hinfsmcfoi.h"
using namespace std;
using namespace cv;
Controller_HinfSMCFOI::Controller_HinfSMCFOI(int n_in, int n_out):Controller(n_in, n_out)
{
    name = "HinfSMCFOI";
    y_th = 10;
    u1 = 0.05;
    TF = 10;
    order = 5;
    alpha = {0.4, 0.8, 0.6, 0.4, 0.6};
    xMat = Mat::zeros(order, 1, CV_64F);
    A = (Mat_<double>(5,5)<<0,1,0,0,0, 0,0,1,0,0, 0,0,0,1,0, 0,0,0,0,1, 0,24.0/708, -244.0/708, 0, 507.0/708);
    B = (Mat_<double>(5,1)<<0,0,0,0,100.0/708);
    Bw= (Mat_<double>(5,1)<<0,0,0,0,100.0/708);
    S = (Mat_<double>(1,5)<<0.1246,-0.0420,-0.0238,-0.0779,0.1414);
    Mat tmp = S*B;
    SB = tmp.at<double>(0,0);SBinv = 1.0/SB;SBw = SB;
    phi = 0.1; w = 0.1; mu = 0.1;
    uM[0] = 0.2;
    um[0] = -0.2;
    saveParas();
    initialize();
}
void Controller_HinfSMCFOI::saveParas()
{
    //保存参数到 qParas 中
    Controller::saveParas();
    QList<double> qlist;
    QVariant qVar;
    qParas["order"] = order;
    qlist = QList<double>::fromVector(QVector<double>(alpha.begin(), alpha.end()));
    qVar.setValue(qlist);
    qParas["alpha"] = qVar;
    qParas["phi"] = phi;
    qParas["w"] = w;
    qParas["mu"] = mu;
    qParas["y_th"] = y_th;
    qParas["u1"] = u1;
}
void Controller_HinfSMCFOI::setParas(map<string, QVariant> q)
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

    if(qParas.count("phi")>0)
        phi = qParas["phi"].toFloat();
    if(qParas.count("w")>0)
        w = qParas["w"].toFloat();
    if(qParas.count("mu")>0)
        mu = qParas["mu"].toFloat();
    if(qParas.count("y_th")>0)
        y_th = qParas["y_th"].toFloat();
    if(qParas.count("u1")>0)
        u1 = qParas["u1"].toFloat();
}

void Controller_HinfSMCFOI::initialize()
{
    // 计算辅助变量
    Controller::initialize();
    W = vector<vector<double>>(order, vector<double>(0, 0));
    //计算对于不同的 gamma， 对应的权重 W 数组
    X.clear();
    for(int i=0; i<order; ++i)
        X.push_back(vector<double>(0));
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
    }
    //初始状态
    for(int i=0; i<order; ++i)
        X.push_back(vector<double>(0));
}
void Controller_HinfSMCFOI::get_yd(double t)
{
    Controller::get_yd(t);
}
void Controller_HinfSMCFOI::set_y(vector<double> y_observe)
{
    assert(y_observe.size()>0);
    x.push_back(y_observe[0]-y_bias[0]);

    Controller::set_y(y_observe);
}
void Controller_HinfSMCFOI::get_u(vector<double>& u)
{
    u.clear();
    X[0].push_back(x.back());xMat.at<double>(0,0)=x.back();
    for (int i = 0; i < order - 1; i++)
    {
        X[i+1].push_back(ite_glfdiff(X[i], i));
        xMat.at<double>(i+1,0) = X[i+1].back();
    }
    Mat ksiMat=(S*xMat);
    double u_var = 0;
    double ksi =ksiMat.at<double>(0,0);
    double M = -SBinv*(phi*SB+w*SBw+mu);
    u_var += (ksi>0?M:-M);                            // sgn(s) M
    Mat tmp = -SBinv*S*A*xMat;
    u_var += tmp.at<double>(0,0);
    u.push_back(u_var);

    Controller::get_u(u);
}

// 辅助函数
double Controller_HinfSMCFOI::ite_glfdiff(vector<double> y, int i)//返回 y 的alpha[i] 阶导数的最后一项
{
    int n = y.size();
    double beta;
    if (i < order)
        beta = alpha[i];
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
vector<double> Controller_HinfSMCFOI::glfdiff(vector<double> y, double alpha)// 计算 Grünwald–Letnikov 分数阶微积分的函数
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
