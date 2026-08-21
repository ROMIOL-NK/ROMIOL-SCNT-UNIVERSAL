#include "controller_mrfoasmc_rbf.h"
#include "Objects/injectpipette.h"
using namespace std;
using namespace cv;

Controller_MRFOASMC_RBF::Controller_MRFOASMC_RBF(int n_in, int n_out):Controller(n_in, n_out)
{
    name = "MRFOASMC_RBF";
    // 默认参数值
    order = 3;
    alpha = {0.5, 0.7, 0.8};
    // K = {-5, -0.7, -10};
    K = {-600, -500, -100};
    // K = {-10, -5, -2};
    theta_hat = {-1, -1, -1};// 3.7305*s^2 - 20.8144*s^1.2 + 41.7518*s^0.5 - 25.8527
    bu = 1;
    // theta_hat = {6.9301, -11.1920, 5.5795};// 3.7305*s^2 - 20.8144*s^1.2 + 41.7518*s^0.5 - 25.8527
    // bu = 0.2681;
    uM[0] = 0.2;
    um[0] = -0.2;
    saveParas();
    eta = 0.01;
    initialize();
}
void Controller_MRFOASMC_RBF::saveParas()
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
    qlist = QList<double>::fromVector(QVector<double>(theta_hat.begin(), theta_hat.end()));
    qVar.setValue(qlist);
    qParas["theta_hat"] = qVar;
    qParas["bu"] = bu;
    qParas["eta"] = eta;
}
void Controller_MRFOASMC_RBF::setParas(map<string, QVariant> q)
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
    if(qParas.count("theta_hat")>0)
    {
        qlist = qParas["theta_hat"].value<QList<double>>();
        copy(qlist.begin(), qlist.end(), theta_hat.begin());
    }
    if(qParas.count("bu")>0)
        bu = qParas["bu"].toFloat();
    if(qParas.count("eta")>0)
        eta = qParas["eta"].toFloat();
}

void Controller_MRFOASMC_RBF::initialize()
{
    Controller::initialize();
    K1 = Mat::ones(order, 1, CV_64F);
    for(int i=0; i<order; ++i)
        K1.at<double>(i,0) = (K[i]-theta_hat[i])/bu;

    betaa = vector<double>(order+1);
    betaa[0] = 0;
    for(int i=1; i<=order; ++i)
        betaa[i] = accumulate(alpha.begin(), alpha.begin()+i, 0.0);
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
    xmMat= Mat::zeros(order, 1, CV_64F);
    eMat = Mat::zeros(order, 1, CV_64F);
    deMat= Mat::zeros(order, 1, CV_64F);
    sMat = Mat::zeros(order, 1, CV_64F);
    //设置 A,B, Am, Bm
    A = Mat::zeros(order, order, CV_64F);
    B = Mat::zeros(order, 1, CV_64F);
    Am = Mat::zeros(order, order, CV_64F);
    Bm = Mat::zeros(order, 1, CV_64F);
    for(int i=1; i<order; ++i)
    {
        A.at<double>(i-1,i) = 1;
        Am.at<double>(i-1,i) = 1;
        A.at<double>(order-1, i) = theta_hat[i];
        Am.at<double>(order-1, i) = K[i];
    }
    A.at<double>(order-1, 0) = theta_hat[0];
    Am.at<double>(order-1, 0) = K[0];
    B.at<double>(order-1, 0) = bu;
    Bm.at<double>(order-1, 0) = 1;

    // 计算辅助变量
    //计算对于不同的 gamma， 对应的权重 W 数组
    x.clear();xm.clear();e.clear();
    X.clear();
    for(int i=0; i<order; ++i)
        X.push_back(vector<double>(0));
    E.clear();
    for(int i=0; i<=order; ++i)
        E.push_back(vector<double>());
    Xm.clear();
    for(int i=0; i<=order; ++i)
        Xm.push_back(vector<double>());
//    Xm[0].push_back(-100);

    W = vector<vector<double>>(order+1, vector<double>(0, 0));
    for(int i=0; i<order; i++)
        W[i].push_back(1);
    W[order].push_back(1);
    for(int j=1; j*delta_t<=TF; j++)//计算 TF 时间内的权重
        for (int i = 0; i <= order; i++)
            W[i].push_back(W[i][j - 1] * (1 - (betaa[i] + 1) / j));
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
void Controller_MRFOASMC_RBF::get_yd(double t)
{
    if(k_step==0)
    {
        if(!x.empty())
            xm.push_back(0.7*x.back());
        else
            xm.push_back(0);
    }
    else
    {
        //公式计算法
        double tmp = 0, diTmp;
        int n = xm.size();
        for(int i=0; i<order; ++i)
        {
            diTmp = 0;
            for(int j=1; j<=n; ++j)
            {
                if(W[i].size()<=j)
                    W[i].push_back(W[i][j-1]*(1-(betaa[i]+1)/j));
                diTmp += W[i][j]*xm[n-j];
            }
            tmp += K[i]*diTmp/pow(delta_t, betaa[i]);
        }
        diTmp = 0;
        for(int j=1; j<=n; ++j)
        {
            if(W[order].size()<=j)
                W[order].push_back(W[order][j-1]*(1-(betaa[order]+1)/j));
            diTmp += W[order][j]*xm[n-j];
        }
        tmp -= diTmp/pow(delta_t, betaa[order]);
        double coe = 1/pow(delta_t, betaa[order]);
        for(int i=0; i<order; ++i)
            coe -= K[i]/pow(delta_t, betaa[i]);
        xm.push_back(tmp/coe);

        // // 记录数据//根据当前时刻创建文件夹保存数据
        // string path = string("F:/Public/");
        // ofstream txtfile(path+"data.txt");
        // txtfile.setf(ios::fixed, ios::floatfield);   txtfile.precision(4);
        // //写入数据
        // for(int k=0; k<xm.size(); ++k)
        // {
        //     txtfile<<k<<"\t";
        //     txtfile<<xm[k]<<endl;
        // }
        // txtfile.close();
    }

    //直接积分法
    // if(k_step==0)// xm, Xm[0~order]计算
    // {
    //     xm.push_back(x[0]);
    //     Xm[0].push_back(xm.back());
    //     for (int i = 1; i < order; i++)
    //         Xm[i].push_back(0);
    //     Xm[order].push_back(K[0]*Xm[0].back());
    // }
    // else
    // {
    //     xm.push_back(ite_glfdiff(Xm[order], order+1));
    //     Xm[0].push_back(xm.back());
    //     double tmp = Xm[0].back()*K[0];
    //     for (int i = 0; i < order-1; i++)
    //     {
    //         Xm[i+1].push_back(ite_glfdiff(Xm[i], i));
    //         tmp += K[i+1]*Xm[i+1].back();
    //     }
    //     Xm[order].push_back(tmp);
    // }

    yd.clear();
    yd.push_back(xm[k_step]);
    yd_history.push_back(yd);
}
void Controller_MRFOASMC_RBF::set_y(vector<double> y_observe)//y 的期望位置是 0，外部程序默认传入的初始值是 0
{
    assert(y_observe.size()>0);
    x.push_back(y_observe[0]-y_bias[0]);
    Controller::set_y(y_observe);
}
void Controller_MRFOASMC_RBF::get_u(vector<double>& u)
{
    u.clear();
    double u_var = 0;
    // 假设 xm 已经计算得到, x 已经观测得到
    e.push_back(x.back() - xm.back());
    E[0].push_back(e.back());
    for (int i = 0; i < order; i++)
        E[i+1].push_back(ite_glfdiff(E[i], i));
    X[0].push_back(x.back());
    for (int i = 0; i < order - 1; i++)
        X[i+1].push_back(ite_glfdiff(X[i], i));
    for(int i=0; i<order; ++i)
    {
        xMat.at<double>(i,0) = X[i].back();
        eMat.at<double>(i,0) = E[i].back();
        deMat.at<double>(i,0) = E[i+1].back();
    }
    sMat = sMat + delta_t*(deMat-Am*eMat);

    double M_est = est_boundary(sMat);
    u_var = (sMat.at<double>(order-1,0)>0? -M_est: M_est);

    K1 = K1 - delta_t*(xMat*sMat.t()*Bm);
    Mat tmp = K1.t()*xMat;
    u_var += tmp.at<double>(0,0);
    u.push_back(u_var);
    Controller::get_u(u);
}

// 辅助函数
double Controller_MRFOASMC_RBF::ite_glfdiff(vector<double> y, int i)//返回 y 的alpha[i] 阶导数的最后一项
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
vector<double> Controller_MRFOASMC_RBF::glfdiff(vector<double> y, double alpha, double h)// 计算 Grünwald–Letnikov 分数阶微积分的函数
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
vector<Mat> Controller_MRFOASMC_RBF::glfdiffMat(vector<Mat> x, vector<double> alpha, double h)
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

double Controller_MRFOASMC_RBF::est_boundary(Mat s)
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
        omega[i] += delta_t*eta*norm(s)*h[i];
        M_est += omega[i]*h[i];
    }
    return M_est;
}
void Controller_MRFOASMC_RBF::sim_y(vector<double> u)
{
    double tmp = 0, diTmp;
    int n = x.size();
    for(int i=0; i<order; ++i)
    {
        diTmp = 0;
        for(int j=1; j<=n; ++j)
        {
            if(W[i].size()<=j)
                W[i].push_back(W[i][j-1]*(1-(betaa[i]+1)/j));
            diTmp += W[i][j]*x[n-j];
        }
        tmp += theta_hat[i]*diTmp/pow(delta_t, betaa[i]);
    }
    diTmp = 0;
    for(int j=1; j<=n; ++j)
    {
        if(W[order].size()<=j)
            W[order].push_back(W[order][j-1]*(1-(betaa[order]+1)/j));
        diTmp += W[order][j]*x[n-j];
    }
    tmp -= diTmp/pow(delta_t, betaa[order]);
    tmp += bu*(u[0]+500*sin(k_step*delta_t));
    double coe = 1/pow(delta_t, betaa[order]);
    for(int i=0; i<order; ++i)
        coe -= theta_hat[i]/pow(delta_t, betaa[i]);
    x.push_back(tmp/coe);
    vector<double> y_tmp;
    y_tmp.push_back(x.back());
    Controller::set_y(y_tmp);
}
