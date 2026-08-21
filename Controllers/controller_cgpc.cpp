#include "controller_cgpc.h"
using namespace cv;
using namespace std;


Controller_CGPC::Controller_CGPC(int n_in, int n_out):Controller(n_in, n_out)
{
    name = "CGPC";
    // U_name = "pos";     //为了便于处理约束，GPC的默认控制量选为 电机位置
    //默认参数值
    na = 5;     nb = 2;
    N = 20;     Nu = 5;
    lambda = 0.8;
    softenCoe=0.5;
    if(n_out==1)
        thetaDefault = {0.64,0.32,0.04,0,0,  1,1};
    else if(n_out==2)
        thetaDefault = {0.64,0.32,0.04,0,0, 0,0,0,0,0,  1,1,  0.2,0.2,0.2,0.2,0.2, 0,0,0,0,0,  1,1};
    theta0exp = thetaDefault;
    for(int i=0; i<p; ++i)
    {
        dum.push_back(-delta_t*0.5);
        duM.push_back( delta_t*0.5);
    }
    uM.clear();um.clear();
    for(int i=0; i<p; ++i)
    {
        uM.push_back( 0.2);
        um.push_back(-0.2);
        u_real.push_back(0);
    }
    saveParas();
    initialize();
}
void Controller_CGPC::saveParas()
{
    //保存参数到 qParas 中
    QList<double> qlist;
    QVariant qVar;
    qParas["na"] = na;
    qParas["nb"] = nb;
    qParas["N"] = N;
    qParas["Nu"] = Nu;
    qParas["lambda"] = lambda;
    qParas["softenCoe"] = softenCoe;
    qlist = QList<double>::fromVector(QVector<double>(duM.begin(), duM.end()));
    qVar.setValue(qlist);
    qParas["duM"] = qVar;
    qlist = QList<double>::fromVector(QVector<double>(dum.begin(), dum.end()));
    qVar.setValue(qlist);
    qParas["dum"] = qVar;
    qlist = QList<double>::fromVector(QVector<double>(thetaDefault.begin(), thetaDefault.end()));
    qVar.setValue(qlist);
    qParas["thetaDefault"] = qVar;
    qlist = QList<double>::fromVector(QVector<double>(theta0exp.begin(), theta0exp.end()));
    qVar.setValue(qlist);
    qParas["theta0exp"] = qVar;
}
void Controller_CGPC::setParas(map<string, QVariant> q)
{
    Controller::setParas(q);

    qParas = q;
    QList<double> qlist;
    QVariant qVar;
    if(qParas.count("duM")>0)
    {
        qlist = qParas["duM"].value<QList<double>>();
        copy(qlist.begin(), qlist.end(), duM.begin());
    }
    if(qParas.count("dum")>0)
    {
        qlist = qParas["dum"].value<QList<double>>();
        copy(qlist.begin(), qlist.end(), dum.begin());
    }
    if(qParas.count("thetaDefault")>0)
    {
        qlist = qParas["thetaDefault"].value<QList<double>>();
        copy(qlist.begin(), qlist.end(), thetaDefault.begin());
    }
    if(qParas.count("theta0exp")>0)
    {
        qlist = qParas["theta0exp"].value<QList<double>>();
        copy(qlist.begin(), qlist.end(), theta0exp.begin());
    }
    if(qParas.count("na")>0)
        na = qParas["na"].toInt();
    if(qParas.count("nb")>0)
        nb = qParas["nb"].toInt();
    if(qParas.count("N")>0)
        N = qParas["N"].toInt();
    if(qParas.count("Nu")>0)
        Nu = qParas["Nu"].toInt();

    if(qParas.count("lambda")>0)
        lambda = qParas["lambda"].toFloat();
    if(qParas.count("softenCoe")>0)
        softenCoe = qParas["softenCoe"].toFloat();
}
void Controller_CGPC::initialize()
{
    Controller::initialize();
    //变量初始化
    int sizeA[3] = {m, m, na+1};
    int sizeB[3] = {m, p, nb};
    int sizeE[3] = {m, m, N};
    //丢番图方程求解变量初始化
    ej = Mat::zeros(m, m, CV_64F);
    E = Mat::zeros(3, sizeE, CV_64F);
    for(int i=0; i<m; ++i)
        E.at<double>(i, i, 0) = 1;
    F = vector<Mat>(N);
    H = vector<Mat>(N);
    Gm=Mat::zeros(N*m, Nu*p, CV_64F);
    Omega = lambda*Mat::eye(Nu*p, Nu*p, CV_64F);

    // GPC 求解关键变量
    Yk_ = vector<Mat>(na, Mat::zeros(m, 1, CV_64F));
    Uk_ = vector<Mat>(nb, Mat::zeros(p, 1, CV_64F));
    yk = Mat::zeros(m, 1, CV_64F);
    Yd = Mat::zeros(N*m, 1, CV_64F);
    Y0 = Mat::zeros(N*m, 1, CV_64F);
    inv = Mat::zeros(Nu*p, Nu*p, CV_64F);
    duN = Mat::zeros(Nu*p, 1, CV_64F);
    //自校正变量
    phi  =Mat::zeros(m*na+p*nb, 1, CV_64F);
    theta_iy=Mat::zeros(m*na+p*nb, 1, CV_64F);
    if(!isInitialized||!isSaveLastPara)//并未进行任何参数初始化
    {
        theta=Mat::zeros(m, m*na+p*nb, CV_64F);
        theta0exp.clear();
        for(int i=0; i<thetaDefault.size(); ++i)
            theta0exp.push_back(thetaDefault[i]);
        for(int iy=0; iy<m; ++iy)
        {
            for(int i=0; i<m; ++i)
                for(int l=0; l<na; ++l)
                    theta.at<double>(iy, i*na+l) = theta0exp[iy*(m*na+p*nb) + i*na+l];
            for(int i=0; i<p; ++i)
                for(int l=0; l<nb; ++l)
                    theta.at<double>(iy, m*na + i*nb+l) = theta0exp[iy*(m*na+p*nb) + m*na + i*nb+l];
        }
        isInitialized = true;
    }
    else
    {
        for(int iy=0; iy<m; ++iy)
        {
            for(int i=0; i<m; ++i)
                for(int l=0; l<na; ++l)
                    theta0exp[iy*(m*na+p*nb) + i*na+l] = theta.at<double>(iy, i*na+l);
            for(int i=0; i<p; ++i)
                for(int l=0; l<nb; ++l)
                    theta0exp[iy*(m*na+p*nb) + m*na + i*nb+l] = theta.at<double>(iy, m*na + i*nb+l);
        }
    }
    P = vector<Mat>(m, 1e-2*Mat::eye(m*na+p*nb, m*na+p*nb, CV_64F));
    A = Mat::ones(3, sizeA, CV_64F);    B = Mat::ones(3, sizeB, CV_64F);
    for(int iy=0; iy<m; ++iy)
    {
        for(int i=0; i<m; ++i)
            for(int j=0; j<na; ++j)
                A.at<double>(iy,i,j+1) = -theta.at<double>(iy, i*na+j);
        for(int i=0; i<p; ++i)
            for(int j=0; j<nb; ++j)
                B.at<double>(iy,i,j) = theta.at<double>(iy, m*na + i*nb+j);
    }
    multipleDelta(A, Aa);

    Ast = Mat::zeros(Nu*p*2, Nu*p, CV_64F);
    duMin = Mat::zeros(Nu*p*2, 1, CV_64F);
    duMax = Mat::zeros(Nu*p*2, 1, CV_64F);
    for(int i=0; i<Nu; ++i)
        for(int j=0; j<p; ++j)
        {
            duMin.at<double>(i*p+j, 0) = dum[j];
            duMin.at<double>(Nu*p+i*p+j, 0) = um[j] - u_real[j];
            duMax.at<double>(i*p+j, 0) = duM[j];
            duMax.at<double>(Nu*p+i*p+j, 0) = uM[j] - u_real[j];
            Ast.at<double>(i*p+j, i*p+j) = 1.0;
            for(int k=0; k<=i; ++k)
                Ast.at<double>(Nu*p+i*p+j, k*p+j) = 1.0;
        }
}
void Controller_CGPC::get_yd(double t)
{
    Controller::get_yd(t);
    yr = Mat::zeros(m, 1, CV_64F);
    for(int i=0; i<m; ++i)
        yr.at<double>(i, 0) = yd[i];
}
void Controller_CGPC::set_y(std::vector<double> y)// -> yk
{
    for(int i=0; i<m; ++i)
        yk.at<double>(i,0) = y[i]-y_bias[i];

    Controller::set_y(y);
}

void Controller_CGPC::get_u(std::vector<double>&u)
{
    u.clear();
    //逐步迭代仿真 假定 参考值和观测值均已获得
    //get_yd() -> yr
    //set_y()  -> yk
    // -> Yd
    for(int j=0; j<m; ++j)
        Yd.at<double>(j,0) = softenCoe*yk.at<double>(j,0)+(1-softenCoe)*yr.at<double>(j,0);
    for(int i=1; i<N; ++i)
        for(int j=0; j<m; ++j)
            Yd.at<double>(i*m+j, 0) = softenCoe*Yd.at<double>((i-1)*m+j, 0)+(1-softenCoe)*yr.at<double>(j,0);

    //-------------------------------------参数校正----------------------------------------
    for(int i=0; i<m; ++i)
        for(int j=0; j<na; ++j)
            phi.at<double>(i*na+j, 0) = Yk_[j].at<double>(i, 0);
    for(int i=0; i<p; ++i)
        for(int j=0; j<nb; ++j)
            phi.at<double>( m*na + i*nb+j, 0) = Uk_[j].at<double>(i, 0);
    for(int iy=0; iy<m; ++iy)
    {
        for(int i=0; i<m; ++i)
            for(int l=0; l<na; ++l)
                theta_iy.at<double>(i*na+l, 0) = -A.at<double>(iy,i,l+1);
        for(int i=0; i<p; ++i)
            for(int l=0; l<nb; ++l)
                theta_iy.at<double>( m*na + i*nb+l, 0) = B.at<double>(iy,i,l);
        //            cout<<"P: "<<endl<<P[iy]<<endl;            cout<<"theta_iy: "<<endl<<theta_iy<<endl;
        double innovation = yk.at<double>(iy,0) - theta_iy.dot(phi);
        P[iy] = P[iy]-P[iy]*(phi*phi.t())*P[iy]/(1+phi.dot(P[iy]*phi));
        theta_iy = theta_iy+P[iy]*phi*innovation;
        //            theta_iy = theta_iy+P[iy]*phi/(1+phi.dot(P[iy]*phi))*innovation;
        //            P[iy] = P[iy]-P[iy]*(phi*phi.t())*P[iy]/(1+phi.dot(P[iy]*phi));
        //更新 A,B
        for(int i=0; i<m; ++i)
            for(int j=0; j<na; ++j)
                A.at<double>(iy,i,j+1) = -theta_iy.at<double>(i*na+j, 0);
        for(int i=0; i<p; ++i)
            for(int j=0; j<nb; ++j)
                B.at<double>(iy,i,j) = theta_iy.at<double>( m*na + i*nb+j, 0);

        //更新theta
        for(int i=0; i<theta_iy.rows; ++i)
            theta.at<double>(iy,i) = theta_iy.at<double>(i,0);
        QList<double> qlist;
        QVariant qVar;
        qlist = QList<double>::fromVector(QVector<double>(theta0exp.begin(), theta0exp.end()));
        qVar.setValue(qlist);
        qParas["theta0exp"] = qVar;
    }
    multipleDelta(A, Aa);
    //-----------------------------------参数校正结束----------------------------------------

    //    求解丢番图方程
    multipleZ(Aa, F[0]);
    F[0] = -F[0];
    //    print3dMat(F[0]);
    for(int i=1; i<N; ++i)
    {
        for(int i1=0; i1<m; ++i1)
            for(int i2=0; i2<m; ++i2)
            {
                ej.at<double>(i1,i2) = F[i-1].at<double>(i1,i2,0);
                E.at<double>(i1,i2,i) = F[i-1].at<double>(i1,i2,0);
            }
        linearOp(F[i-1], -ej, Aa, F[i]);//Fi = z(Fi-1 - ej*Aa)
        multipleZ(F[i], F[i]);
        //        print3dMat(F[i]);
    }
    //    print3dMat(E);
    for(int i=0; i<N; ++i)
    {
        multipleMat(E, i+1, B, B.size[2], EiB);
        clipMat(EiB, i+1, EiB.size[2], H[i]);
        //        cout<<i<<": "<<endl;        print3dMat(H[i]);
    }
    clipMat(EiB, 0, EiB.size[2]-1, G);
    //    print3dMat(G);

    for(int j=0; j<Nu; ++j)
        for(int i=j; i<N; ++i)
            for(int k=0; k<m; ++k)
                for(int l=0; l<p; ++l)
                    Gm.at<double>(m*i+k, p*j+l) = G.at<double>(k,l,i-j);
    //    cout<<endl<<"Gm: "<<Gm<<endl;    cout<<endl<<"Omega: "<<Omega<<endl;



    //更新 Yk_,计算 Y0
    Yk_.insert(Yk_.begin(), yk.clone());

    for(int i=0; i<N; ++i)
    {
        multipleDelta(H[i], HiDelta);
        for(int j=0; j<m; ++j)
        {
            Y0.at<double>(i*m+j, 0) = 0;
            for(int k=0; k<m; ++k)
                for(int l=0; l<na+1; ++l)
                    Y0.at<double>(i*m+j, 0) += F[i].at<double>(j,k,l)*Yk_[l].at<double>(k);
            for(int k=0; k<p; ++k)
                for(int l=0; l<nb; ++l)
                    Y0.at<double>(i*m+j, 0) += HiDelta.at<double>(j,k,l)*Uk_[l].at<double>(k);
        }
    }

    // invert(Gm.t()*Gm+Omega, inv);
    // duN = inv*Gm.t()*(Yd-Y0);
    for(int i=0; i<Nu; ++i)
        for(int j=0; j<p; ++j)
        {
            duMin.at<double>(Nu*p+i*p+j, 0) = um[j] - u_real[j];
            duMax.at<double>(Nu*p+i*p+j, 0) = uM[j] - u_real[j];
        }
    Mat H = Gm.t()*Gm+Omega;
    Mat b = (Y0-Yd).t()*Gm;
    solveQP(H, b, Ast, duMin, duMax, duN);
    // l<=AΔU<=u
    //        cout<<endl<<"Yd: "<<Yd<<endl<<"Y0: "<<Y0<<endl<<"duN: "<<duN<<endl;
    //        cout<<"duN: "<<duN<<endl;
    uk = Uk_[0] + duN(Range(0, p), Range::all());
    Yk_.pop_back();
    Uk_.pop_back();

    //        cout<<"After popback, Yk_: "<<endl;
    //        for(int i=0; i<Yk_.size(); ++i)
    //            cout<<i<<": "<<Yk_[i]<<endl;
    //        cout<<"Uk_: "<<endl;
    //        for(int i=0; i<Uk_.size(); ++i)
    //            cout<<i<<": "<<Uk_[i]<<endl;

    Uk_.insert(Uk_.begin(), uk.clone());

    //将计算结果传递出去
    u = vector<double>(p, 0);
    for(int i=0; i<p; ++i)
        u[i] = uk.at<double>(i,0);

    Controller::get_u(u);
}
