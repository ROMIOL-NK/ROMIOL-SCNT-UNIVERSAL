#include "utils/custom_algorithms.h"
#include "3rdparty/osqp/osqp.h"
using namespace cv;
using namespace std;

//-----------------------------------------   MHI   -----------------------------------------
MHI::MHI(int Tau, int Ksi, int Delta)
{
    tau=Tau;
    ksi=Ksi;
    delta=Delta;
    isNull = true;
}
bool MHI::run(Mat img, Mat &Himg)//输入格式错误或首次输入图像时，返回 false
{
    int w = img.cols;
    int h = img.rows;
    if(isNull)
    {
        H = Mat::zeros(h,w, CV_8UC1);
        preImg = img.clone();
        isNull = false;
        return false;
    }

    if(w!=preImg.cols || h!=preImg.rows || preImg.type()!=CV_8UC1)
        return false;

    for(int x=0; x<w; ++x)
        for(int y=0; y<h; ++y)
        {
            H.at<uchar>(y,x)=max(0, H.at<uchar>(y,x)-delta);
            if(abs(img.at<uchar>(y,x)-preImg.at<uchar>(y,x))>ksi)
                H.at<uchar>(y,x) = tau;
        }
    preImg = img.clone();
    Himg = H.clone();
    return true;
}


//------------------------------------   Kalman Filter   ------------------------------------
Kalman_Filter::Kalman_Filter()
{
    n=-1;//未初始化
}
void Kalman_Filter::init(Mat x0, Mat P0, Mat F0, Mat H0, Mat Q0, Mat R0)
{
    n = x0.rows;
    //卡尔曼滤波参数
    F   = F0.clone();
    H   = H0.clone();
    Q   = Q0.clone();
    R   = R0.clone();
    xjj = x0.clone();
    Pjj = P0.clone();
    // zk = Mat::zeros(Size(1, n), CV_32F);

    //初始预测
    xkj = F*xjj;
    Pkj = F*Pjj*F.t() + Q;
}
void Kalman_Filter::observe_update(Mat z)
{
    //更新
    zk = z;
    yk = zk-H*xkj;
    Sk = H*Pkj*H.t()+R;
    Kk = Pkj*H.t()*Sk.inv();

    xkk = xkj+Kk*yk;
    Pkk = Pkj-Kk*H*Pkj;
    //迭代
    xjj = xkk;
    Pjj = Pkk;
    //预测
    xkj = F*xjj;
    Pkj = F*Pjj*F.t() + Q;
}

void Plots::linePlot(std::vector<double> x_data, std::vector<double> y_data, QString yLabel, QString xLabel)
{
    int n = min(x_data.size(), y_data.size());
    QChart* chart = new QChart();
    QLineSeries* series = new QLineSeries();
    for(int i=0; i<n; ++i)
        series->append(x_data[i], y_data[i]);
    chart->addSeries(series);
    QChartView *chartView = new QChartView(chart);
    chartView->setRenderHint(QPainter::Antialiasing);//继承来的抗锯齿方法
    chartView->resize(400, 300);
    chartView->show();
}
void Plots::linePlot2Mat(std::vector<double> x_data, std::vector<cv::Mat> y1_data, std::vector<cv::Mat> y2_data, QString yLabel, QString xLabel)
{
    int n = min({x_data.size(), y1_data.size(), y2_data.size()});
    if(n<=0)
        return;
    int ndim = min(y1_data[0].rows, y2_data[0].rows);
    QChart* chart = new QChart();
    QValueAxis *axisX = new QValueAxis;
    QValueAxis *axisY = new QValueAxis;
    axisX->setRange(x_data[0], x_data.back());
    // axisY->setRange(y1_data[0].at<double>(ndim-1,0), y1_data.back().at<double>(ndim-1,0));
    axisY->setRange(-1.5, 5);
    for(int dim=0; dim<ndim; ++dim)
    {
        QLineSeries* series1 = new QLineSeries();
        QLineSeries* series2 = new QLineSeries();
        for(int i=0; i<n; ++i)
            series1->append(x_data[i], y1_data[i].at<double>(dim, 0));
        for(int i=0; i<n; ++i)
            series2->append(x_data[i], y2_data[i].at<double>(dim, 0));
        // series1->attachAxis(axisX);
        // series1->attachAxis(axisY);
        // series2->attachAxis(axisX);
        // series2->attachAxis(axisY);
        chart->addSeries(series1);
        chart->addSeries(series2);
        chart->setAxisX(axisX, series1);
        chart->setAxisY(axisY, series1);
        chart->setAxisX(axisX, series2);
        chart->setAxisY(axisY, series2);
    }
    QChartView *chartView = new QChartView(chart);
    chartView->setRenderHint(QPainter::Antialiasing);//继承来的抗锯齿方法
    chartView->resize(400, 300);
    // chart->addAxis(axisX, Qt::AlignBottom);
    // chart->addAxis(axisY, Qt::AlignLeft);
    chartView->show();
//    int n = min({x_data.size(), y1_data.size(), y2_data.size()});
//    if(n<=0)
//        return;
//    int ndim = min(y1_data[0].rows, y2_data[0].rows);
//    QChart* chart = new QChart();
//    for(int dim=0; dim<ndim; ++dim)
//    {
//        QLineSeries* series1 = new QLineSeries();
//        QLineSeries* series2 = new QLineSeries();
//        for(int i=0; i<n; ++i)
//            series1->append(x_data[i], y1_data[i].at<double>(dim, 0));
//        chart->addSeries(series1);
//        for(int i=0; i<n; ++i)
//            series2->append(x_data[i], y2_data[i].at<double>(dim, 0));
//        chart->addSeries(series2);
//    }
//    QChartView *chartView = new QChartView(chart);
//    chartView->setRenderHint(QPainter::Antialiasing);//继承来的抗锯齿方法
//    chartView->resize(400, 300);
//    chartView->show();
}
void Plots::test(std::vector<Mat>)
{
    qDebug()<<"Test";
}

void linearOp(Mat b, Mat k, Mat x, Mat& result) // result = b+kx; kx 为矩阵乘法
{
    assert(b.dims==3 && x.dims==3 && k.dims==2);//k为 2维矩阵，b和x为 3维，前两个维度尺寸相同，最后一维不必相同；
    assert(b.size[0]==x.size[0] && b.size[0]==k.size[0] && b.size[1]==x.size[1] && b.size[1]==k.size[1]);
    int nb = b.size[2], nx = x.size[2];
    int n = max(nb, nx);
    int size[3] = {b.size[0], b.size[1], n};
    result = Mat(3, size, CV_64F);
    for(int i=0; i<b.size[0]; ++i)
        for(int j=0; j<b.size[1]; ++j)
            for(int l=0; l<n; ++l)
                if(l<nx)
                {
                    double kx = 0;
                    for(int q=0; q<b.size[1]; ++q)
                        kx+=k.at<double>(i,q)*x.at<double>(q,j,l);
                    if(l<nb)
                        result.at<double>(i,j,l) = b.at<double>(i,j,l) + kx;
                    else
                        result.at<double>(i,j,l) = kx;
                }
                else
                    result.at<double>(i,j,l) = b.at<double>(i,j,l);
}
void multipleZ(Mat a, Mat& m)// z*a -> m
{
    assert(a.dims==3);
    int size[3] = {a.size[0], a.size[1], a.size[2]-1};
    m = Mat::zeros(3, size, CV_64F);
    for(int i=0; i<m.size[0]; ++i)
        for(int j=0; j<m.size[1]; ++j)
        {
            //            assert(a.at<double>(i, j, 0)==0);//运算前确保第一个元素为 0
            for(int k=0; k<m.size[2]; ++k)
                m.at<double>(i, j, k) = a.at<double>(i, j, k+1);
        }
}
void multipleDelta(Mat a, Mat& m)// Δ a -> m
{
    assert(a.dims==3);
    int size[3] = {a.size[0], a.size[1], a.size[2]+1};
    m = Mat::zeros(3, size, CV_64F);
    for(int i=0; i<a.size[0]; ++i)
        for(int j=0; j<a.size[1]; ++j)
        {
            m.at<double>(i,j,0) = a.at<double>(i,j,0);
            for(int k=1; k<a.size[2]; ++k)
                m.at<double>(i,j,k) = a.at<double>(i,j,k)-a.at<double>(i,j,k-1);
            m.at<double>(i,j,a.size[2]) = -a.at<double>(i,j,a.size[2]-1);
        }
}
void multipleMat(Mat a, int na, Mat b, int nb, Mat& result)//na 和 nb 为第三个维度的尺寸，计算时忽略后面的数据
{
    //要求a与b维度相为 3，前两个维度符合矩阵乘法的要求
    assert(a.dims==3 && b.dims==3);
    assert(a.size[1]==b.size[0]);
    int size[3] = {a.size[0], b.size[1], na+nb-1};
    result = Mat::zeros(3, size, CV_64F);
    for(int i=0; i<result.size[0]; ++i)
        for(int j=0; j<result.size[1]; ++j)
            for(int k=0; k<result.size[2]; ++k)
                for(int l=0; l<a.size[1]; ++l)
                    for(int q=0; q<=k; ++q)
                        if(q<na && k-q<nb)
                            result.at<double>(i,j,k) += a.at<double>(i,l,q)*b.at<double>(l,j,k-q);
}
void clipMat(Mat a, int ns, int ne, Mat& result)
{
    assert(a.dims==3);
    int size[3] = {a.size[0], a.size[1], ne-ns};
    result = Mat::zeros(3, size, CV_64F);
    for(int i=0; i<result.size[0]; ++i)
        for(int j=0; j<result.size[1]; ++j)
            for(int k=0; k<result.size[2]; ++k)
                result.at<double>(i,j,k) = a.at<double>(i,j,ns+k);
}
void print3dMat(Mat m)
{
    for(int i=0; i<m.size[0]; ++i)
        for(int j=0; j<m.size[1]; ++j)
        {
            for(int k=0; k<m.size[2]; ++k)
                cout<<m.at<double>(i,j,k)<<",";
            cout<<endl;
        }
}
void solveQP(Mat P, Mat Q, Mat A, Mat L, Mat U, Mat& x)// min 1/2 xTPx+QTx, s.t. L<=Ax<=U
{
    assert(P.size[0]==P.size[1] && P.size[0]==Q.size[1] && P.size[0]==A.size[1] && A.size[0]==L.size[0] && A.size[0]==U.size[0]);
    c_int m = A.size[0];
    c_int n = A.size[1];
    c_float* l = new c_float[m];
    c_float* u = new c_float[m];
    c_float* q = new c_float[n];
    x = Mat::zeros(n, 1, CV_64F);
    for(int i=0; i<m; ++i)
    {
        l[i] = L.at<double>(i,0);
        u[i] = U.at<double>(i,0);
    }
    for(int i=0; i<n; ++i)
        q[i] = Q.at<double>(0,i);

    vector<double> indptr, indices, dataV;
    int cnt = 0;
    indptr.push_back(cnt);
    for(int i=0; i<n; ++i)// i 列 j 行
    {
        for(int j=0; j<=i; ++j)
            if(P.at<double>(j,i)!=0)
            {
                cnt+=1;
                indices.push_back(j);
                dataV.push_back(P.at<double>(j,i));
            }
        indptr.push_back(cnt);
    }
    // QP solve example
    // 转化为 csc_matrix 数据格式
    c_int   P_nnz  = cnt;
    c_float* P_x = new c_float[cnt];
    c_int*   P_i = new c_int[cnt];
    c_int*   P_p = new c_int[n+1];
    std::copy(indptr.begin(), indptr.end(), P_p);
    std::copy(indices.begin(), indices.end(), P_i);
    std::copy(dataV.begin(), dataV.end(), P_x);

    cnt = 0;indptr.clear();indices.clear();dataV.clear();
    indptr.push_back(cnt);
    for(int i=0; i<n; ++i)// i 列 j 行
    {
        for(int j=0; j<m; ++j)
            if(A.at<double>(j,i)!=0)
            {
                cnt+=1;
                indices.push_back(j);
                dataV.push_back(A.at<double>(j,i));
            }
        indptr.push_back(cnt);
    }
    c_int   A_nnz  = cnt;
    c_float* A_x = new c_float[cnt];
    c_int*   A_i = new c_int[cnt];
    c_int*   A_p = new c_int[n+1];
    std::copy(indptr.begin(), indptr.end(), A_p);
    std::copy(indices.begin(), indices.end(), A_i);
    std::copy(dataV.begin(), dataV.end(), A_x);
    // Exitflag
    c_int exitflag = 0;
    // Workspace structures
    OSQPWorkspace *work;
    OSQPSettings  *settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
    OSQPData      *data     = (OSQPData *)c_malloc(sizeof(OSQPData));
    // Populate data
    if (data) {
        data->n = n;
        data->m = m;
        data->P = csc_matrix(data->n, data->n, P_nnz, P_x, P_i, P_p);
        data->q = q;
        data->A = csc_matrix(data->m, data->n, A_nnz, A_x, A_i, A_p);
        data->l = l;
        data->u = u;
    }
    // Define solver settings as default
    if (settings) osqp_set_default_settings(settings);
    // Setup workspace
    exitflag = osqp_setup(&work, data, settings);
    // Solve Problem
    osqp_solve(work);
    for(int i=0; i<n; ++i)
        x.at<double>(i,0) = work->solution->x[i];
    // Clean workspace
    osqp_cleanup(work);
    if(data) c_free(data);
    if(settings) c_free(settings);
}
