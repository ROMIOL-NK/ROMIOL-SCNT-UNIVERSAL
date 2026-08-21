#ifndef CUSTOM_ALGORITHMS_H
#define CUSTOM_ALGORITHMS_H
/*自己实现的一些算法，包括：
 * 1. MHI 运动历史图像 H_τ(x,y,t) = τ, if |I(x,y,t)-I(x,y,t-1)| > ξ; else max(0, H_τ(x,y,t-1)-δ).
 * 2. Kalman Filter
*/

#include "public.h"

class MHI
{
public:
    MHI(){tau=200;ksi=20;delta=50;}
    MHI(int Tau, int Ksi, int Delta);
    bool run(cv::Mat img, cv::Mat &Himg);
    void reset(){isNull = true;preImg.release();}

    bool isNull;            //未启用的标识符
    int tau;                //运动记录持续时间
    int ksi;                //运动像素差异阈值
    int delta;              //运动记录衰减速率
    cv::Mat H;              //运动历史记录强度
    cv::Mat preImg;         //上一帧图像
};

class Kalman_Filter
{
public:
    Kalman_Filter();
    void init(cv::Mat x0, cv::Mat P0, cv::Mat F, cv::Mat H, cv::Mat Q, cv::Mat R);
    void observe_update(cv::Mat z);

private:
    int n;  //状态维度
    cv::Mat F, H, Q, R, xkj, xjj, Pkj, Pjj, yk, zk, Sk, Kk, xkk, Pkk;   //卡尔曼滤波参数、数据
};
class Plots: public QObject
{
    Q_OBJECT
public:
    Plots(){}
    ~Plots(){}
public slots:
    void linePlot(std::vector<double> x_data, std::vector<double> y_data, QString yLabel="", QString xLabel="");
    void linePlot2Mat(std::vector<double> x_data, std::vector<cv::Mat> y1_data, std::vector<cv::Mat> y2_data, QString yLabel="", QString xLabel="");
    void test(std::vector<cv::Mat>);
};


void linearOp(cv::Mat b, cv::Mat k, cv::Mat x, cv::Mat& result); // result = b+kx; kx 为矩阵乘法
void multipleZ(cv::Mat a, cv::Mat& m);// z*a -> m
void multipleDelta(cv::Mat a, cv::Mat& m);// Δ a -> m
void multipleMat(cv::Mat a, int na, cv::Mat b, int nb, cv::Mat& result);//na 和 nb 为第三个维度的尺寸，计算时忽略后面的数据
void clipMat(cv::Mat a, int ns, int ne, cv::Mat& result);
void print3dMat(cv::Mat m);
void solveQP(cv::Mat P, cv::Mat Q, cv::Mat A, cv::Mat L, cv::Mat U, cv::Mat& x);// min 1/2 xTPx+QTx, s.t. L<=Ax<=U

#endif // CUSTOM_ALGORITHMS_H
