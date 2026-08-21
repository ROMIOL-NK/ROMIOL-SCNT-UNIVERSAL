#ifndef CONTROLLER_H
#define CONTROLLER_H
#include "public.h"
#undef slots
#include <Python.h>
#define slots Q_SLOTS
enum TRAJ_MODE{TRAJ_MODE_SIGMOID, TRAJ_MODE_ACC_DEC, TRAJ_MODE_SINE, TRAJ_MODE_SETPOINT, TRAJ_MODE_SLOPE};
//通用的控制器类，使用方式：setTrajParas[, setParas], initialize -> (loop) {get_yd(), (observe) set_y(_), get_u(&u), step()}，其中 y由外部观测器获得，计算得到 u 后提供给执行器
class Controller: public QObject
{
public:
    Controller(int n_in, int n_out, int n_observe=0);
    void setTrajParas(std::vector<std::map<std::string, double>>);  //设置目标轨迹参数
    virtual void saveParas();                                       //保存控制器参数
    virtual void setParas(std::map<std::string, QVariant>);         //设置控制器参数
    virtual void initialize();                                      //初始化控制器参数、相关变量；保存控制器参数到 qParas 中
    virtual void get_yd(double t=-1);                               //根据当前时刻，轨迹参数paras，获取系统参考值   r，维数 m;
    virtual void set_y(std::vector<double> y_observe);              //获取系统输出值 y_o，维数 m
    virtual void set_z(std::vector<double> z_observe);              //获取系统观测值 z_o，维数 nz
    virtual void get_u(std::vector<double>&u);                      //获取控制器输出   u，维数 p
    void step();                                                    //迭代到下一步
    double getDT(){return delta_t;}                                 //返回控制步长
    void drawTrajectory();                                          //绘制轨迹曲线
    void drawU();                                                   //绘制输入曲线
    void set_yd(std::vector<double> yd_);                           //设置当前时刻的期望值;
    void loadParasFromJson(QString path, QString key);              //从json文件导入控制器参数
    void save_data(std::string filename);                           //保存数据

    std::vector<std::map<std::string, double>> trajParas;           //多个（m个）目标轨迹的参数字典
    std::map<std::string, QVariant> qParas;                         //控制器参数

    QString name;                       //控制器的名称
    QString U_name;                     //控制量的名称，如 speed, pos... 决定 u 如何施加到执行器上
    int p, m, nz;                       //控制器的输入输出、观测维数
    int k_step;                         //当前迭代步数，每次计算完 u (get_u) 之后 k_step++
    double delta_t;                     //计算步长（单位：s）
    qint64 t0;                          //初始时刻
    std::vector<qint64> mtime;          //时间列表
    std::vector<double> uM;             //输入最大限幅
    std::vector<double> um;             //输入最小限幅
    std::vector<double> iuM;			//输入积分最大阈值
    std::vector<double> ium;			//输入积分最小阈值
    std::vector<double> setPoint;       //设定点
    std::vector<double> yd;             //当前输出的目标值
    std::vector<double> y;              //当前输出的观测值（控制目标）
    std::vector<double> z;              //当前的观测变量（参考信息）
    std::vector<double> y_bias;         //系统输出的偏移量设置 有些情况希望初始值为 0，有些情况希望目标值为 0
    std::vector<double> u_idea;         //当前输入的理想值
    std::vector<double> u_real;         //当前输入的实际值

    std::vector<std::vector<double>> yd_history;            //输出的目标值
    std::vector<std::vector<double>> y_history;             //输出的观测值
    std::vector<std::vector<double>> z_history;             //系统观测值
    std::vector<std::vector<double>> u_idea_history;        //输入的理想值
    std::vector<std::vector<double>> u_real_history;        //输入的实际值
    double y_th;                             //分阶段的阈值
    double u1;                               //第一阶段(y_observe[0]-y_history[0][0]-y_bias[0]<y_th)的控制输入
    bool isStage2;                           //进入第二阶段的标志；y_observe[0]-y_history[0][0]-y_bias[0]<y_th时恒定控制输入为u1
    bool isSaveLastPara;                     //是否保持上次辨识的参数
    bool isInitialized;                      //是否已完成初始化

    PyObject *pModule;
    PyObject * pFuncDrawTraj;
    PyObject * pFuncDrawU;
};

#endif // CONTROLLER_H
