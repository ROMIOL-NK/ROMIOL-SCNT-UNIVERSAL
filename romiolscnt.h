/* ----------------------------------------   ROMIOL-SCNT  ----------------------------------------
 * 该程序进行了3个层次的抽象：设备层(Devices)-部件层(Components)-任务层(Tasks)
 * 硬件的通讯控制在设备层(Devices)完成，高级功能在任务层(Tasks)完成；这两部分也是该程序的关键之处。
 * 部件层(Components)起到中间连接、整合的作用，使得两部分的代码可以单独工作。
 * 任务层可以调用硬件资源，程序结束时销毁硬件相关的指针。
 * 1. 设备层(Devices)
 * 该文件夹下是具体的硬件接口，会涉及到不同类型的显微镜、微操作手臂、注射器电机、运动平台、电动调焦设备等；
 * 可以包括各种通讯接口：RS232、USB、CAN、EtherCAT等；在厂家提供的手册基础上，要实现相应的开机检测、控制指令发送、返回信号处理等功能。
 * 2. 部件层(Components)
 * 不关心具体硬件由哪个厂家生产，只要求能够实现点动和连动的基本控制指令[，能够返回各自的当前状态 OPTIONAL]。
 * 电机位置的显示以 um 为单位，方向有固定的标准：XY 正方向与 opencv图像一致，Z 轴以靠近镜头为正方向
 * 3. 任务层(Tasks)
 * 每一个任务处在单独的线程中，各自也是单独的类，类成员包括执行任务需要调用的硬件资源（Component 指针）；
 * 相应按钮点击时触发Task的run() 槽函数，同时ui界面有相应的提示和响应；如果要顺序执行多个任务，可以新建多个任务变量，[加入到同一个子线程中，]顺序执行 run
 *
 * public.h中包含了需要的头文件、全局静态变量
 * components.h, objects.h 是各个层次下头文件的打包
 *
 */

#ifndef ROMIOLSCNT_H
#define ROMIOLSCNT_H

#include <QMainWindow>

#include "public.h"
#include "childwindows.h"
#include "Components/components.h"
#include "Objects/objects.h"
#include "utils/objectdetector.h"
#include "utils/neuralnetwork.h"
#include "WorkFlow/vision.h"
#include "WorkFlow/track.h"
#include "WorkFlow/pservo.h"
#include "WorkFlow/autofocus.h"
#include "WorkFlow/autoclone.h"
#include "WorkFlow/functions.h"


QT_BEGIN_NAMESPACE
namespace Ui { class RomiolSCNT; }
QT_END_NAMESPACE

class RomiolSCNT : public QMainWindow
{
    Q_OBJECT
public:
    RomiolSCNT(QWidget *parent = nullptr);
    ~RomiolSCNT();
private:
    Ui::RomiolSCNT *ui;

    QPointer<QWidget> popup_;  // 自动感知窗口销毁
    void togglePopup();

    QLabel* pixelStatus;        //像素信息状态栏标签
    QLabel* workFlowStatus;     //工作流信息状态栏标签
    QLabel* motorStatus;        //电机运动信息状态栏标签
    //-------------------- 子线程 --------------------
    QThread threadLManip;       //左操作臂
    QThread threadLPSensor;     //左气压传感器
    QThread threadLSMotor;      //左注射器电机
    QThread threadRManip;       //右操作臂
    QThread threadRPSensor;     //右气压传感器
    QThread threadRSMotor;      //右注射器电机
    QThread threadStage;        //电动平台
    QThread threadCamera;       //相机
    QThread threadFocus;        //调焦设备

    QThread threadVision;       //视觉检测模块
    QThread threadTrack;        //视觉跟踪模块
    QThread threadPServo;       //气压伺服模块

    QThread threadFunctions;
    QThread threadWorkFlow;

    //-------------------- 子窗口 --------------------
    HumanAssistance     humanAssistance;
    //------------------- 辅助成员 -------------------

    //记忆内部状态的变量
    QPointF             mouseClickPos;
    QStringList         imgSourcePath;
    cv::VideoCapture    videoCapSource;
    bool                isRecording;    //记录数据中的标志
    bool                isPlaying;      //播放视频中
    int                 recordDT;       //记录数据的时间间隔

    std::ofstream       recordfile;
    QString             datasetPath;

    std::vector<QPushButton*> scnt_single_group;
    std::vector<QPushButton*> scnt_batch_group;
    std::vector<QPushButton*> icsi_group;

    //定时器
    QTimer* thumbnailTimer;
    QTimer* playSourceTimer;
    QTimer* recordTimer;
signals:
    // 触发任务线程开始执行的信号: connect(this, SIGNAL(sgn[Task]()), [Task], SLOT(run()));
    void sgnBuildMap();
    void sgnStepBuildMap();
    void sgnStepFetchOocyte();
    void sgnStepRotateOutPlane();
    void sgnStepRotateInPlane();
    void sgnStepPenetrate();
    void sgnStepEnucleation();
    void sgnStepExpel();
    void sgnStepFetchSoma();
    void sgnStepFetchOocyteForSoma();
    void sgnStepRotateForSoma();
    void sgnStepPenetrateForSoma();

    void sgnStepInjectSoma();
    void sgnStepReleaseOocyte();
    void sgnAutoClone(WORKSTEP);
    void sgnFunction(QString);

    void sgnConnectSocket();
    void sgnCloseSocket();

    void sgnConnectStage(QString);
    void sgnConnectCamera(int);
    void sgnConnectFocus(QString);
    void sgnConnectLManip(QString);
    void sgnConnectLPSensor(QString);
    void sgnConnectLSMotor(QString);
    void sgnConnectRManip(QString);
    void sgnConnectRPSensor(QString);
    void sgnConnectRSMotor(QString);

    void sgnGotoArea(cv::Point2f);
    void sgnMoveStageX(float, bool, float);
    void sgnMoveStageY(float, bool, float);
    void sgnCaptureFrame(QString);
    void sgnStartCameraRecord(QString);
    void sgnEndCameraRecord();
    void sgnPauseCameraRecord();
    void sgnContinueCameraRecord();
    void sgnMoveFocusZ(float, bool, float);
    void sgnMoveLManipX(float, float, bool);
    void sgnMoveLManipY(float, float, bool);
    void sgnMoveLManipZ(float, float, bool);
    void sgnLSMotorSuck(float, float, bool);
    void sgnLSMotorBlow(float, float, bool);
    void sgnMoveRManipX(float, float, bool);
    void sgnMoveRManipY(float, float, bool);
    void sgnMoveRManipZ(float, float, bool);
    void sgnRSMotorSuck(float, float, bool);
    void sgnRSMotorBlow(float, float, bool);

    void sgnCloseStage();
    void sgnCloseCamera();
    void sgnCloseFocus();
    void sgnCloseLManip();
    void sgnCloseLPSensor();
    void sgnCloseLSMotor();
    void sgnCloseRManip();
    void sgnCloseRPSensor();
    void sgnCloseRSMotor();

    void sgnStopFocus();
    void sgnStopPServo();
    void sgnStopStage();
    void sgnStopLManip();
    void sgnStopLSMotor();
    void sgnStopRManip();
    void sgnStopRSMotor();

private slots:
    //响应按钮点击的槽函数，运动的参数都通过界面获得
    void init();                //初始化：变量、文件夹、定时器
    void initDevices();         //初始化所有设备
    void modeChanged(QString);  //操作模式发生变化
    void btnReset();            //终止所有运动,恢复到开机状态 TODO
    void stopAllMotors();       //终止所有电机的运动
    // **************************************** 设备操控 ****************************************
    void connectCamera();           //连接相机
    void connectFocusDevice();      //连接调焦设备
    void connectStage();            //连接平台
    void connectLManip();           //连接左操作臂
    void connectRManip();           //连接右操作臂
    void connectHoldPSensor();      //连接吸持针气压传感器
    void connectInjPSensor();       //连接注射针气压传感器
    void connectHolderMotor();      //连接右侧注射器电机
    void connectInjectorMotor();    //连接右侧注射器电机

    // -------------------- 显微镜 --------------------
    void btnScopeUp();      //视野 上移
    void btnScopeDown();    //视野 下移
    void btnScopeLeft();    //视野 左移
    void btnScopeRight();   //视野 右移
    void btnScopeForward(); //视野 前移（远离操作员）
    void btnScopeBack();    //视野 后移（靠近操作员）
    //调焦设备
    void focusStartConnection();//调焦设备建立连接
    void focusNoConnection();   //调焦设备无法连接
    void focusOpenPortFailure();//无法打开相应端口
    void scopeUpdateZ();        //定时器驱动下，更新调焦位置
    //平台
    void stageStartConnection();//平台建立连接
    void stageNoConnection();   //平台无法连接
    void stageOpenPortFailure();//无法打开端口
    void scopeUpdateXY();       //定时器驱动下，更新平台位置
    //相机
    void cameraStartConnection();   //相机建立连接
    void cameraNoConnection();      //相机无法连接
    void scopeUpdateImage();        //定时器驱动下，更新图像
    void btnCapture();              //抓拍图像
    void btnVideo();                //录制视频
    void btnPauseRecord();          //暂停录制
    void btnSelPath();              //选择保存路径
    void btnSelImgSource();         //选择图像源
    void btnPlaySource();           //播放视频源：视频/图像序列
    void imgSourceChanged(QString); //图像源发生变化

    // --------------------吸持装置--------------------
    //左操作臂
    void btnLManipUp();         //上
    void btnLManipDown();       //下
    void btnLManipLeft();       //左
    void btnLManipRight();      //右
    void btnLManipForward();    //前（远离操作员）
    void btnLManipBack();       //后（靠近操作员）
    void lManipOpenPortFailure();   //无法打开相应端口
    void lManipNoConnection();      //左操作臂无法连接
    void lManipStartConnection();   //左操作臂建立连接
    void lManipUpdatePos();         //更新左操作臂位置
    //气压传感器
    void lPSensorOpenPortFailure(); //左侧气压传感器无法打开端口
    void lPSensorNoConnection();    //左侧气压传感器无法连接
    void lPSensorStartConnection(); //左侧气压传感器建立连接
    void lPSensorupdatePressure();  //左侧气压传感器更新气压
    //左注射器电机
    void holderMotorStartConnection();  //吸持电机建立连接
    void btnHolderSuck();               //吸持电机吸取
    void btnHolderBlow();               //吸持电机注入
    void holderUpdatePos();             //吸持电机更新位置
    void lSyringeMotorNoConnection();   //吸持电机失去连接
    // --------------------注射装置--------------------
    //右操作臂
    void btnRManipUp();         //上
    void btnRManipDown();       //下
    void btnRManipLeft();       //左
    void btnRManipRight();      //右
    void btnRManipForward();    //前（远离操作员）
    void btnRManipBack();       //后（靠近操作员）
    void rManipOpenPortFailure();   //无法打开相应端口
    void rManipNoConnection();      //右操作臂无法连接
    void rManipStartConnection();   //右操作臂建立连接
    void rManipUpdatePos();         //更新右操作臂位置
    //气压传感器
    void rPSensorOpenPortFailure(); //右侧气压传感器无法打开端口
    void rPSensorNoConnection();    //右侧气压传感器无法连接
    void rPSensorStartConnection(); //右侧气压传感器建立连接
    void rPSensorupdatePressure();  //右侧气压传感器更新气压
    //右注射器电机
    void injectorMotorStartConnection();//注射电机建立连接
    void btnInjectorSuck();             //注射电机吸取
    void btnInjectorBlow();             //注射电机注入
    void injectorUpdatePos();           //注射电机更新位置
    void rSyringeMotorNoConnection();   //注射电机失去连接

    // ************************************* 工作流（多线程） ************************************
    //btn[Task]: 初始化任务线程，发送信号 sgn[Task]调用任务处理函数 run()；完成后执行 finish[Task]，销毁任务对象
    void btnRestart();                      //重新开始自动化克隆流程
    void btnStepBuildMap();                 //运行步骤 构建地图
    void btnStepFetchOocyte();              //运行步骤 取卵
    void btnStepRotateOutPlane();           //运行步骤 旋转卵细胞
    void btnStepRotateInPlane();            //运行步骤 平面内旋转
    void btnStepPenetrate();                //运行步骤 刺入
    void btnStepEnucleation();              //运行步骤 抽核
    void btnStepExpel();                    //运行步骤 吐核
    void btnStepFetchSoma();                //运行步骤 取体细胞

    void btnStepFetchOocyteForSoma();       //运行步骤 取卵细胞，为体细胞注射做准备
    void btnStepRotateForSoma();            //运行步骤 旋转卵细胞，为体细胞注射做准备
    void btnStepPenetrateForSoma();         //运行步骤 刺入卵细胞，为体细胞注射做准备

    void btnStepInjectSoma();               //运行步骤 注射体细胞
    void btnStepReleaseOocyte();            //运行步骤 释放卵细胞
    void btnAutoClone();                    //运行自动化克隆流程
    void dealFault(QString);            //处理工作流中出现的故障
    void finishStep();                  //完成某项步骤后显示下一步
    void finishStepBuildMap();          //步骤构建地图结束后处理：更新步骤记录 nextStep，ui中提示
    void finishStepFetchOocyte();       //步骤取卵结束后处理：更新步骤记录 nextStep，ui中提示
    void finishStepRotateOutPlane();    //步骤旋转卵细胞结束后处理：更新步骤记录 nextStep，ui中提示
    void finishStepRotateInPlane();     //步骤平面内旋转结束后处理：更新步骤记录 nextStep，ui中提示
    void finishStepPenetrate();         //步骤刺入结束后处理：更新步骤记录 nextStep，ui中提示
    void finishStepEnucleation();       //步骤抽核结束后处理：更新步骤记录 nextStep，ui中提示
    void finishStepExpel();             //步骤吐核结束后处理：更新步骤记录 nextStep，ui中提示
    void finishStepFetchSoma();         //步骤取体细胞结束后处理：更新步骤记录 nextStep，ui中提示

    void finishStepFetchOocyteForSoma();//步骤取卵细胞结束后处理：更新步骤记录 nextStep，ui中提示
    void finishStepRotateForSoma();     //步骤取旋转卵细胞结束后处理：更新步骤记录 nextStep，ui中提示
    void finishStepPenetrateForSoma();  //步骤取刺入卵细胞结束后处理：更新步骤记录 nextStep，ui中提示

    void finishStepInjectSoma();        //步骤注射体细胞结束后处理：更新步骤记录 nextStep，ui中提示
    void finishStepReleaseOocyte();     //步骤释放卵细胞结束后处理：更新步骤记录 nextStep，ui中提示
    void finishAutoClone();             //自动化克隆流程结束后处理，ui中提示
    void finishFunc(QString);           //完成 functions 任务

    void cfgControlParas(Controller* &);//弹窗显示、修改控制器的参数
    void cfgTrajParas(Controller* &);   //弹窗显示、修改控制器的轨迹参数
    // *************************************自定义功能（多线程）***********************************
    void btnObjectDetector();

    // **************************************** 辅助功能 ****************************************
    void btnPosType();          //改变位置显示格式：绝对/相对
    void btnSetOrigin();        //设置当前位置为坐标原点
    void btnColor();            //图像显示颜色切换 彩色/灰色
    void btnShowAll();          //显示所有物体 开|关

    void recordData();          //记录数据

    void updateThumbnail();     //更新显示全局缩略图
    void menuHumanAssistance();

private:
    //键盘响应函数
    virtual bool event(QEvent* event);
    virtual void mouseMoveEvent(QMouseEvent* event);
};
#endif // ROMIOLSCNT_H
