#ifndef PUBLIC_H
#define PUBLIC_H
/*该文件提供全局变量*/
#include <QCamera>
#include <QChartView>
#include <QDateTime>
#include <QDebug>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QImage>
#include <QKeyEvent>
#include <QLineSeries>
#include <QMediaDevices>
#include <QMessageBox>
#include <QObject>
#include <QProcess>
#include <QPushButton>
#include <QString>
#include <QThread>
#include <QTime>
#include <QTimer>
#include <QTranslator>
#include <QtCharts>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QWheelEvent>
#include <QValueAxis>
#include <QVariant>

#include <QVector>

// float.h, limits.h 中定义了 int, float, double 的最大值
#include <algorithm>
#include <cmath>
#include <direct.h>
#include <float.h>
#include <fstream>
#include <limits.h>
#include <map>
#include <numeric>
#include <string>
#include <vector>
#include <random>

//#define OPENCV_TRAITS_ENABLE_DEPRECATED//traits.hpp 报错
#include <opencv2/core/core.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/video/video.hpp>

//图像的高度和宽度
#define WIDTH  640                    //图像宽度  （单位：像素）
#define HEIGHT 480                    //图像高度  （单位：像素）
//显微镜放大参数
// Basler
// #define Pixel2Um_4x  2.870//4x
// #define Pixel2Um_10x 1.164//10x
// #define Pixel2Um_20x 0.582//20x
// #define Pixel2Um_40x 0.291//40x
// Toup
// #define Pixel2Um_4x  7.212//4x
// #define Pixel2Um_10x 2.899//10x
// #define Pixel2Um_20x 1.455//20x
// #define Pixel2Um_40x 0.727//40x

#ifdef SYSTEM_TJ
    #define Pixel2Um_4x  3.515//4x
    #define Pixel2Um_10x 1.406//10x
    #define Pixel2Um_20x 0.703//20x
    #define Pixel2Um_40x 0.351//40x
#endif

#if (defined SYSTEM_SZ_IPC)||(defined SYSTEM_SZ_STATION)
    #define Pixel2Um_4x  6.700//4x
    #define Pixel2Um_10x 2.680//10x
    #define Pixel2Um_20x 1.340//20x
    #define Pixel2Um_40x 0.670//40x
#endif

#define BiasX_4to10 -583.76//4x → 10x
#define BiasY_4to10 -437.52//4x → 10x
#define BiasZ_4to10 291.05//4x → 10x
#define BiasX_10to20 -187.24//10x → 20x //4x → 20x -771
#define BiasY_10to20 -149.49//10x → 20x //4x → 20x -587.01
#define BiasZ_10to20 37.053//10x → 20x  //4x → 20x 328.1
#define BiasX_20to40 -83.324//20x → 40x //4x → 40x -854.324
#define BiasY_20to40 -73.190//20x → 40x //4x → 40x -660.2
#define BiasZ_20to40 10.441//20x → 40x  //4x → 40x 338.541

//不同镜头下构建地图|运动时的速度
#define PIXEL_VEL 1000
#define STAGE_VEL_AREA 3000                      // 3000 um/s
#define STAGE_VEL PIXEL_VEL*Pixel2Um             // 1000 pixel/s
#define MANIP_VEL 1000             // 1000 pixel/s

#define OOCYTE_DIA  OOCYTE_DIA_UM/Pixel2Um  //卵细胞直径（单位：像素）
#define SOMA_DIA    SOMA_DIA_UM/Pixel2Um    //体细胞直径（单位：像素）
#define HOLD_DIA    HOLD_DIA_UM/Pixel2Um    //吸持针直径（单位：像素）
#define INJ_DIA     INJ_DIA_UM/Pixel2Um     //注射针直径（单位：像素）
#define WIDTH_UM    WIDTH*Pixel2Um          //图像宽度  （单位：um）
#define HEIGHT_UM   HEIGHT*Pixel2Um         //图像高度  （单位：um）


//控件样式
#define NORMAL_QSS "QPushButton{\
    border: 1px solid gray;\
    border-radius: 10px;\
    background-color: rgb(225, 225, 225);\
    font-family: 华文楷体;\
    font-size: 12pt;\
    color: black;\
    font: bold;}"
#define RUNNING_QSS "QPushButton{\
    border: 1px solid #3a8ee6;\
    border-radius: 10px;\
    background-color: rgba(0, 255, 50, 0.8);\
    font-family: 华文楷体;\
    font-size: 12pt;\
    color: black;\
    font: bold;}"
#define FINISHED_QSS "QPushButton{\
    border: 1px solid black;\
    border-radius: 10px;\
    background-color: rgba(50, 50, 50, 0.5);\
    font-family: 华文楷体;\
    font-size: 12pt;\
    color: black;\
    font: bold;}"
#define READY_QSS "QPushButton{\
    border: 1px solid #3a8ee6;\
    border-radius: 10px;\
    background-color: rgba(255, 255, 0, 0.8);\
    font-family: 华文楷体;\
    font-size: 12pt;\
    color: black;\
    font: bold;}"
//硬件端口编号
extern QString strPortStage;    //平台
extern QString strPortFocus;    //调焦设备
extern QString strPortLManip;   //左操作臂
extern QString strPortRManip;   //右操作臂 COM13
extern QString strPortPHold;    //吸持针压力传感器
extern QString strPortPInj;     //注射针压力传感器
extern QString strPortLMotor;   //左注射器电机 8 9
extern QString strPortRMotor;   //右注射器电机 COM2气泵、 COM13 油泵
// //-----------------深圳系统 IPC start-----------------------
// #ifdef SYSTEM_SZ_IPC
//     #define strPortStage   "192.168.0.250"  //平台
//     #define strPortFocus   "COM10"           //调焦设备
//     #define strPortLManip  "COM5"           //左操作臂
//     #define strPortRManip  "COM16"           //右操作臂 COM13
//     #define strPortPHold   "COM11"          //吸持针压力传感器
//     #define strPortPInj    "COM12"          //注射针压力传感器
//     #define strPortLMotor  "COM9"           //左注射器电机 8 9
//     #define strPortRMotor  "COM13"           //右注射器电机 COM2气泵、 COM13 油泵
// #endif
// //-----------------深圳系统 IPC end------------------------

// //-----------------深圳系统 Station start-----------------------
// #ifdef SYSTEM_SZ_STATION
//     #define strPortStage   "192.168.0.250"  //平台
//     #define strPortFocus   "COM8"           //调焦设备
//     #define strPortLManip  "COM1"           //左操作臂
//     #define strPortRManip  "COM4"           //右操作臂 COM13
//     #define strPortPHold   "COM11"          //吸持针压力传感器
//     #define strPortPInj    "COM12"          //注射针压力传感器
//     #define strPortLMotor  "COM9"           //左注射器电机
//     #define strPortRMotor  "COM7"           //右注射器电机 COM2气泵、 COM 油泵
// #endif
// //-----------------深圳系统 Station end------------------------

// //-----------------南开系统 start-----------------------
// #ifdef SYSTEM_TJ
//     #define strPortStage   "COM11"          //南开平台
//     #define strPortFocus   "COM8"           //调焦设备
//     #define strPortLManip  "COM20"          //左操作臂
//     #define strPortRManip  "COM21"          //右操作臂
//     #define strPortPHold   "COM13"          //吸持针压力传感器
//     #define strPortPInj    "COM12"          //注射针压力传感器
//     #define strPortLMotor  "COM14"          //左注射器电机
//     #define strPortRMotor  "COM15"          //右注射器电机
// #endif
// //-----------------南开系统 end------------------------

//运动控制按钮显示的图标
#define strStateUp         "▲"// ↑
#define strStateDown       "▼"// ↓
#define strStateForward    "▲"// ↑
#define strStateBack       "▼"// ↓
#define strStateLeft       "◀"// ←
#define strStateRight      "▶"// →
#define strStatePause      "⏸"
/*----------------------------------------枚举类型----------------------------------------*/
enum MODE{SCNT_SINGLE, SCNT_BATCH, ICSI};//操作模式：体细胞核移植（单个细胞循环）、批量化体细胞核移植、胞浆内精子注射
enum Objective{x4, x10, x20, x40};
enum ImgSource{SOURCE_CAMERA, SOURCE_IMG, SOURCE_VIDEO};
enum WORKSTEP{ZEROPOS, BUILDMAP, FETCHOOCYTE, ROTATEOUTPLANE, ROTATEINPLANE, PENETRATE, ENUCLEATION, EXPEL, FETCHSOMA, FETCHOOCYTEFORSOMA, ROTATEFORSOMA, PENETRATEFORSOMA, INJECTSOMA, RELEASEOOCYTE, EXCEPTION, END};

/*----------------------------------------全局变量----------------------------------------*/
extern float petriDishHeight;   //将微针抬升到安全位置的高度
extern float areaMoveHeight;    //将微针从培养皿底部抬升到不会触碰细胞的高度
extern float focusHigh;         //调焦设备对准操作高度时的位置
extern float focusBottom;       //调焦设备对准培养皿底面时的位置
extern float injHigh;           //注射针对准操作高度时的位置
extern float injBottom;         //注射针对准培养皿底面时的位置
extern float tarHoldP;
extern float tarInjP;
extern float holdPressure;      //所用吸持针可以吸持住细胞的气压
extern float holdBalanceP;      //所用吸持针 气柱刚好到管口的气压
extern float holdRotateP;      //所用吸持针 旋转卵细胞时的气压
extern float injReleaseP;       //所用注射针 可以释放胞质的大致压强
extern float injBalanceP;       //所用注射针 气柱刚好到管口附近的气压
extern float injBiasP;
extern float injExtractSafeP;   //所用注射针 气柱刚好到管口附近的气压
//针尖、细胞的估计尺寸
extern int OOCYTE_DIA_UM;       //卵细胞的大致直径（单位：um）
extern int SOMA_DIA_UM;         //体细胞的大致直径（单位：um）
extern int HOLD_DIA_UM;         //吸持针的大致直径（单位：um）
extern int INJ_DIA_UM;          //注射针的大致直径（单位：um）
extern int ZP_WIDTH;            //透明带的大致宽度（单位：um）

//当前图像、全局地图的位置和尺寸
extern int GMAP_WIDTH;          //全局地图的宽度（单位：像素） GMAP_WIDTH  = GMAP_WIDTH_UM /Pixel2Um_x4
extern int GMAP_HEIGHT;         //全局地图的高度（单位：像素） GMAP_HEIGHT = GMAP_HEIGHT_UM/Pixel2Um_x4
extern int GTHUMB_WIDTH;        //全局缩略图宽度（单位：像素）
extern int GTHUMB_HEIGHT;       //全局缩略图高度（单位：像素）
extern int REGION_THUMB_OX;     //当前图像（左上角）在全局地图（左上角）中的 X位置（单位：像素）
extern int REGION_THUMB_OY;     //当前图像（左上角）在全局地图（左上角）中的 Y位置（单位：像素）
extern int REGION_THUMB_WIDTH;  //当前图像在全局缩略图中的宽度 （单位：像素）
extern int REGION_THUMB_HEIGHT; //当前图像在全局缩略图中的高度 （单位：像素）
extern int MOUSE_PRESS_REGION_X;//鼠标点击的再当前缩略图中的相对 X位置
extern int MOUSE_PRESS_REGION_Y;//鼠标点击的再当前缩略图中的相对 Y位置
extern float GMAP_WIDTH_UM;     //全局地图的宽度（单位：um）
extern float GMAP_HEIGHT_UM;    //全局地图的高度（单位：um）
extern float REGION_OX_UM;      //当前图像（左上角）在全局地图（左上角）中的 X位置（单位：um）
extern float REGION_OY_UM;      //当前图像（左上角）在全局地图（左上角）中的 Y位置（单位：um）

//相机参数或配置
extern float IMG_STAGE_ANGLE;   //显微图像偏转的角度
extern float Pixel2Um;          //当前镜头下像素的尺寸 (um)
extern QString exceptionInfo;   //异常信息
extern Objective CurObj;        //当前的镜头（放大倍率）

//辅助信息存储变量
extern MODE     mode;
extern WORKSTEP nextStep;                                       //等待完成的步骤
extern QString FILE_PATH;                                       //图像、视频、文件的保存路径
extern std::vector<cv::Point> selectPoints;                     //人工辅助标记点
extern std::map<std::string, std::vector<std::string>> trajMaps;//控制器期望轨迹不同模式涉及的参数
//（预先声明 Public 中用到的类，需要使用指针，因为
//Forward declaration will only help you specify pointer or reference types, not value types.）
class Yolo;
class Vision;
class Track;
class PServo;
class ObjectDetector;

class Microscope;
class Holder;
class Injector;
class GlobalMap;
class HoldPipette;
class InjectPipette;
class Oocyte;
class Soma;
class Oocytes;
class Somas;
class AutoClone;
class StepBuildMap;
class StepFetchOocyte;
class StepRotateOutPlane;
class StepRotateInPlane;
class StepPenetrate;
class StepEnucleation;
class StepExpel;
class StepFetchSoma;
class StepFetchOocyteForSoma;
class StepRotateForSoma;
class StepPenetrateForSoma;
class StepInjectSoma;
class StepReleaseOocyte;
class StepException;
class Functions;
class Plots;
//关注对象
extern cv::Point2f        donorArea;//供体细胞区域位置
extern cv::Point2f         workArea;//操作区域位置
extern cv::Point2f         spitArea;//吐核区域位置
extern cv::Point2f         spitEdge;//吐核边缘位置
extern cv::Point2f        storeArea;//存储区域位置
extern cv::Point2f       oocyteArea;//卵细胞区域位置
extern cv::Point2f         failArea;//失败后存储位置
extern cv::Mat          CurrentGray;//当前图像（灰度图），在 camera 更新图像时更新
extern cv::Mat           CurrentBGR;//当前图像（彩色图），在 camera 更新图像时更新
extern cv::Mat              PreGray;//上一帧图像（灰度图）
extern cv::Mat                Image;//当前 ui 中显示的图像
extern Microscope*       microscope;//显微镜
extern Holder*               holder;//吸持器（包括操作臂、活塞注射器电机、气压传感器）
extern Injector*           injector;//注射器（包括操作臂、活塞注射器电机、气压传感器）
extern GlobalMap*         globalMap;//全局地图
extern HoldPipette*     holdPipette;//吸持针（图像信息）
extern InjectPipette* injectPipette;//注射针（图像信息）
extern Oocyte*        currentOocyte;//当前卵细胞（图像信息）
extern Soma*            currentSoma;//当前体细胞（图像信息）
extern Oocytes*          allOocytes;//全局地图中所有卵细胞（图像信息）
extern Oocytes*       oocytesInView;//当前视野中的所有卵细胞
extern Somas*              allSomas;//所有体细胞（图像信息）
extern AutoClone*               autoClone;
extern StepBuildMap*            stepBuildMap;
extern StepFetchOocyte*         stepFetchOocyte;
extern StepRotateOutPlane*      stepRotateOutPlane;
extern StepRotateInPlane*       stepRotateInPlane;
extern StepPenetrate*           stepPenetrate;
extern StepEnucleation*         stepEnucleation;
extern StepExpel*               stepExpel;
extern StepFetchSoma*           stepFetchSoma;
extern StepFetchOocyteForSoma*  stepFetchOocyteForSoma;
extern StepRotateForSoma*       stepRotateForSoma;
extern StepPenetrateForSoma*    stepPenetrateForSoma;
extern StepInjectSoma*          stepInjectSoma;
extern StepReleaseOocyte*       stepReleaseOocyte;
extern StepException*           stepException;
extern Functions*            functions;
extern Plots*                plots;
extern Yolo*            yolo;               //Yolo 目标检测网络
extern Yolo*            yoloPB;             //Yolo 目标检测网络
extern Vision*          vision;             //物体检测模块
extern PServo*          pServo;             //气压伺服
extern ObjectDetector*  objectDetector;     //物体检测模块
extern Track*           track;              //视觉跟踪模块
extern bool             areAllOocytesEnucleated;
extern bool             areAllSomasFetched;
extern bool             areAllSomasInjected;
extern bool             isWorking;          //执行工作中的标志，防止多项工作冲突；在进入 run函数前要确保没有其他进行中的工作
extern bool             isUsingSocket;      //物体检测是否使用 socket通讯
extern bool             isHCI;              //是否使用人机交互模式 （部分workflow完全交给操作员完成）
#endif // PUBLIC_H
