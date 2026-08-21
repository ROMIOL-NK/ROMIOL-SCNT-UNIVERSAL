#include"public.h"

//----------------------------------------------------------------- 声明全局变量 (初始化） -----------------------------------------------------------------
//-----------------深圳系统 IPC start-----------------------
#ifdef SYSTEM_SZ_IPC
    QString strPortStage  = "192.168.0.250";
    QString strPortFocus  = "COM10";
    QString strPortLManip = "COM5";
    QString strPortRManip = "COM16";
    QString strPortPHold  = "COM11";
    QString strPortPInj   = "COM12";
    QString strPortLMotor = "COM9";
    QString strPortRMotor = "COM8";//wzf 油泵 COM13; 气泵：COM8
#endif
//-----------------深圳系统 IPC end------------------------

//-----------------深圳系统 Station start-----------------------
#ifdef SYSTEM_SZ_STATION
    QString strPortStage  = "192.168.0.250";
    QString strPortFocus  = "COM8";
    QString strPortLManip = "COM1";
    QString strPortRManip = "COM4";
    QString strPortPHold  = "COM11";
    QString strPortPInj   = "COM12";
    QString strPortLMotor = "COM9";
    QString strPortRMotor = "COM7";
#endif
//-----------------深圳系统 Station end------------------------

//-----------------南开系统 start-----------------------
#ifdef SYSTEM_TJ
    QString strPortStage  = "192.168.0.11";
    QString strPortFocus  = "COM11";
    QString strPortLManip = "COM1";
    QString strPortRManip = "COM12";
    QString strPortPHold  = "COM14";
    QString strPortPInj   = "COM0";
    QString strPortLMotor = "COM15";
    QString strPortRMotor = "COM13";
#endif
//-----------------南开系统 end------------------------

float petriDishHeight = 15000;
float areaMoveHeight = 250;
float focusHigh = INFINITY;
float focusBottom = INFINITY;
float injHigh = INFINITY;
float injBottom = INFINITY;
float tarHoldP;
float tarInjP;
float holdRotateP = -300;
float holdPressure = -3000;
float holdBalanceP = 3300;
float injReleaseP = 10000;
float injBalanceP = 6500;
float injBiasP = 0;
float injExtractSafeP = -10000;
//针尖、细胞的估计尺寸
int OOCYTE_DIA_UM = 120;        //卵细胞的大致直径（单位：um）
int SOMA_DIA_UM   = 20;         //体细胞的大致直径（单位：um）
int HOLD_DIA_UM   = 150;        //吸持针的大致直径（单位：um）
int INJ_DIA_UM    = 30;         //注射针的大致直径（单位：um）
int ZP_WIDTH      = 20;         //透明带的大致宽度（单位：um）

//全局地图尺寸、当前位置信息
int GMAP_WIDTH = -1;                //全局地图的宽度（单位：像素） GMAP_WIDTH  = GMAP_WIDTH_UM /Pixel2Um_x4
int GMAP_HEIGHT = -1;               //全局地图的高度（单位：像素） GMAP_HEIGHT = GMAP_HEIGHT_UM/Pixel2Um_x4
int GTHUMB_WIDTH = 360;             //全局缩略图宽度（单位：像素）
int GTHUMB_HEIGHT = 270;            //全局缩略图高度（单位：像素）
int REGION_THUMB_OX = 0;            //当前图像（左上角）在全局地图（左上角）中的 X位置（单位：像素）
int REGION_THUMB_OY = 0;            //当前图像（左上角）在全局地图（左上角）中的 Y位置（单位：像素）
int REGION_THUMB_WIDTH = -1;        //当前图像在全局缩略图中的宽度 （单位：像素）
int REGION_THUMB_HEIGHT = -1;       //当前图像在全局缩略图中的高度 （单位：像素）
int MOUSE_PRESS_REGION_X = 0;       //鼠标点击的再当前缩略图中的相对 X位置
int MOUSE_PRESS_REGION_Y = 0;       //鼠标点击的再当前缩略图中的相对 Y位置
float GMAP_WIDTH_UM = 2000;         //全局地图的宽度（单位：um）
float GMAP_HEIGHT_UM = 6000;        //全局地图的高度（单位：um）
float REGION_OX_UM = 0;             //当前图像（左上角）在全局地图（左上角）中的 X位置（单位：um）
float REGION_OY_UM = 0;             //当前图像（左上角）在全局地图（左上角）中的 Y位置（单位：um）
//相机参数或配置
float IMG_STAGE_ANGLE = 0;          //显微图像偏转的角度 默认为 0°
float Pixel2Um = Pixel2Um_4x;       //4x 下 1 pixel = 2.78 um
QString exceptionInfo = "";         //异常信息
Objective CurObj = x4;              //默认为 4x

//辅助信息存储变量
MODE    mode = SCNT_BATCH;
WORKSTEP nextStep = BUILDMAP;
QString FILE_PATH = "F:/Public/";
std::vector<cv::Point>  selectPoints;
std::map<std::string, std::vector<std::string>> trajMaps;
cv::Point2f             donorArea = cv::Point2f(INFINITY, INFINITY);
cv::Point2f             workArea =  cv::Point2f(INFINITY, INFINITY);
cv::Point2f             spitArea =  cv::Point2f(INFINITY, INFINITY);
cv::Point2f             spitEdge =  cv::Point2f(INFINITY, INFINITY);
cv::Point2f             storeArea = cv::Point2f(INFINITY, INFINITY);
cv::Point2f             oocyteArea= cv::Point2f(INFINITY, INFINITY);
cv::Point2f             failArea  = cv::Point2f(INFINITY, INFINITY);
Yolo*                   yolo;
Yolo*                   yoloPB;
Vision*                 vision;
Track*                  track;
PServo*                 pServo;
ObjectDetector*         objectDetector;
//关注对象
cv::Mat                 CurrentGray;
cv::Mat                 CurrentBGR;
cv::Mat                 PreGray;
cv::Mat                 Image;
Microscope*             microscope;
Holder*                 holder;
Injector*               injector;
GlobalMap*              globalMap;
HoldPipette*            holdPipette;
InjectPipette*          injectPipette;
Oocyte*                 currentOocyte;
Soma*                   currentSoma;
Oocytes*                allOocytes;
Oocytes*                oocytesInView;
Somas*                  allSomas;
//工作流程
AutoClone*              autoClone;
StepBuildMap*           stepBuildMap;
StepFetchOocyte*        stepFetchOocyte;
StepRotateOutPlane*     stepRotateOutPlane;
StepRotateInPlane*      stepRotateInPlane;
StepPenetrate*          stepPenetrate;
StepEnucleation*        stepEnucleation;
StepExpel*              stepExpel;
StepFetchSoma*          stepFetchSoma;
StepFetchOocyteForSoma* stepFetchOocyteForSoma;
StepRotateForSoma*      stepRotateForSoma;
StepPenetrateForSoma*   stepPenetrateForSoma;
StepInjectSoma*         stepInjectSoma;
StepReleaseOocyte*      stepReleaseOocyte;
StepException*          stepException;
Functions*             functions;
Plots*                 plots;
bool areAllOocytesEnucleated = false;
bool areAllSomasFetched = false;
bool areAllSomasInjected = false;
bool isWorking = false;
bool isUsingSocket = true;
bool isHCI = false;
