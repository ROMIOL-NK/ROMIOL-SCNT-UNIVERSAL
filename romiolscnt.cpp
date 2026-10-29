#include "romiolscnt.h"
#include "ui_romiolscnt.h"
#define LOW_SPEED_FOCUS     5
#define MEDIUM_SPEED_FOCUS  50
#define HIGH_SPEED_FOCUS    500
#define LOW_SPEED_STAGE     20
#define MEDIUM_SPEED_STAGE  200
#define HIGH_SPEED_STAGE    2000
//要将宏的值取成字符串，要宏转义两次；将pro文件中定义的 PYTHONHOME 作为Py_SetPythonHome的参数
#define SSS(R) #R
#define TTT(R) SSS(R)

using namespace cv;
using namespace std;
RomiolSCNT::RomiolSCNT(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::RomiolSCNT)
{
    ui->setupUi(this);
    setWindowIcon(QIcon(":/images/NK.ico"));
    // 状态栏信息：工作流、电机设备、像素
    workFlowStatus = new QLabel;
    ui->statusbar->addPermanentWidget(workFlowStatus);
    motorStatus = new QLabel;
    ui->statusbar->addPermanentWidget(motorStatus);
    pixelStatus = new QLabel;
    ui->statusbar->addPermanentWidget(pixelStatus);

    scnt_batch_group.push_back(ui->BTN_STEP_FETCH_OOCYTE);
    scnt_batch_group.push_back(ui->BTN_STEP_ROTATE_OUT_PLANE);
    scnt_batch_group.push_back(ui->BTN_STEP_ROTATE_IN_PLANE);
    scnt_batch_group.push_back(ui->BTN_STEP_PENETRATE);
    scnt_batch_group.push_back(ui->BTN_STEP_ENUCLEATION);
    scnt_batch_group.push_back(ui->BTN_STEP_RELEASE_OOCYTE);
    scnt_batch_group.push_back(ui->BTN_STEP_EXPEL);
    scnt_batch_group.push_back(ui->BTN_STEP_FETCH_SOMA);
    scnt_batch_group.push_back(ui->BTN_STEP_ROTATE_FOR_SOMA);
    scnt_batch_group.push_back(ui->BTN_STEP_PENETRATE_FOR_SOMA);
    scnt_batch_group.push_back(ui->BTN_STEP_INJECT_SOMA);
    scnt_batch_group.push_back(ui->BTN_STEP_FETCH_OOCYTE_FOR_SOMA);
    scnt_batch_group.push_back(ui->BTN_FINISH_ENU_OOCYTE);
    scnt_batch_group.push_back(ui->BTN_FINISH_FETCH_SOMA);
    scnt_batch_group.push_back(ui->BTN_FINISH_INJ_SOMA);

    scnt_single_group.push_back(ui->BTN_STEP_FETCH_OOCYTE);
    scnt_single_group.push_back(ui->BTN_STEP_ROTATE_OUT_PLANE);
    scnt_single_group.push_back(ui->BTN_STEP_ROTATE_IN_PLANE);
    scnt_single_group.push_back(ui->BTN_STEP_PENETRATE);
    scnt_single_group.push_back(ui->BTN_STEP_ENUCLEATION);
    scnt_single_group.push_back(ui->BTN_STEP_EXPEL);
    scnt_single_group.push_back(ui->BTN_STEP_FETCH_SOMA);
    scnt_single_group.push_back(ui->BTN_STEP_INJECT_SOMA);
    scnt_single_group.push_back(ui->BTN_STEP_RELEASE_OOCYTE);

    QFile pyFile(":/config/viz.py");
    pyFile.copy("./debug/viz.py");
    pyFile.copy("./release/viz.py");
    pyFile.copy("./viz.py");

    QFile parasFile(":/config/controller_paras.json");
    parasFile.copy("controller_paras.json");
    QFile imFile(":/images/demo.jpg");
    imFile.copy("demo.jpg");
    CurrentGray = cv::imread("demo.jpg", 0);
    CurrentBGR  = cv::imread("demo.jpg");
    // ***************************************** 初始化 *****************************************
    connect(ui->CMB_MODE,     SIGNAL(currentTextChanged(QString)), this, SLOT(modeChanged(QString)));
    init();         //初始化：变量、文件夹、定时器
    initDevices();  //初始化所有设备

    // ***************************************    通讯   ***************************************
    connect(ui->BTN_SOCKET_CONNECT,     &QPushButton::clicked, [this](){
        if(ui->BTN_SOCKET_CONNECT->isChecked())
        {
            emit sgnConnectSocket();
            ui->BTN_SOCKET_CONNECT->setText("断\n开");
        }
        else
        {
            emit sgnCloseSocket();
            ui->BTN_SOCKET_CONNECT->setText("连\n接");
        }
    });

    // **************************************** 设备操控 ****************************************
    // ------------------- 连接设备 -------------------
    connect(ui->BTN_CAMERA,             SIGNAL(clicked()), this, SLOT(connectCamera()));        //连接相机
    connect(ui->LED_FOCUSDEVICE,        SIGNAL(clicked()), this, SLOT(connectFocusDevice()));   //连接调焦设备
    connect(ui->LED_STAGE,              SIGNAL(clicked()), this, SLOT(connectStage()));         //连接平台
    connect(ui->LED_LEFT_MANIP,         SIGNAL(clicked()), this, SLOT(connectLManip()));        //连接左操作臂
    connect(ui->LED_RIGHT_MANIP,        SIGNAL(clicked()), this, SLOT(connectRManip()));        //连接右操作臂
    connect(ui->LED_HOLDER_PRESSURE,    SIGNAL(clicked()), this, SLOT(connectHoldPSensor()));   //连接吸持针气压传感器
    connect(ui->LED_INJECTOR_PRESSURE,  SIGNAL(clicked()), this, SLOT(connectInjPSensor()));    //连接注射针气压传感器
    connect(ui->LED_HOLDER,             SIGNAL(clicked()), this, SLOT(connectHolderMotor()));   //连接左注射器电机
    connect(ui->LED_INJECTOR,           SIGNAL(clicked()), this, SLOT(connectInjectorMotor())); //连接右注射器电机
    // -------------------- 显微镜 --------------------
    connect(ui->BTN_SCOPE_UP,       SIGNAL(clicked()), this, SLOT(btnScopeUp()));               //调焦
    connect(ui->BTN_SCOPE_DOWN,     SIGNAL(clicked()), this, SLOT(btnScopeDown()));             //调焦
    connect(ui->BTN_SCOPE_LEFT,     SIGNAL(clicked()), this, SLOT(btnScopeLeft()));             //移动平台
    connect(ui->BTN_SCOPE_RIGHT,    SIGNAL(clicked()), this, SLOT(btnScopeRight()));            //移动平台
    connect(ui->BTN_SCOPE_FORWARD,  SIGNAL(clicked()), this, SLOT(btnScopeForward()));          //移动平台
    connect(ui->BTN_SCOPE_BACK,     SIGNAL(clicked()), this, SLOT(btnScopeBack()));             //移动平台
    //相机
    connect(ui->BTN_CAPTURE,        SIGNAL(clicked()), this, SLOT(btnCapture()));               //相机：抓拍
    connect(ui->BTN_VIDEO,          SIGNAL(clicked()), this, SLOT(btnVideo()));                 //相机：录像
    connect(ui->BTN_PAUSE_RECORD,   SIGNAL(clicked()), this, SLOT(btnPauseRecord()));           //相机：暂停录像
    connect(ui->BTN_SEL_PATH,       SIGNAL(clicked()), this, SLOT(btnSelPath()));               //选择文件保存位置：图像、视频 ...
    connect(ui->BTN_SEL_IMG_SOURCE, SIGNAL(clicked()), this, SLOT(btnSelImgSource()));          //选择图像源文件
    connect(ui->BTN_PLAY_SOURCE,    SIGNAL(clicked()), this, SLOT(btnPlaySource()));            //播放选择的图像源
    connect(ui->CMB_IMG_SOURCE,     SIGNAL(currentTextChanged(QString)), this, SLOT(imgSourceChanged(QString)));
    connect(ui->BTN_PLAY_NEXT,      &QPushButton::clicked, [this](){if(!imgSourcePath.empty()) imgSourcePath.removeFirst();});

    //镜头转换
    connect(ui->BTN_X4,  &QPushButton::clicked, [](){microscope->changeObjective(x4 );});   //选择 x4 物镜，移动平台、调焦，使左上角保持不变
    connect(ui->BTN_X10, &QPushButton::clicked, [](){microscope->changeObjective(x10);});   //选择 x10物镜，移动平台、调焦，使左上角保持不变
    connect(ui->BTN_X20, &QPushButton::clicked, [](){microscope->changeObjective(x20);});   //选择 x20物镜，移动平台、调焦，使左上角保持不变
    connect(ui->BTN_X40, &QPushButton::clicked, [](){microscope->changeObjective(x40);});   //选择 x40物镜，移动平台、调焦，使左上角保持不变

    // --------------------吸持装置--------------------
    //左操作臂
    connect(ui->BTN_LEFT_MANIP_BACK,    SIGNAL(clicked()), this, SLOT(btnLManipBack()));
    connect(ui->BTN_LEFT_MANIP_FORWARD, SIGNAL(clicked()), this, SLOT(btnLManipForward()));
    connect(ui->BTN_LEFT_MANIP_LEFT,    SIGNAL(clicked()), this, SLOT(btnLManipLeft()));
    connect(ui->BTN_LEFT_MANIP_RIGHT,   SIGNAL(clicked()), this, SLOT(btnLManipRight()));
    connect(ui->BTN_LEFT_MANIP_UP,      SIGNAL(clicked()), this, SLOT(btnLManipUp()));
    connect(ui->BTN_LEFT_MANIP_DOWN,    SIGNAL(clicked()), this, SLOT(btnLManipDown()));
    //注射电机
    connect(ui->BTN_HOLDER_SUCK,  SIGNAL(clicked()), this, SLOT(btnHolderSuck()));
    connect(ui->BTN_HOLDER_BLOW,  SIGNAL(clicked()), this, SLOT(btnHolderBlow()));

    // --------------------注射装置--------------------
    //右操作臂
    connect(ui->BTN_RIGHT_MANIP_BACK,   SIGNAL(clicked()), this, SLOT(btnRManipBack()));
    connect(ui->BTN_RIGHT_MANIP_FORWARD,SIGNAL(clicked()), this, SLOT(btnRManipForward()));
    connect(ui->BTN_RIGHT_MANIP_LEFT,   SIGNAL(clicked()), this, SLOT(btnRManipLeft()));
    connect(ui->BTN_RIGHT_MANIP_RIGHT,  SIGNAL(clicked()), this, SLOT(btnRManipRight()));
    connect(ui->BTN_RIGHT_MANIP_UP,     SIGNAL(clicked()), this, SLOT(btnRManipUp()));
    connect(ui->BTN_RIGHT_MANIP_DOWN,   SIGNAL(clicked()), this, SLOT(btnRManipDown()));
    //注射电机
    connect(ui->BTN_INJECTOR_SUCK,  SIGNAL(clicked()), this, SLOT(btnInjectorSuck()));
    connect(ui->BTN_INJECTOR_BLOW,  SIGNAL(clicked()), this, SLOT(btnInjectorBlow()));

    // ************************************* 工作流（多线程） ************************************
    connect(ui->BTN_SET_DONOR_AREA, &QPushButton::clicked, [this](){donorArea=Point2f(microscope->getRelX(), microscope->getRelY());ui->BTN_SET_DONOR_AREA->setChecked(true);});
    connect(ui->BTN_SET_SPIT_AREA,  &QPushButton::clicked, [this](){spitArea =Point2f(microscope->getRelX(), microscope->getRelY());ui->BTN_SET_SPIT_AREA->setChecked(true);});
    connect(ui->BTN_SET_SPIT_EDGE,  &QPushButton::clicked, [this](){spitEdge =Point2f(microscope->getRelX(), microscope->getRelY());ui->BTN_SET_SPIT_EDGE->setChecked(true);});
    connect(ui->BTN_SET_WORK_AREA,  &QPushButton::clicked, [this](){workArea =Point2f(microscope->getRelX(), microscope->getRelY());ui->BTN_SET_WORK_AREA->setChecked(true);});
    connect(ui->BTN_SET_STORE_AREA, &QPushButton::clicked, [this](){storeArea=Point2f(microscope->getRelX(), microscope->getRelY());ui->BTN_SET_STORE_AREA->setChecked(true);});
    connect(ui->BTN_SET_FAIL_AREA,  &QPushButton::clicked, [this](){failArea =Point2f(microscope->getRelX(), microscope->getRelY());ui->BTN_SET_FAIL_AREA->setChecked(true);});

    connect(ui->BTN_GOTO_DONOR_AREA,&QPushButton::clicked, [this](){emit sgnGotoArea(donorArea);});
    connect(ui->BTN_GOTO_SPIT_AREA, &QPushButton::clicked, [this](){emit sgnGotoArea(spitArea);});
    connect(ui->BTN_GOTO_SPIT_EDGE, &QPushButton::clicked, [this](){emit sgnGotoArea(spitEdge);});
    connect(ui->BTN_GOTO_WORK_AREA, &QPushButton::clicked, [this](){emit sgnGotoArea(workArea);});
    connect(ui->BTN_GOTO_STORE_AREA,&QPushButton::clicked, [this](){emit sgnGotoArea(storeArea);});
    connect(ui->BTN_GOTO_OOCYTE_AREA,&QPushButton::clicked,[this](){emit sgnGotoArea(oocyteArea);});
    connect(ui->BTN_GOTO_FAIL_AREA, &QPushButton::clicked, [this](){emit sgnGotoArea(failArea);});

    connect(ui->BTN_FINISH_ENU_OOCYTE,&QPushButton::clicked,[this](){if(ui->BTN_FINISH_ENU_OOCYTE->isChecked()) areAllOocytesEnucleated=true; else areAllOocytesEnucleated=false;});
    connect(ui->BTN_FINISH_FETCH_SOMA,&QPushButton::clicked,[this](){if(ui->BTN_FINISH_FETCH_SOMA->isChecked()) areAllSomasFetched=true; else areAllSomasFetched=false;});
    connect(ui->BTN_FINISH_INJ_SOMA,&QPushButton::clicked, [this](){if(ui->BTN_FINISH_INJ_SOMA->isChecked()) areAllSomasInjected=true; else areAllSomasInjected=false;});
    connect(ui->BTN_OBJECT_DETECTOR,SIGNAL(clicked()), this, SLOT(btnObjectDetector()));
    // **************************************** 辅助功能 ****************************************
    //附加功能
    connect(ui->BTN_RESET,      SIGNAL(clicked()), this, SLOT(btnReset()));
    connect(ui->BTN_POS_TYPE,   SIGNAL(clicked()), this, SLOT(btnPosType()));
    connect(ui->BTN_SET_ORIGIN, SIGNAL(clicked()), this, SLOT(btnSetOrigin()));
    connect(ui->BTN_COLOR,      SIGNAL(clicked()), this, SLOT(btnColor()));
    connect(ui->BTN_SHOW_ALL,   SIGNAL(clicked()), this, SLOT(btnShowAll()));
    connect(ui->BTN_RESTART, SIGNAL(clicked()), this, SLOT(btnRestart()));

    connect(thumbnailTimer,         SIGNAL(timeout()),      this, SLOT(updateThumbnail()));//更新全局缩略图
    connect(playSourceTimer,        SIGNAL(timeout()),      this, SLOT(scopeUpdateImage()));//更新图像
    connect(recordTimer,            SIGNAL(timeout()),      this, SLOT(recordData()));//记录数据

    //菜单栏
    connect(ui->MENU_HUMAN,         SIGNAL(aboutToShow()),  this, SLOT(menuHumanAssistance()));
}
RomiolSCNT::~RomiolSCNT()
{
    //退出 Python
    Py_Finalize();
    //如果当天没有保存文件，删除新建的文件夹
    QDir dir(FILE_PATH);
    if(dir.isEmpty())
        dir.removeRecursively();
    emit sgnCloseStage();
    emit sgnCloseCamera();
    emit sgnCloseFocus();
    emit sgnCloseLManip();
    emit sgnCloseLPSensor();
    emit sgnCloseLSMotor();
    emit sgnCloseRManip();
    emit sgnCloseRPSensor();
    emit sgnCloseRSMotor();
    //结束所有子线程
    threadVision.terminate();
    threadTrack.terminate();
    threadPServo.terminate();

    threadCamera.terminate();
    threadFocus.terminate();
    threadStage.terminate();
    threadLManip.terminate();
    threadLPSensor.terminate();
    threadLSMotor.terminate();
    threadRManip.terminate();
    threadRPSensor.terminate();
    threadRSMotor.terminate();

    threadFunctions.terminate();
    threadWorkFlow.terminate();
    //删除新建的指针
    // delete microscope;
    // delete holder;
    // delete injector;
    delete holdPipette;
    delete injectPipette;
    delete currentOocyte;
    delete currentSoma;
    delete allOocytes;
    delete allSomas;
    delete oocytesInView;
    delete yolo;
    delete yoloPB;
    delete globalMap;

    delete thumbnailTimer;
    delete playSourceTimer;
    delete recordTimer;

    delete objectDetector;
    delete track;
    delete pServo;

    delete stepBuildMap;
    delete stepFetchOocyte;
    delete stepRotateOutPlane;
    delete stepRotateInPlane;
    delete stepPenetrate;
    delete stepEnucleation;
    delete stepExpel;
    delete stepFetchSoma;
    delete stepFetchOocyteForSoma;
    delete stepRotateForSoma;
    delete stepPenetrateForSoma;
    delete stepInjectSoma;
    delete stepReleaseOocyte;
    delete stepException;
    delete autoClone;
    delete functions;
    delete plots;
    delete ui;
}

void RomiolSCNT::init()
{
    holdPipette =   new HoldPipette;
    injectPipette = new InjectPipette;
    currentOocyte = new Oocyte;
    currentSoma =   new Soma;
    allOocytes =    new Oocytes;
    oocytesInView = new Oocytes;
    allSomas =      new Somas;
    yolo =          new Yolo;
    yoloPB =        new Yolo("polarbody", cv::Size(256, 256));
    globalMap =     new GlobalMap;
    //定时器初始化
    thumbnailTimer = new QTimer(this);
    playSourceTimer = new QTimer(this);
    recordTimer = new QTimer(this);
    isRecording = false;
    recordDT = 50;  //50 ms
    //创建新的文件夹，保存当天的图像、数据等 (如果关闭时文件夹为空，自动删除)
    QDateTime dateTime(QDateTime::currentDateTime());
    QString time = dateTime.toString("yyyy-MM-dd");
    FILE_PATH = FILE_PATH + QString("%1").arg(time);
    if(!QDir(FILE_PATH).exists())
        QDir().mkpath(FILE_PATH);
    // python初始化
    string str = TTT(PYTHONHOME);
    wchar_t * wc = new wchar_t[str.size()];
    swprintf(wc,100,L"%S",str.c_str());
    Py_SetPythonHome(wc);
    Py_Initialize();
    if(!Py_IsInitialized())
        qDebug()<<"Error Python Initialize";
    // Object Detector
    vision = new Vision();
    objectDetector = new ObjectDetector();
    objectDetector->moveToThread(&threadVision);

    track = new Track();
    track->moveToThread(&threadTrack);
    pServo = new PServo();
    pServo->moveToThread(&threadPServo);
    autoFocus = new AutoFocus();
    autoFocus->moveToThread(&threadWorkFlow);

    // 连接各步骤的子线程信号
    stepBuildMap = new StepBuildMap();
    stepFetchOocyte = new StepFetchOocyte();
    stepRotateOutPlane = new StepRotateOutPlane();
    stepRotateInPlane = new StepRotateInPlane();
    stepPenetrate = new StepPenetrate();
    stepEnucleation = new StepEnucleation();
    stepExpel = new StepExpel();
    stepFetchSoma = new StepFetchSoma();
    stepFetchOocyteForSoma = new StepFetchOocyteForSoma();
    stepRotateForSoma = new StepRotateForSoma();
    stepPenetrateForSoma = new StepPenetrateForSoma();
    stepInjectSoma = new StepInjectSoma();
    stepReleaseOocyte = new StepReleaseOocyte();
    stepException = new StepException();//因为异常处理不需要单独按钮和线程（总是在自动化线程中执行），不需要后面的信号连接
    autoClone = new AutoClone();
    functions = new Functions();
    plots = new Plots();
    stepBuildMap->moveToThread(&threadWorkFlow);
    stepFetchOocyte->moveToThread(&threadWorkFlow);
    stepRotateOutPlane->moveToThread(&threadWorkFlow);
    stepRotateInPlane->moveToThread(&threadWorkFlow);
    stepPenetrate->moveToThread(&threadWorkFlow);
    stepEnucleation->moveToThread(&threadWorkFlow);
    stepExpel->moveToThread(&threadWorkFlow);
    stepFetchSoma->moveToThread(&threadWorkFlow);
    stepFetchOocyteForSoma->moveToThread(&threadWorkFlow);
    stepRotateForSoma->moveToThread(&threadWorkFlow);
    stepPenetrateForSoma->moveToThread(&threadWorkFlow);
    stepInjectSoma->moveToThread(&threadWorkFlow);
    stepReleaseOocyte->moveToThread(&threadWorkFlow);
    autoClone->moveToThread(&threadWorkFlow);
    stepException->moveToThread(&threadWorkFlow);
    functions->moveToThread(&threadFunctions);

    threadWorkFlow.start();
    threadFunctions.start();
    threadVision.start();
    threadTrack.start();
    threadPServo.start();

    ui->BTN_STEP_BUILD_MAP->setStyleSheet(READY_QSS);
    ui->BTN_STEP_BUILD_MAP->setChecked(false);
    connect(ui->BTN_STEP_BUILD_MAP, SIGNAL(clicked()), this, SLOT(btnStepBuildMap()));
    connect(ui->BTN_STEP_FETCH_OOCYTE, SIGNAL(clicked()), this, SLOT(btnStepFetchOocyte()));
    connect(ui->BTN_STEP_ROTATE_OUT_PLANE, SIGNAL(clicked()), this, SLOT(btnStepRotateOutPlane()));
    connect(ui->BTN_STEP_ROTATE_IN_PLANE, SIGNAL(clicked()), this, SLOT(btnStepRotateInPlane()));
    connect(ui->BTN_STEP_PENETRATE, SIGNAL(clicked()), this, SLOT(btnStepPenetrate()));
    connect(ui->BTN_STEP_ENUCLEATION, SIGNAL(clicked()), this, SLOT(btnStepEnucleation()));
    connect(ui->BTN_STEP_EXPEL, SIGNAL(clicked()), this, SLOT(btnStepExpel()));
    connect(ui->BTN_STEP_FETCH_SOMA, SIGNAL(clicked()), this, SLOT(btnStepFetchSoma()));    
    connect(ui->BTN_STEP_FETCH_OOCYTE_FOR_SOMA, SIGNAL(clicked()), this, SLOT(btnStepFetchOocyteForSoma()));
    connect(ui->BTN_STEP_ROTATE_FOR_SOMA, SIGNAL(clicked()), this, SLOT(btnStepRotateForSoma()));
    connect(ui->BTN_STEP_PENETRATE_FOR_SOMA, SIGNAL(clicked()), this, SLOT(btnStepPenetrateForSoma()));
    connect(ui->BTN_STEP_INJECT_SOMA, SIGNAL(clicked()), this, SLOT(btnStepInjectSoma()));
    connect(ui->BTN_STEP_RELEASE_OOCYTE, SIGNAL(clicked()), this, SLOT(btnStepReleaseOocyte()));
    connect(ui->BTN_AUTOCLONE, SIGNAL(clicked()), this, SLOT(btnAutoClone()));

    connect(ui->BTN_CFG_CONTROL_PARAS,&QPushButton::clicked,[this](){
        QString str=ui->CMB_CONTROLLER_LIST->currentText();
        if(str=="注射液面")cfgControlParas(functions->injGLI_Controller);
        else if(str=="去核")cfgControlParas(stepEnucleation->enuController);
        else if(str=="去核气压")cfgControlParas(stepEnucleation->injP_Controller);
        else if(str=="吸持压")cfgControlParas(functions->holdP_Controller);
        else if(str=="注射压")cfgControlParas(functions->injP_Controller);
        else if(str=="吸持伺服")cfgControlParas(pServo->pHoldController);
        else if(str=="注射伺服")cfgControlParas(pServo->pInjController);
    });
    connect(ui->BTN_CFG_TRAJ_PARAS, &QPushButton::clicked,[this](){
        QString str=ui->CMB_CONTROLLER_LIST->currentText();
        if(str=="注射液面")cfgTrajParas(functions->injGLI_Controller);
        else if(str=="去核")cfgTrajParas(stepEnucleation->enuController);
        else if(str=="去核气压")cfgTrajParas(stepEnucleation->injP_Controller);
        else if(str=="吸持压")cfgTrajParas(functions->holdP_Controller);
        else if(str=="注射压")cfgTrajParas(functions->injP_Controller);
        else if(str=="吸持伺服")cfgControlParas(pServo->pHoldController);
        else if(str=="注射伺服")cfgControlParas(pServo->pInjController);});
    connect(ui->BTN_STOP,           &QPushButton::clicked, [](){functions->ON = false;});
    connect(ui->BTN_ADJ_INJ_GLI,    &QPushButton::clicked, [this](){ui->BTN_ADJ_INJ_GLI->setChecked(true);emit sgnFunction("ADJ_INJ_GLI");});
    connect(ui->BTN_ADJ_HOLD_P,     &QPushButton::clicked, [this](){ui->BTN_ADJ_HOLD_P->setChecked(true);tarHoldP=ui->EDIT_HOLD_P->text().toDouble();emit sgnFunction("ADJ_HOLD_P");});
    connect(ui->BTN_ADJ_INJ_P,      &QPushButton::clicked, [this](){ui->BTN_ADJ_INJ_P->setChecked(true);tarInjP =ui->EDIT_INJ_P->text().toDouble(); emit sgnFunction("ADJ_INJ_P");});
    connect(ui->BTN_WITHDRAW_INJ,   &QPushButton::clicked, [this](){ui->BTN_WITHDRAW_INJ->setChecked(true);emit sgnFunction("WITHDRAW_INJ");});
    connect(ui->BTN_CO_UP,          &QPushButton::clicked, [this](){ui->BTN_CO_UP->setChecked(true);emit sgnFunction("CO_UP");});
    connect(ui->BTN_CO_DOWN,        &QPushButton::clicked, [this](){ui->BTN_CO_DOWN->setChecked(true);emit sgnFunction("CO_DOWN");});
    connect(ui->BTN_SET_FOCUS_HIGH, &QPushButton::clicked, [this](){focusHigh = microscope->getAbsZ();ui->BTN_SET_FOCUS_HIGH->setChecked(true);});
    connect(ui->BTN_SET_FOCUS_BOTTOM,&QPushButton::clicked,[this](){focusBottom = microscope->getAbsZ();ui->BTN_SET_FOCUS_BOTTOM->setChecked(true);});
    connect(ui->BTN_SET_INJ_HIGH,   &QPushButton::clicked, [this](){injHigh = injector->manipulator->getAbsZ();ui->BTN_SET_INJ_HIGH->setChecked(true);});
    connect(ui->BTN_SET_INJ_BOTTOM, &QPushButton::clicked, [this](){injBottom = injector->manipulator->getAbsZ();ui->BTN_SET_INJ_BOTTOM->setChecked(true);});
    connect(ui->BTN_FOCUS_TO_HIGH,  &QPushButton::clicked, [this](){emit sgnMoveFocusZ(focusHigh-microscope->getAbsZ(),false,200);});
    connect(ui->BTN_FOCUS_TO_BOTTOM,&QPushButton::clicked, [this](){emit sgnMoveFocusZ(focusBottom-microscope->getAbsZ(),false,200);});
    connect(ui->BTN_INJ_TO_HIGH,    &QPushButton::clicked, [this](){emit sgnMoveRManipZ(injHigh-injector->manipulator->getAbsZ(),500,false);});
    connect(ui->BTN_INJ_TO_BOTTOM,  &QPushButton::clicked, [this](){emit sgnMoveRManipZ(injBottom-injector->manipulator->getAbsZ(),500,false);});


    connect(ui->BTN_MOVE_UP,        &QPushButton::clicked, [this](){ui->BTN_MOVE_UP->setChecked(true);  emit sgnMoveLManipZ(-areaMoveHeight, 1000, false); emit sgnMoveRManipZ(-areaMoveHeight, 1000, false);   ui->BTN_MOVE_UP->setChecked(false);});
    connect(ui->BTN_MOVE_DOWN,      &QPushButton::clicked, [this](){ui->BTN_MOVE_DOWN->setChecked(true);emit sgnMoveLManipZ( areaMoveHeight, 1000, false); emit sgnMoveRManipZ( areaMoveHeight, 1000, false);   ui->BTN_MOVE_DOWN->setChecked(false);});


    connect(ui->BTN_NEXT_OOCYTE,    &QPushButton::clicked, [this](){ui->BTN_NEXT_OOCYTE->setChecked(true);emit sgnFunction("NEXT_OOCYTE");});
    connect(ui->BTN_FIND_OOCYTE,    &QPushButton::clicked, [this](){ui->BTN_FIND_OOCYTE->setChecked(true);emit sgnFunction("FIND_OOCYTE");});
    connect(ui->BTN_HOLD_OOCYTE,    &QPushButton::clicked, [this](){ui->BTN_HOLD_OOCYTE->setChecked(true);emit sgnFunction("HOLD_OOCYTE");});
    connect(ui->BTN_RELEASE_OOCYTE, &QPushButton::clicked, [this](){ui->BTN_RELEASE_OOCYTE->setChecked(true);emit sgnFunction("RELEASE_OOCYTE");});
    connect(ui->BTN_FOCUS_OOCYTE,   &QPushButton::clicked, [this](){ui->BTN_FOCUS_OOCYTE->setChecked(true);emit sgnFunction("FOCUS_OOCYTE");});
    connect(ui->BTN_FOCUS_HOLD,     &QPushButton::clicked, [this](){ui->BTN_FOCUS_HOLD->setChecked(true);emit sgnFunction("FOCUS_HOLD");});
    connect(ui->BTN_FOCUS_INJ,      &QPushButton::clicked, [this](){ui->BTN_FOCUS_INJ->setChecked(true);emit sgnFunction("FOCUS_INJ");});
    connect(ui->BTN_TEST,           &QPushButton::clicked, [this](){emit sgnFunction("TEST");});
    connect(ui->BTN_USING_SOCKET,   &QPushButton::clicked, [this](){if(ui->BTN_USING_SOCKET->isChecked()) isUsingSocket=true; else isUsingSocket=false;});
    connect(ui->BTN_HCI,            &QPushButton::clicked, [this](){if(ui->BTN_HCI->isChecked())          isHCI=true; else isHCI=false;});
    connect(ui->BTN_RECORD,         &QPushButton::clicked, [this](){
        if(!isRecording)
        {
            ui->BTN_RECORD->setChecked(true);
            QDateTime dateTime(QDateTime::currentDateTime());
            QString time = dateTime.toString("hh-mm-ss");

            QString tmp = FILE_PATH+"/record_"+QString("%1").arg(time)+".txt";

            // datasetPath = FILE_PATH+"/"+QString("%1").arg(time)+"/";
            // if(!QDir(datasetPath).exists())
            //     QDir().mkpath(datasetPath);
            // QString tmp = datasetPath+"record.txt";

            recordfile.open(tmp.toStdString(), 'w');

            recordfile<<"φ_injPipette(μm): "<<injectPipette->diameter*Pixel2Um<<"\tΔΤ="<<recordDT<<endl;
            recordfile<<"P_inj(Pa)\tx_motor\tv_motor\tx_OCI(μm)\tStrain\tx_GLI\n";

            // recordfile<<"Time\tscopeX\tscopeY\tscopeZ\tHoldX\tHoldY\tHoldZ\tInjectX\tInjectY\tInjectZ"<<endl;//电机位置

            recordTimer->start(recordDT);
            isRecording = true;
        }
        else
        {
            ui->BTN_RECORD->setChecked(false);
            recordfile.close();
            recordTimer->stop();
            isRecording = false;
        }
    });
    connect(ui->BTN_DRAW,           &QPushButton::clicked, [this](){
        QString str=ui->CMB_CONTROLLER_LIST->currentText();
        if(str=="注射液面"){functions->injGLI_Controller->drawTrajectory(); functions->injGLI_Controller->drawU();}
        else if(str=="去核"){stepEnucleation->enuController->drawTrajectory(); stepEnucleation->enuController->drawU();}
        else if(str=="去核气压"){stepEnucleation->injP_Controller->drawTrajectory(); stepEnucleation->injP_Controller->drawU();}
        else if(str=="吸持压"){functions->holdP_Controller->drawTrajectory(); functions->holdP_Controller->drawU();}
        else if(str=="注射压"){functions->injP_Controller->drawTrajectory(); functions->injP_Controller->drawU();}
        else if(str=="吸持伺服"){pServo->pHoldController->drawTrajectory(); pServo->pHoldController->drawU();}
        else if(str=="注射伺服"){pServo->pInjController->drawTrajectory(); pServo->pInjController->drawU();}
    });
    connect(ui->BTN_SAVE_CTL_RESULT,&QPushButton::clicked, [this](){
        QString str=ui->CMB_CONTROLLER_LIST->currentText();
        if(str=="注射液面"){functions->injGLI_Controller->save_data("GLI_data.txt");}
        else if(str=="去核"){stepEnucleation->enuController->save_data("OCI_data.txt");}
        else if(str=="去核气压"){stepEnucleation->injP_Controller->save_data("Enu_P_data.txt");}
        else if(str=="吸持压"){functions->holdP_Controller->save_data("holdP_data.txt");}
        else if(str=="注射压"){functions->injP_Controller->save_data("injP_data.txt");}
        else if(str=="吸持伺服"){pServo->pHoldController->save_data("holdServo_data.txt");}
        else if(str=="注射伺服"){pServo->pInjController->save_data("injServo_data.txt");}
    });

    connect(ui->BTN_LOAD_MAP,       &QPushButton::clicked, [this](){
        globalMap->mapImage=imread((FILE_PATH+"/globalMap.png").toStdString(), 0);
        globalMap->pixel2Um = Pixel2Um;
        oocyteArea = Point2f(microscope->getRelX(), microscope->getRelY());
        GMAP_WIDTH = globalMap->mapImage.cols;
        GMAP_HEIGHT = globalMap->mapImage.rows;
        GMAP_HEIGHT_UM = GMAP_WIDTH*Pixel2Um;
        GMAP_WIDTH_UM  = GMAP_HEIGHT*Pixel2Um;
        allOocytes->find(globalMap->mapImage, oocyteArea);
        thumbnailTimer->start(50);
    });
    connect(ui->BTN_LOAD_PARAS, &QPushButton::clicked, [this](){
        // 加载默认控制器及相关参数
        QString filePath("controller_paras.json");
        delete functions->injP_Controller;
        functions->injP_Controller = new Controller_PNEUMATIC;
        functions->injP_Controller->loadParasFromJson(filePath, "injP_Controller_PNEUMATIC_spd");
        delete functions->holdP_Controller;
        functions->holdP_Controller = new Controller_PID;
        functions->holdP_Controller->loadParasFromJson(filePath, "holdP_Controller_PID");
    });
    connect(this, SIGNAL(sgnStepBuildMap()),        stepBuildMap, SLOT(run()));
    connect(this, SIGNAL(sgnStepFetchOocyte()),     stepFetchOocyte, SLOT(run()));
    connect(this, SIGNAL(sgnStepRotateOutPlane()),  stepRotateOutPlane, SLOT(run()));
    connect(this, SIGNAL(sgnStepRotateInPlane()),   stepRotateInPlane, SLOT(run()));
    connect(this, SIGNAL(sgnStepPenetrate()),       stepPenetrate, SLOT(run()));
    connect(this, SIGNAL(sgnStepEnucleation()),     stepEnucleation, SLOT(run()));
    connect(this, SIGNAL(sgnStepExpel()),           stepExpel, SLOT(run()));
    connect(this, SIGNAL(sgnStepFetchSoma()),       stepFetchSoma, SLOT(run()));
    connect(this, SIGNAL(sgnStepFetchOocyteForSoma()),  stepFetchOocyteForSoma, SLOT(run()));
    connect(this, SIGNAL(sgnStepRotateForSoma()),       stepRotateForSoma, SLOT(run()));
    connect(this, SIGNAL(sgnStepPenetrateForSoma()),    stepPenetrateForSoma, SLOT(run()));
    connect(this, SIGNAL(sgnStepInjectSoma()),      stepInjectSoma, SLOT(run()));
    connect(this, SIGNAL(sgnStepReleaseOocyte()),   stepReleaseOocyte, SLOT(run()));
    connect(this, SIGNAL(sgnAutoClone(WORKSTEP)),   autoClone, SLOT(run(WORKSTEP)));
    connect(this, SIGNAL(sgnFunction(QString)),     functions, SLOT(run(QString)));

    connect(stepBuildMap,       SIGNAL(sgnFault(QString)), this, SLOT(dealFault(QString)));
    connect(stepFetchOocyte,    SIGNAL(sgnFault(QString)), this, SLOT(dealFault(QString)));
    connect(stepRotateOutPlane, SIGNAL(sgnFault(QString)), this, SLOT(dealFault(QString)));
    connect(stepRotateInPlane,  SIGNAL(sgnFault(QString)), this, SLOT(dealFault(QString)));
    connect(stepPenetrate,      SIGNAL(sgnFault(QString)), this, SLOT(dealFault(QString)));
    connect(stepEnucleation,    SIGNAL(sgnFault(QString)), this, SLOT(dealFault(QString)));
    connect(stepExpel,          SIGNAL(sgnFault(QString)), this, SLOT(dealFault(QString)));
    connect(stepFetchSoma,      SIGNAL(sgnFault(QString)), this, SLOT(dealFault(QString)));
    connect(stepFetchOocyteForSoma, SIGNAL(sgnFault(QString)), this, SLOT(dealFault(QString)));
    connect(stepRotateForSoma,      SIGNAL(sgnFault(QString)), this, SLOT(dealFault(QString)));
    connect(stepPenetrateForSoma,   SIGNAL(sgnFault(QString)), this, SLOT(dealFault(QString)));
    connect(stepInjectSoma,     SIGNAL(sgnFault(QString)), this, SLOT(dealFault(QString)));
    connect(stepReleaseOocyte,  SIGNAL(sgnFault(QString)), this, SLOT(dealFault(QString)));

    connect(stepBuildMap,       SIGNAL(finish()), this, SLOT(finishStepBuildMap()));
    connect(stepFetchOocyte,    SIGNAL(finish()), this, SLOT(finishStepFetchOocyte()));
    connect(stepRotateOutPlane, SIGNAL(finish()), this, SLOT(finishStepRotateOutPlane()));
    connect(stepRotateInPlane,  SIGNAL(finish()), this, SLOT(finishStepRotateInPlane()));
    connect(stepPenetrate,      SIGNAL(finish()), this, SLOT(finishStepPenetrate()));
    connect(stepEnucleation,    SIGNAL(finish()), this, SLOT(finishStepEnucleation()));
    connect(stepExpel,          SIGNAL(finish()), this, SLOT(finishStepExpel()));
    connect(stepFetchSoma,      SIGNAL(finish()), this, SLOT(finishStepFetchSoma()));
    connect(stepFetchOocyteForSoma, SIGNAL(finish()), this, SLOT(finishStepFetchOocyteForSoma()));
    connect(stepRotateForSoma,      SIGNAL(finish()), this, SLOT(finishStepRotateForSoma()));
    connect(stepPenetrateForSoma,   SIGNAL(finish()), this, SLOT(finishStepPenetrateForSoma()));
    connect(stepInjectSoma,     SIGNAL(finish()), this, SLOT(finishStepInjectSoma()));
    connect(stepReleaseOocyte,  SIGNAL(finish()), this, SLOT(finishStepReleaseOocyte()));
    connect(autoClone,          SIGNAL(finish()), this, SLOT(finishAutoClone()));

    connect(functions,          SIGNAL(finish(QString)), this, SLOT(finishFunc(QString)));

    // yolo ->run(CurrentBGR);
    // Socket 通信
    vision->moveToThread(&threadVision);
    connect(this, SIGNAL(sgnConnectSocket()), vision, SLOT(connectServer()));
    connect(this, SIGNAL(sgnCloseSocket()), vision, SLOT(close()));

    trajMaps["SETPOINT"] = {"setPoint"};
    trajMaps["SIGMOID"] = {"a", "b", "k"};
    trajMaps["SINE"] = {"A", "w", "p", "B"};
    trajMaps["ACCDEC"] = {"AM", "DM", "VM", "XF", "TF"};
    trajMaps["SLOPE"] = {"VM", "XF"};
}
void RomiolSCNT::initDevices()
{
    microscope =    new Microscope;
    holder =        new Holder;
    injector =      new Injector;
    //调焦设备
    connect(microscope->focusdevice,    SIGNAL(openPortFailure()),  this, SLOT(focusOpenPortFailure()));    //调焦设备：打开端口失败
    connect(microscope->focusdevice,    SIGNAL(startConnection()),  this, SLOT(focusStartConnection()));    //调焦设备：开始连接
    connect(microscope->focusdevice,    SIGNAL(noConnection()),     this, SLOT(focusNoConnection()));       //调焦设备：失去连接
    connect(microscope->focusdevice,    SIGNAL(updateZPos()),       this, SLOT(scopeUpdateZ()));            //调焦设备：更新 z 位置
    //平台
    connect(microscope->stage,          SIGNAL(openPortFailure()),  this, SLOT(stageOpenPortFailure()));    //平台：打开端口失败
    connect(microscope->stage,          SIGNAL(startConnection()),  this, SLOT(stageStartConnection()));    //平台：开始连接
    connect(microscope->stage,          SIGNAL(updateXYPos()),      this, SLOT(scopeUpdateXY()));           //平台：更新 xy 位置
    connect(microscope->stage,          SIGNAL(noConnection()),     this, SLOT(stageNoConnection()));      //左操作臂：失去连接
    //相机
    connect(microscope->camera,         SIGNAL(startConnection()),  this, SLOT(cameraStartConnection()));   //相机：开始连接
    connect(microscope->camera,         SIGNAL(noConnection()),     this, SLOT(cameraNoConnection()));      //相机：失去连接
    connect(microscope->camera,         SIGNAL(updateImage()),      this, SLOT(scopeUpdateImage()));        //相机：更新图像
    //左操作臂
    connect(holder->manipulator,        SIGNAL(openPortFailure()),  this, SLOT(lManipOpenPortFailure()));   //左操作臂：打开端口失败
    connect(holder->manipulator,        SIGNAL(startConnection()),  this, SLOT(lManipStartConnection()));   //左操作臂：开始连接
    connect(holder->manipulator,        SIGNAL(noConnection()),     this, SLOT(lManipNoConnection()));      //左操作臂：失去连接
    connect(holder->manipulator,        SIGNAL(updatePos()),        this, SLOT(lManipUpdatePos()));         //左操作臂：更新位置
    //吸持机构
    connect(holder->pSensor,            SIGNAL(openPortFailure()),  this, SLOT(lPSensorOpenPortFailure()));
    connect(holder->pSensor,            SIGNAL(startConnection()),  this, SLOT(lPSensorStartConnection()));
    connect(holder->pSensor,            SIGNAL(noConnection()),     this, SLOT(lPSensorNoConnection()));
    connect(holder->pSensor,            SIGNAL(updatePressure()),   this, SLOT(lPSensorupdatePressure()));
    connect(holder->syringeMotor,       SIGNAL(startConnection()),  this, SLOT(holderMotorStartConnection()));
    connect(holder->syringeMotor,       SIGNAL(updatePos()),        this, SLOT(holderUpdatePos()));

    connect(holder->syringeMotor,       SIGNAL(noConnection()),     this, SLOT(lSyringeMotorNoConnection()));
    //右操作臂
    connect(injector->manipulator,      SIGNAL(openPortFailure()),  this, SLOT(rManipOpenPortFailure()));   //右操作臂：打开端口失败
    connect(injector->manipulator,      SIGNAL(startConnection()),  this, SLOT(rManipStartConnection()));   //右操作臂：开始连接
    connect(injector->manipulator,      SIGNAL(noConnection()),     this, SLOT(rManipNoConnection()));      //右操作臂：失去连接
    connect(injector->manipulator,      SIGNAL(updatePos()),        this, SLOT(rManipUpdatePos()));         //右操作臂：更新位置
    //注射机构
    connect(injector->pSensor,          SIGNAL(openPortFailure()),  this, SLOT(rPSensorOpenPortFailure()));
    connect(injector->pSensor,          SIGNAL(startConnection()),  this, SLOT(rPSensorStartConnection()));
    connect(injector->pSensor,          SIGNAL(noConnection()),     this, SLOT(rPSensorNoConnection()));
    connect(injector->pSensor,          SIGNAL(updatePressure()),   this, SLOT(rPSensorupdatePressure()));
    connect(injector->syringeMotor,     SIGNAL(startConnection()),  this, SLOT(injectorMotorStartConnection()));
    connect(injector->syringeMotor,     SIGNAL(updatePos()),        this, SLOT(injectorUpdatePos()));

    connect(injector->syringeMotor,     SIGNAL(noConnection()),     this, SLOT(rSyringeMotorNoConnection()));

    microscope->camera->moveToThread(&threadCamera);
    microscope->focusdevice->moveToThread(&threadFocus);
    microscope->stage->moveToThread(&threadStage);
    holder->manipulator->moveToThread(&threadLManip);
    holder->pSensor->moveToThread(&threadLPSensor);
    holder->syringeMotor->moveToThread(&threadLSMotor);
    injector->manipulator->moveToThread(&threadRManip);
    injector->pSensor->moveToThread(&threadRPSensor);
    injector->syringeMotor->moveToThread(&threadRSMotor);
    threadCamera.start();
    threadFocus.start();
    threadStage.start();
    threadLManip.start();
    threadLPSensor.start();
    threadLSMotor.start();
    threadRManip.start();
    threadRPSensor.start();
    threadRSMotor.start();

    connect(pServo,             SIGNAL(holdBlow(float, float, bool)),   holder->syringeMotor,   SLOT(blow(float, float, bool)), Qt::BlockingQueuedConnection);
    connect(pServo,             SIGNAL(injSuck(float, float, bool)),    injector->syringeMotor, SLOT(suck(float, float, bool)), Qt::BlockingQueuedConnection);
    connect(pServo,             SIGNAL(stopHoldSyringe()),              holder->syringeMotor,   SLOT(stop()),                    Qt::BlockingQueuedConnection);
    connect(pServo,             SIGNAL(stopInjSyringe()),               injector->syringeMotor,   SLOT(stop()),                    Qt::BlockingQueuedConnection);

    //连接子线程信号槽
    connect(stepBuildMap,       SIGNAL(moveStage(float,float,float)),       microscope->stage,      SLOT(move(float,float,float)),          Qt::BlockingQueuedConnection);
    connect(stepBuildMap,       SIGNAL(moveHold(float,float,float,float)),  holder->manipulator,    SLOT(move(float,float,float,float)),    Qt::BlockingQueuedConnection);
    connect(stepBuildMap,       SIGNAL(moveInj(float,float,float,float)),   injector->manipulator,  SLOT(move(float,float,float,float)),    Qt::BlockingQueuedConnection);
    connect(stepBuildMap,       SIGNAL(setLManipSpeed(float)),          holder->manipulator,    SLOT(setInchingSpeed(float)),   Qt::BlockingQueuedConnection);
    connect(stepBuildMap,       SIGNAL(setRManipSpeed(float)),          injector->manipulator,  SLOT(setInchingSpeed(float)),   Qt::BlockingQueuedConnection);

    connect(stepFetchOocyte,    SIGNAL(moveHold(float,float,float,float)),  holder->manipulator,    SLOT(move(float,float,float,float)),    Qt::BlockingQueuedConnection);
    connect(stepFetchOocyte,    SIGNAL(moveInj(float,float,float,float)),   injector->manipulator,  SLOT(move(float,float,float,float)),    Qt::BlockingQueuedConnection);
    connect(stepFetchOocyte,    SIGNAL(moveStage(float, float, float)), microscope->stage,      SLOT(move(float, float, float)),Qt::BlockingQueuedConnection);
    connect(stepFetchOocyte,    SIGNAL(moveFocus(float, bool, float)),  microscope->focusdevice,SLOT(move(float, bool, float)),Qt::BlockingQueuedConnection);
    connect(stepFetchOocyte,    SIGNAL(gotoArea(cv::Point2f)),          microscope->stage,      SLOT(moveToArea(cv::Point2f)),  Qt::BlockingQueuedConnection);
    connect(stepFetchOocyte,    SIGNAL(holdBlow(float, float, bool)),   holder->syringeMotor,   SLOT(blow(float, float, bool)), Qt::BlockingQueuedConnection);
    connect(stepFetchOocyte,    SIGNAL(funcAdjHoldP(float)),            functions,              SLOT(adjHoldP(float)));
    connect(stepFetchOocyte,    SIGNAL(funcHoldOocyte()),               functions,              SLOT(holdOocyte()),             Qt::BlockingQueuedConnection);

    connect(stepRotateOutPlane, SIGNAL(sgnStartPServo()), pServo, SLOT(start()), Qt::BlockingQueuedConnection);
    connect(stepRotateOutPlane, SIGNAL(sgnStopPServo()), pServo, SLOT(stop()), Qt::BlockingQueuedConnection);
    connect(stepRotateOutPlane, SIGNAL(sgnStartCompare(cv::Mat, cv::Rect)), pServo, SLOT(startCompare(cv::Mat, cv::Rect)), Qt::BlockingQueuedConnection);
    connect(stepRotateOutPlane, SIGNAL(sgnCompareAndCalHoldP(cv::Mat)), pServo, SLOT(compareAndCalHoldP(cv::Mat)), Qt::BlockingQueuedConnection);
    connect(stepRotateOutPlane, SIGNAL(sgnInitTrackerOocyte(cv::Mat, cv::Rect)), track, SLOT(initOocyte(cv::Mat, cv::Rect)), Qt::BlockingQueuedConnection);
    connect(stepRotateOutPlane, SIGNAL(sgnStopTrackOocyte()), track, SLOT(stopOocyte()), Qt::BlockingQueuedConnection);
    connect(stepRotateOutPlane, SIGNAL(moveInj(float,float,float,float)),   injector->manipulator,  SLOT(move(float,float,float,float)),    Qt::BlockingQueuedConnection);
    connect(stepRotateOutPlane, SIGNAL(funcAdjHoldP(float)),            functions,              SLOT(adjHoldP(float)));

    connect(stepRotateInPlane,  SIGNAL(stopAllMotors()), this, SLOT(stopAllMotors()), Qt::BlockingQueuedConnection);
    connect(stepRotateInPlane,  SIGNAL(sgnStartPServo()), pServo, SLOT(start()), Qt::BlockingQueuedConnection);
    connect(stepRotateInPlane,  SIGNAL(sgnStopPServo()), pServo, SLOT(stop()), Qt::BlockingQueuedConnection);
    connect(stepRotateInPlane,  SIGNAL(sgnStartCompare(cv::Mat, cv::Rect)), pServo, SLOT(startCompare(cv::Mat, cv::Rect)), Qt::BlockingQueuedConnection);
    connect(stepRotateInPlane,  SIGNAL(sgnCompareAndCalHoldP(cv::Mat)), pServo, SLOT(compareAndCalHoldP(cv::Mat)), Qt::BlockingQueuedConnection);
    connect(stepRotateInPlane,  SIGNAL(sgnInitTrackerPB(cv::Mat, cv::Rect)), track, SLOT(initPB(cv::Mat, cv::Rect)), Qt::BlockingQueuedConnection);
    connect(stepRotateInPlane,  SIGNAL(sgnInitTrackerOocyte(cv::Mat, cv::Rect)), track, SLOT(initOocyte(cv::Mat, cv::Rect)), Qt::BlockingQueuedConnection);
    connect(stepRotateInPlane,  SIGNAL(sgnStopTrackPB()), track, SLOT(stopPB()), Qt::BlockingQueuedConnection);
    connect(stepRotateInPlane,  SIGNAL(sgnStopTrackOocyte()), track, SLOT(stopOocyte()), Qt::BlockingQueuedConnection);
    connect(stepRotateInPlane,  SIGNAL(moveInj(float,float,float,float)),   injector->manipulator,  SLOT(move(float,float,float,float)),    Qt::BlockingQueuedConnection);
    connect(stepRotateInPlane,  SIGNAL(funcAdjHoldP(float)),            functions,              SLOT(adjHoldP(float)),          Qt::BlockingQueuedConnection);

    connect(stepPenetrate,      SIGNAL(funcAdjHoldP(float)),            functions,              SLOT(adjHoldP(float)));
    connect(stepPenetrate,      SIGNAL(moveInj(float,float,float,float)),   injector->manipulator,  SLOT(move(float,float,float,float)),    Qt::BlockingQueuedConnection);
    connect(stepPenetrate,      SIGNAL(moveInjX(float, float, bool)),   injector->manipulator,  SLOT(moveX(float, float, bool)),Qt::BlockingQueuedConnection);

    connect(stepEnucleation,    SIGNAL(injSuck(float, float, bool)),    injector->syringeMotor, SLOT(suck(float, float, bool)), Qt::BlockingQueuedConnection);
    connect(stepEnucleation,    SIGNAL(gotoArea(cv::Point2f)),          microscope->stage,      SLOT(moveToArea(cv::Point2f)),  Qt::BlockingQueuedConnection);
    connect(stepEnucleation,    SIGNAL(funcAdjInjP(float)),             functions,              SLOT(adjInjP(float)),           Qt::BlockingQueuedConnection);
    connect(stepEnucleation,    SIGNAL(funcWithdrawInj()),              functions,              SLOT(withdrawInj()),            Qt::BlockingQueuedConnection);

    connect(stepExpel,          SIGNAL(funcAdjInjP(float)),             functions,              SLOT(adjInjP(float)),           Qt::BlockingQueuedConnection);
    connect(stepExpel,          SIGNAL(injSuck(float, float, bool)),    injector->syringeMotor, SLOT(suck(float, float, bool)), Qt::BlockingQueuedConnection);
    connect(stepExpel,          SIGNAL(gotoArea(cv::Point2f)),          microscope->stage,      SLOT(moveToArea(cv::Point2f)),  Qt::BlockingQueuedConnection);
    connect(stepExpel,          SIGNAL(funcWithdrawInj()),              functions,              SLOT(withdrawInj()),            Qt::BlockingQueuedConnection);
    connect(stepExpel,          SIGNAL(moveHold(float,float,float,float)),  holder->manipulator,    SLOT(move(float,float,float,float)),    Qt::BlockingQueuedConnection);
    connect(stepExpel,          SIGNAL(moveInj(float,float,float,float)),   injector->manipulator,  SLOT(move(float,float,float,float)),    Qt::BlockingQueuedConnection);

    connect(stepFetchSoma,      SIGNAL(gotoArea(cv::Point2f)),          microscope->stage,      SLOT(moveToArea(cv::Point2f)),  Qt::BlockingQueuedConnection);
    connect(stepFetchSoma,      SIGNAL(moveInjZ(float, float, bool)),   injector->manipulator,  SLOT(moveZ(float, float, bool)),Qt::BlockingQueuedConnection);
    connect(stepFetchSoma,      SIGNAL(moveFocus(float, bool, float)),  microscope->focusdevice,SLOT(move(float, bool, float)),Qt::BlockingQueuedConnection);
    connect(stepFetchSoma,      SIGNAL(moveStage(float, float, float)), microscope->stage,      SLOT(move(float, float, float)),Qt::BlockingQueuedConnection);
    connect(stepFetchSoma,      SIGNAL(moveHold(float,float,float,float)),  holder->manipulator,    SLOT(move(float,float,float,float)),    Qt::BlockingQueuedConnection);
    connect(stepFetchSoma,      SIGNAL(moveInj(float,float,float,float)),   injector->manipulator,  SLOT(move(float,float,float,float)),    Qt::BlockingQueuedConnection);
    connect(stepFetchSoma,      SIGNAL(funcAdjInjP(float)),             functions,              SLOT(adjInjP(float)),           Qt::BlockingQueuedConnection);

    // 批量化处理程序新添加信号槽连接 START
    connect(stepFetchOocyteForSoma,SIGNAL(gotoArea(cv::Point2f)),       microscope->stage,      SLOT(moveToArea(cv::Point2f)),  Qt::BlockingQueuedConnection);
    connect(stepFetchOocyteForSoma,SIGNAL(moveHold(float,float,float,float)),  holder->manipulator,    SLOT(move(float,float,float,float)),    Qt::BlockingQueuedConnection);
    connect(stepFetchOocyteForSoma,SIGNAL(moveInj(float,float,float,float)),   injector->manipulator,  SLOT(move(float,float,float,float)),    Qt::BlockingQueuedConnection);
    connect(stepFetchOocyteForSoma,SIGNAL(moveStage(float, float, float)), microscope->stage,   SLOT(move(float, float, float)),Qt::BlockingQueuedConnection);
    connect(stepFetchOocyteForSoma,SIGNAL(setRManipSpeed(float)),       injector->manipulator,  SLOT(setInchingSpeed(float)),   Qt::BlockingQueuedConnection);
    connect(stepFetchOocyteForSoma,SIGNAL(funcAdjHoldP(float)),         functions,              SLOT(adjHoldP(float)));
    connect(stepFetchOocyteForSoma,SIGNAL(holdBlow(float, float, bool)),holder->syringeMotor,   SLOT(blow(float, float, bool)), Qt::BlockingQueuedConnection);

    connect(stepRotateForSoma,  SIGNAL(stopAllMotors()), this, SLOT(stopAllMotors()), Qt::BlockingQueuedConnection);
    connect(stepRotateForSoma,  SIGNAL(sgnStartPServo()), pServo, SLOT(start()), Qt::BlockingQueuedConnection);
    connect(stepRotateForSoma,  SIGNAL(sgnStopPServo()), pServo, SLOT(stop()), Qt::BlockingQueuedConnection);
    connect(stepRotateForSoma,  SIGNAL(sgnStartCompare(cv::Mat, cv::Rect)), pServo, SLOT(startCompare(cv::Mat, cv::Rect)), Qt::BlockingQueuedConnection);
    connect(stepRotateForSoma,  SIGNAL(sgnCompareAndCalHoldP(cv::Mat)), pServo, SLOT(compareAndCalHoldP(cv::Mat)), Qt::BlockingQueuedConnection);
    connect(stepRotateForSoma,  SIGNAL(sgnInitTrackerPB(cv::Mat, cv::Rect)), track, SLOT(initPB(cv::Mat, cv::Rect)), Qt::BlockingQueuedConnection);
    connect(stepRotateForSoma,  SIGNAL(sgnInitTrackerOocyte(cv::Mat, cv::Rect)), track, SLOT(initOocyte(cv::Mat, cv::Rect)), Qt::BlockingQueuedConnection);
    connect(stepRotateForSoma,  SIGNAL(sgnStopTrackPB()), track, SLOT(stopPB()), Qt::BlockingQueuedConnection);
    connect(stepRotateForSoma,  SIGNAL(sgnStopTrackOocyte()), track, SLOT(stopOocyte()), Qt::BlockingQueuedConnection);
    connect(stepRotateForSoma,  SIGNAL(moveInj(float,float,float,float)),   injector->manipulator,  SLOT(move(float,float,float,float)),    Qt::BlockingQueuedConnection);
    connect(stepRotateForSoma,  SIGNAL(funcAdjHoldP(float)),            functions,              SLOT(adjHoldP(float)),          Qt::BlockingQueuedConnection);

    connect(stepPenetrateForSoma,      SIGNAL(funcAdjHoldP(float)),             functions,              SLOT(adjHoldP(float)));
    connect(stepPenetrateForSoma,      SIGNAL(moveInj(float,float,float,float)),injector->manipulator,  SLOT(move(float,float,float,float)),    Qt::BlockingQueuedConnection);
    connect(stepPenetrateForSoma,      SIGNAL(moveInjX(float, float, bool)),    injector->manipulator,  SLOT(moveX(float, float, bool)),Qt::BlockingQueuedConnection);
    // 批量化处理程序新添加信号槽连接 END

    connect(stepInjectSoma,     SIGNAL(moveInjX(float, float, bool)),   injector->manipulator,  SLOT(moveX(float, float, bool)),Qt::BlockingQueuedConnection);
    connect(stepInjectSoma,     SIGNAL(moveInj(float,float,float,float)),   injector->manipulator,  SLOT(move(float,float,float,float)),    Qt::BlockingQueuedConnection);
    connect(stepInjectSoma,     SIGNAL(funcWithdrawInj()),              functions,              SLOT(withdrawInj()),            Qt::BlockingQueuedConnection);
    connect(stepInjectSoma,     SIGNAL(funcAdjInjP(float)),             functions,              SLOT(adjInjP(float)),           Qt::BlockingQueuedConnection);

    connect(stepReleaseOocyte,  SIGNAL(moveHold(float,float,float,float)),holder->manipulator,  SLOT(move(float,float,float,float)),Qt::BlockingQueuedConnection);
    connect(stepReleaseOocyte,  SIGNAL(moveInj(float,float,float,float)),injector->manipulator, SLOT(move(float,float,float,float)),Qt::BlockingQueuedConnection);
    connect(stepReleaseOocyte,  SIGNAL(gotoArea(cv::Point2f)),          microscope->stage,      SLOT(moveToArea(cv::Point2f)),  Qt::BlockingQueuedConnection);
    connect(stepReleaseOocyte,  SIGNAL(holdBlow(float, float, bool)),   holder->syringeMotor,   SLOT(blow(float, float, bool)), Qt::BlockingQueuedConnection);
    connect(stepReleaseOocyte,  SIGNAL(funcAdjHoldP(float)),            functions,              SLOT(adjHoldP(float)),          Qt::BlockingQueuedConnection);
    connect(stepReleaseOocyte,  SIGNAL(moveStage(float, float, float)), microscope->stage,      SLOT(move(float, float, float)),Qt::BlockingQueuedConnection);

    connect(functions,          SIGNAL(injSuck(float, float, bool)),    injector->syringeMotor, SLOT(suck(float, float, bool)), Qt::BlockingQueuedConnection);
    connect(functions,          SIGNAL(holdBlow(float, float, bool)),   holder->syringeMotor,   SLOT(blow(float, float, bool)), Qt::BlockingQueuedConnection);
    connect(functions,          SIGNAL(moveHold(float, float, float, float)),  holder->manipulator,    SLOT(move(float, float, float, float)),Qt::BlockingQueuedConnection);
    connect(functions,          SIGNAL(moveInjX(float, float, bool)),   injector->manipulator,  SLOT(moveX(float, float, bool)),Qt::BlockingQueuedConnection);
    connect(functions,          SIGNAL(moveHoldZ(float, float, bool)),  holder->manipulator,    SLOT(moveZ(float, float, bool)),Qt::BlockingQueuedConnection);
    connect(functions,          SIGNAL(moveInjZ(float, float, bool)),   injector->manipulator,  SLOT(moveZ(float, float, bool)),Qt::BlockingQueuedConnection);
    connect(functions,          SIGNAL(moveStage(float, float, float)), microscope->stage,      SLOT(move(float, float, float)),Qt::BlockingQueuedConnection);
    connect(functions,          SIGNAL(gotoArea(cv::Point2f)),          microscope->stage,      SLOT(moveToArea(cv::Point2f)),  Qt::BlockingQueuedConnection);
    connect(functions,          SIGNAL(moveFocus(float, bool, float)),  microscope->focusdevice,SLOT(move(float, bool, float)),Qt::BlockingQueuedConnection);
    connect(functions,          SIGNAL(moveInj(float, float, float, float)),   injector->manipulator,  SLOT(move(float, float, float, float)),Qt::BlockingQueuedConnection);

    connect(functions,          SIGNAL(sgnStartPServo()), pServo, SLOT(start()), Qt::BlockingQueuedConnection);
    connect(functions,          SIGNAL(sgnStopPServo()), pServo, SLOT(stop()), Qt::BlockingQueuedConnection);

    connect(functions,          SIGNAL(sgnPlotLineMat(std::vector<double>, std::vector<cv::Mat>, std::vector<cv::Mat>, QString, QString)),
            plots,              SLOT(linePlot2Mat(std::vector<double>, std::vector<cv::Mat>, std::vector<cv::Mat>, QString, QString)),Qt::BlockingQueuedConnection);
    // 设备连接与 ui 点击控制
    connect(this,          SIGNAL(sgnConnectStage(QString)),   microscope->stage,  SLOT(connectDevice(QString)),Qt::BlockingQueuedConnection);
    connect(this,          SIGNAL(sgnConnectCamera(int)),   microscope->camera,  SLOT(connectDevice(int)),Qt::BlockingQueuedConnection);
    connect(this,          SIGNAL(sgnConnectFocus(QString)),   microscope->focusdevice,  SLOT(connectDevice(QString)),Qt::BlockingQueuedConnection);
    connect(this,          SIGNAL(sgnConnectLManip(QString)),   holder->manipulator,  SLOT(connectDevice(QString)),Qt::BlockingQueuedConnection);
    connect(this,          SIGNAL(sgnConnectLPSensor(QString)),   holder->pSensor,  SLOT(connectDevice(QString)),Qt::BlockingQueuedConnection);
    connect(this,          SIGNAL(sgnConnectLSMotor(QString)),   holder->syringeMotor,  SLOT(connectDevice(QString)),Qt::BlockingQueuedConnection);
    connect(this,          SIGNAL(sgnConnectRManip(QString)),   injector->manipulator,  SLOT(connectDevice(QString)),Qt::BlockingQueuedConnection);
    connect(this,          SIGNAL(sgnConnectRPSensor(QString)),   injector->pSensor,  SLOT(connectDevice(QString)),Qt::BlockingQueuedConnection);
    connect(this,          SIGNAL(sgnConnectRSMotor(QString)),   injector->syringeMotor,  SLOT(connectDevice(QString)),Qt::BlockingQueuedConnection);
    connect(this,          SIGNAL(sgnCloseStage()),   microscope->stage,  SLOT(close()),Qt::BlockingQueuedConnection);
    connect(this,          SIGNAL(sgnCloseCamera()),   microscope->camera,  SLOT(close()),Qt::BlockingQueuedConnection);
    connect(this,          SIGNAL(sgnCloseFocus()),   microscope->focusdevice,  SLOT(close()),Qt::BlockingQueuedConnection);
    connect(this,          SIGNAL(sgnCloseLManip()),   holder->manipulator,  SLOT(close()),Qt::BlockingQueuedConnection);
    connect(this,          SIGNAL(sgnCloseLPSensor()),   holder->pSensor,  SLOT(close()),Qt::BlockingQueuedConnection);
    connect(this,          SIGNAL(sgnCloseLSMotor()),   holder->syringeMotor,  SLOT(close()),Qt::BlockingQueuedConnection);
    connect(this,          SIGNAL(sgnCloseRManip()),   injector->manipulator,  SLOT(close()),Qt::BlockingQueuedConnection);
    connect(this,          SIGNAL(sgnCloseRPSensor()),   injector->pSensor,  SLOT(close()),Qt::BlockingQueuedConnection);
    connect(this,          SIGNAL(sgnCloseRSMotor()),   injector->syringeMotor,  SLOT(close()),Qt::BlockingQueuedConnection);

    connect(this,          SIGNAL(sgnGotoArea(cv::Point2f)),          microscope->stage,  SLOT(moveToArea(cv::Point2f)));
    connect(this,          SIGNAL(sgnMoveStageX(float,bool,float)),   microscope->stage,  SLOT(moveX(float,bool,float)),Qt::BlockingQueuedConnection);
    connect(this,          SIGNAL(sgnMoveStageY(float,bool,float)),   microscope->stage,  SLOT(moveY(float,bool,float)),Qt::BlockingQueuedConnection);
    connect(this,          SIGNAL(sgnCaptureFrame(QString)),   microscope->camera,  SLOT(captureFrame(QString)),Qt::BlockingQueuedConnection);
    connect(this,          SIGNAL(sgnStartCameraRecord(QString)),   microscope->camera,  SLOT(startRecord(QString)),Qt::BlockingQueuedConnection);
    connect(this,          SIGNAL(sgnEndCameraRecord()),   microscope->camera,  SLOT(endRecord()),Qt::BlockingQueuedConnection);
    connect(this,          SIGNAL(sgnPauseCameraRecord()),   microscope->camera,  SLOT(pauseRecord()),Qt::BlockingQueuedConnection);
    connect(this,          SIGNAL(sgnContinueCameraRecord()),   microscope->camera,  SLOT(continueRecord()),Qt::BlockingQueuedConnection);
    connect(this,          SIGNAL(sgnMoveFocusZ(float, bool, float)),   microscope->focusdevice,  SLOT(move(float, bool, float)),Qt::BlockingQueuedConnection);
    connect(this,          SIGNAL(sgnMoveLManipX(float, float, bool)),   holder->manipulator,  SLOT(moveX(float, float, bool)),Qt::BlockingQueuedConnection);
    connect(this,          SIGNAL(sgnMoveLManipY(float, float, bool)),   holder->manipulator,  SLOT(moveY(float, float, bool)),Qt::BlockingQueuedConnection);
    connect(this,          SIGNAL(sgnMoveLManipZ(float, float, bool)),   holder->manipulator,  SLOT(moveZ(float, float, bool)),Qt::BlockingQueuedConnection);
    connect(this,          SIGNAL(sgnLSMotorSuck(float, float, bool)),   holder->syringeMotor,  SLOT(suck(float, float, bool)),Qt::BlockingQueuedConnection);
    connect(this,          SIGNAL(sgnLSMotorBlow(float, float, bool)),   holder->syringeMotor,  SLOT(blow(float, float, bool)),Qt::BlockingQueuedConnection);
    connect(this,          SIGNAL(sgnMoveRManipX(float, float, bool)),   injector->manipulator,  SLOT(moveX(float, float, bool)),Qt::BlockingQueuedConnection);
    connect(this,          SIGNAL(sgnMoveRManipY(float, float, bool)),   injector->manipulator,  SLOT(moveY(float, float, bool)),Qt::BlockingQueuedConnection);
    connect(this,          SIGNAL(sgnMoveRManipZ(float, float, bool)),   injector->manipulator,  SLOT(moveZ(float, float, bool)),Qt::BlockingQueuedConnection);
    connect(this,          SIGNAL(sgnRSMotorSuck(float, float, bool)),   injector->syringeMotor,  SLOT(suck(float, float, bool)),Qt::BlockingQueuedConnection);
    connect(this,          SIGNAL(sgnRSMotorBlow(float, float, bool)),   injector->syringeMotor,  SLOT(blow(float, float, bool)),Qt::BlockingQueuedConnection);

    connect(this,          SIGNAL(sgnStopFocus()),   microscope->focusdevice,  SLOT(stop()),Qt::BlockingQueuedConnection);
    connect(this,          SIGNAL(sgnStopStage()),   microscope->stage,  SLOT(stop()),Qt::BlockingQueuedConnection);
    connect(this,          SIGNAL(sgnStopLManip()),  holder->manipulator,  SLOT(stop()),Qt::BlockingQueuedConnection);
    connect(this,          SIGNAL(sgnStopLSMotor()),  holder->syringeMotor,  SLOT(stop()),Qt::BlockingQueuedConnection);
    connect(this,          SIGNAL(sgnStopRManip()),  injector->manipulator,  SLOT(stop()),Qt::BlockingQueuedConnection);
    connect(this,          SIGNAL(sgnStopRSMotor()),  injector->syringeMotor,  SLOT(stop()),Qt::BlockingQueuedConnection);

    //连接设备
    emit sgnConnectStage(strPortStage);
    emit sgnConnectCamera(0);
    emit sgnConnectFocus(strPortFocus);
    emit sgnConnectLManip(strPortLManip);
    emit sgnConnectLPSensor(strPortPHold);
    emit sgnConnectLSMotor(strPortLMotor);
    emit sgnConnectRManip(strPortRManip);
    emit sgnConnectRPSensor(strPortPInj);
    emit sgnConnectRSMotor(strPortRMotor);
}
void RomiolSCNT::modeChanged(QString s)
{
    if(s=="Single")
    {
        mode=SCNT_SINGLE;
        for(auto btn:scnt_batch_group)
            btn->setVisible(false);
        for(auto btn:scnt_single_group)
            btn->setVisible(true);
    }
    else if(s=="Batch")
    {
        mode=SCNT_BATCH;
        for(auto btn:scnt_single_group)
            btn->setVisible(false);
        for(auto btn:scnt_batch_group)
            btn->setVisible(true);
    }
    else if(s=="ICSI")
        mode=ICSI;
}
void RomiolSCNT::btnReset()
{
    isWorking = false;
    //关闭与运动相关的子线程
    // threadFunctions.terminate();
    // threadWorkFlow.terminate();
    stepBuildMap->ON = false;
    stepFetchOocyte->ON = false;
    stepRotateOutPlane->ON = false;
    stepRotateInPlane->ON = false;
    stepPenetrate->ON = false;
    stepEnucleation->ON = false;
    stepFetchSoma->ON = false;
    stepReleaseOocyte->ON = false;
    stepExpel->ON = false;
    stepFetchSoma->ON = false;
    stepFetchOocyteForSoma->ON = false;
    stepRotateForSoma->ON = false;
    stepPenetrateForSoma->ON = false;
    stepInjectSoma->ON = false;
    //停止所有电机的运动
    stopAllMotors();

    holdPipette->reset();
    currentOocyte->reset();
    injectPipette->reset();
    ui->BTN_AUTOCLONE->setStyleSheet(NORMAL_QSS);
    //恢复所有按钮点击功能
    QList<QPushButton*> btns = findChildren<QPushButton*>();//按钮列表
    foreach(QPushButton* btn, btns)
        btn->setEnabled(true);
    // threadFunctions.start();
    // threadWorkFlow.start();
    motorStatus->setText("Reset!");
}
void RomiolSCNT::stopAllMotors()
{
    emit sgnStopFocus();
    emit sgnStopStage();
    emit sgnStopLManip();
    emit sgnStopLSMotor();
    emit sgnStopRManip();
    emit sgnStopRSMotor();
}
// **************************************** 设备操控 ****************************************
// ------------------- 连接设备 -------------------
void RomiolSCNT::connectStage()
{
    if(ui->LED_STAGE->styleSheet()!="background-color: rgb(85, 255, 0);")
        emit sgnConnectStage(strPortStage);
    else
    {
        emit sgnCloseStage();
        ui->LED_STAGE->setStyleSheet("background-color: red;");
    }
}
void RomiolSCNT::connectCamera()
{
    if(ui->BTN_CAMERA->toolTip()=="连接相机")
    {
        motorStatus->setText("连接相机中");
        emit sgnConnectCamera(0);
    }
    else if(ui->BTN_CAMERA->toolTip()=="关闭相机")
    {
        ui->BTN_CAMERA->setIcon(QIcon(":/images/StartRed.png"));
        ui->BTN_CAMERA->setToolTip("连接相机");
        emit sgnCloseCamera();
    }
}
void RomiolSCNT::connectFocusDevice()
{
    if(ui->LED_FOCUSDEVICE->styleSheet()!="background-color: rgb(85, 255, 0);")
        emit sgnConnectFocus(strPortFocus);
    else
    {
        emit sgnCloseFocus();
        ui->LED_FOCUSDEVICE->setStyleSheet("background-color: red;");
    }
}
void RomiolSCNT::connectLManip()
{
    if(ui->LED_LEFT_MANIP->styleSheet()!="background-color: rgb(85, 255, 0);")
        emit sgnConnectLManip(strPortLManip);
    else
    {
        emit sgnCloseLManip();
        ui->LED_LEFT_MANIP->setStyleSheet("background-color: red;");
    }
}
void RomiolSCNT::connectHoldPSensor()
{
    if(ui->LED_HOLDER_PRESSURE->styleSheet()!="background-color: rgb(85, 255, 0);")
        emit sgnConnectLPSensor(strPortPHold);
    else
    {
        emit sgnCloseLPSensor();
        ui->LED_HOLDER_PRESSURE->setStyleSheet("background-color: red;");
    }
}
void RomiolSCNT::connectHolderMotor()
{
    if(ui->LED_HOLDER->styleSheet()!="background-color: rgb(85, 255, 0);")
        emit sgnConnectLSMotor(strPortLMotor);
    else
    {
        emit sgnCloseLSMotor();
        ui->LED_HOLDER->setStyleSheet("background-color: red;");
    }
}
void RomiolSCNT::connectRManip()
{
    if(ui->LED_RIGHT_MANIP->styleSheet()!="background-color: rgb(85, 255, 0);")
        emit sgnConnectRManip(strPortRManip);
    else
    {
        emit sgnCloseRManip();
        ui->LED_RIGHT_MANIP->setStyleSheet("background-color: red;");
    }
}
void RomiolSCNT::connectInjPSensor()
{
    if(ui->LED_INJECTOR_PRESSURE->styleSheet()!="background-color: rgb(85, 255, 0);")
        emit sgnConnectRPSensor(strPortPInj);
    else
    {
        emit sgnCloseRPSensor();
        ui->LED_INJECTOR_PRESSURE->setStyleSheet("background-color: red;");
    }
}
void RomiolSCNT::connectInjectorMotor()
{
    if(ui->LED_INJECTOR->styleSheet()!="background-color: rgb(85, 255, 0);")
        emit sgnConnectRSMotor(strPortRMotor);
    else
    {
        emit sgnCloseRSMotor();
        ui->LED_INJECTOR->setStyleSheet("background-color: red;");
    }
}
// -------------------- 显微镜 --------------------
void RomiolSCNT::btnScopeUp()
{
    bool CONTINUOUS = ui->RADIO__SCOPE_CONTINUOUS->isChecked(); //连动: true; 点动: false
    float displacement = ui->EDIT_SCOPE_DISP->text().toDouble();//点动的距离 [um]
    float vel;                                                  //连动的速度 [um/s]
    if(ui->BTN_SCOPE_CUSTOM_SPEED->isChecked())
        vel = ui->EDIT_SCOPE_SPEED->text().toDouble();
    else if(ui->BTN_SCOPE_HIGH_SPEED->isChecked())
        vel = HIGH_SPEED_FOCUS;
    else if(ui->BTN_SCOPE_MEDIUM_SPEED->isChecked())
        vel = MEDIUM_SPEED_FOCUS;
    else
        vel = LOW_SPEED_FOCUS;

    if(!CONTINUOUS)//点动，只发送一条指令
    {
        emit sgnMoveFocusZ(displacement, CONTINUOUS, vel);
        return;
    }
    QPushButton* theBtn = ui->BTN_SCOPE_UP;//当前按钮
    QString state = theBtn->text();//用于检测状态
    QList<QPushButton*> btns = findChildren<QPushButton*>();//按钮列表，任务进行时禁用其他按钮
    if(state == strStateUp)
    {
        theBtn->setText(strStatePause);
        ui->BTN_SCOPE_UP->      setShortcut(Qt::Key_PageUp);
        foreach(QPushButton* btn, btns){
            if(btn!=theBtn)
                btn->setEnabled(false);
        }
        motorStatus->setText("镜头上移");
        emit sgnMoveFocusZ(displacement, CONTINUOUS, vel);
    }
    else if(state == strStatePause)
    {
        theBtn->setText(strStateUp);
        ui->BTN_SCOPE_UP->      setShortcut(Qt::Key_PageUp);
        foreach(QPushButton* btn, btns){
            if(btn!=theBtn)
                btn->setEnabled(true);
        }
        emit sgnStopFocus();
        motorStatus->setText("");
    }
}
void RomiolSCNT::btnScopeDown()
{
    bool CONTINUOUS = ui->RADIO__SCOPE_CONTINUOUS->isChecked(); //连动: true; 点动: false
    float displacement = ui->EDIT_SCOPE_DISP->text().toDouble();//点动的距离 [um]
    float vel;                                                  //连动的速度 [um/s]
    if(ui->BTN_SCOPE_CUSTOM_SPEED->isChecked())
        vel = ui->EDIT_SCOPE_SPEED->text().toDouble();
    else if(ui->BTN_SCOPE_HIGH_SPEED->isChecked())
        vel = HIGH_SPEED_FOCUS;
    else if(ui->BTN_SCOPE_MEDIUM_SPEED->isChecked())
        vel = MEDIUM_SPEED_FOCUS;
    else
        vel = LOW_SPEED_FOCUS;

    if(!CONTINUOUS)//点动，只发送一条指令
    {
        emit sgnMoveFocusZ(-displacement, CONTINUOUS, vel);
        return;
    }
    QPushButton* theBtn = ui->BTN_SCOPE_DOWN;//当前按钮
    QString state = theBtn->text();//用于检测状态
    QList<QPushButton*> btns = findChildren<QPushButton*>();//按钮列表，任务进行时禁用其他按钮
    if(state == strStateDown)
    {
        theBtn->setText(strStatePause);
        ui->BTN_SCOPE_DOWN->    setShortcut(Qt::Key_PageDown);
        foreach(QPushButton* btn, btns){
            if(btn!=theBtn)
                btn->setEnabled(false);
        }
        motorStatus->setText("镜头下移");
        emit sgnMoveFocusZ(-displacement, CONTINUOUS, -vel);
    }
    else if(state == strStatePause)
    {
        theBtn->setText(strStateDown);
        ui->BTN_SCOPE_DOWN->    setShortcut(Qt::Key_PageDown);
        foreach(QPushButton* btn, btns){
            if(btn!=theBtn)
                btn->setEnabled(true);
        }
        emit sgnStopFocus();
        motorStatus->setText("");
    }
}
void RomiolSCNT::btnScopeLeft()
{
    bool CONTINUOUS = ui->RADIO__SCOPE_CONTINUOUS->isChecked(); //连动: true; 点动: false
    float displacement = ui->EDIT_SCOPE_DISP->text().toDouble();//点动的距离 [um]
    float vel;                                                  //连动的速度 [um/s]
    if(ui->BTN_SCOPE_CUSTOM_SPEED->isChecked())
        vel = ui->EDIT_SCOPE_SPEED->text().toDouble();
    else if(ui->BTN_SCOPE_HIGH_SPEED->isChecked())
        vel = HIGH_SPEED_STAGE;
    else if(ui->BTN_SCOPE_MEDIUM_SPEED->isChecked())
        vel = MEDIUM_SPEED_STAGE;
    else
        vel = LOW_SPEED_STAGE;

    if(!CONTINUOUS)//点动，只发送一条指令
    {
        emit sgnMoveStageX(-displacement, CONTINUOUS, -vel);
        return;
    }
    QPushButton* theBtn = ui->BTN_SCOPE_LEFT;//当前按钮
    QString state = theBtn->text();//用于检测状态
    QList<QPushButton*> btns = findChildren<QPushButton*>();//按钮列表，任务进行时禁用其他按钮
    if(state == strStateLeft)
    {
        theBtn->setText(strStatePause);
        ui->BTN_SCOPE_LEFT->    setShortcut(Qt::Key_Left);
        foreach(QPushButton* btn, btns){
            if(btn!=theBtn)
                btn->setEnabled(false);
        }
        motorStatus->setText("视野左移");
        emit sgnMoveStageX(-displacement, CONTINUOUS, -vel);
    }
    else if(state == strStatePause)
    {
        theBtn->setText(strStateLeft);
        ui->BTN_SCOPE_LEFT->    setShortcut(Qt::Key_Left);
        foreach(QPushButton* btn, btns){
            if(btn!=theBtn)
                btn->setEnabled(true);
        }
        emit sgnStopStage();
        motorStatus->setText("");
    }
}
void RomiolSCNT::btnScopeRight()
{
    bool CONTINUOUS = ui->RADIO__SCOPE_CONTINUOUS->isChecked(); //连动: true; 点动: false
    float displacement = ui->EDIT_SCOPE_DISP->text().toDouble();//点动的距离 [um]
    float vel;                                                  //连动的速度 [um/s]
    if(ui->BTN_SCOPE_CUSTOM_SPEED->isChecked())
        vel = ui->EDIT_SCOPE_SPEED->text().toDouble();
    else if(ui->BTN_SCOPE_HIGH_SPEED->isChecked())
        vel = HIGH_SPEED_STAGE;
    else if(ui->BTN_SCOPE_MEDIUM_SPEED->isChecked())
        vel = MEDIUM_SPEED_STAGE;
    else
        vel = LOW_SPEED_STAGE;

    if(!CONTINUOUS)//点动，只发送一条指令
    {
        emit sgnMoveStageX(displacement, CONTINUOUS, vel);
        return;
    }
    QPushButton* theBtn = ui->BTN_SCOPE_RIGHT;//当前按钮
    QString state = theBtn->text();//用于检测状态
    QList<QPushButton*> btns = findChildren<QPushButton*>();//按钮列表，任务进行时禁用其他按钮
    if(state == strStateRight)
    {
        theBtn->setText(strStatePause);
        ui->BTN_SCOPE_RIGHT->   setShortcut(Qt::Key_Right);
        foreach(QPushButton* btn, btns){
            if(btn!=theBtn)
                btn->setEnabled(false);
        }
        motorStatus->setText("视野右移");
        emit sgnMoveStageX(displacement, CONTINUOUS, vel);
    }
    else if(state == strStatePause)
    {
        theBtn->setText(strStateRight);
        ui->BTN_SCOPE_RIGHT->   setShortcut(Qt::Key_Right);
        foreach(QPushButton* btn, btns){
            if(btn!=theBtn)
                btn->setEnabled(true);
        }
        emit sgnStopStage();
        motorStatus->setText("");
    }
}
void RomiolSCNT::btnScopeForward()
{
    bool CONTINUOUS = ui->RADIO__SCOPE_CONTINUOUS->isChecked(); //连动: true; 点动: false
    float displacement = ui->EDIT_SCOPE_DISP->text().toDouble();//点动的距离 [um]
    float vel;                                                  //连动的速度 [um/s]
    if(ui->BTN_SCOPE_CUSTOM_SPEED->isChecked())
        vel = ui->EDIT_SCOPE_SPEED->text().toDouble();
    else if(ui->BTN_SCOPE_HIGH_SPEED->isChecked())
        vel = HIGH_SPEED_STAGE;
    else if(ui->BTN_SCOPE_MEDIUM_SPEED->isChecked())
        vel = MEDIUM_SPEED_STAGE;
    else
        vel = LOW_SPEED_STAGE;

    if(!CONTINUOUS)//点动，只发送一条指令
    {
        emit sgnMoveStageY(-displacement, CONTINUOUS, -vel);
        return;
    }
    QPushButton* theBtn = ui->BTN_SCOPE_FORWARD;//当前按钮
    QString state = theBtn->text();//用于检测状态
    QList<QPushButton*> btns = findChildren<QPushButton*>();//按钮列表，任务进行时禁用其他按钮
    if(state == strStateForward)
    {
        theBtn->setText(strStatePause);
        ui->BTN_SCOPE_FORWARD-> setShortcut(Qt::Key_Up);
        foreach(QPushButton* btn, btns){
            if(btn!=theBtn)
                btn->setEnabled(false);
        }
        motorStatus->setText("视野前移");
        emit sgnMoveStageY(-displacement, CONTINUOUS, -vel);
    }
    else if(state == strStatePause)
    {
        theBtn->setText(strStateForward);
        ui->BTN_SCOPE_FORWARD-> setShortcut(Qt::Key_Up);
        foreach(QPushButton* btn, btns){
            if(btn!=theBtn)
                btn->setEnabled(true);
        }
        emit sgnStopStage();
        motorStatus->setText("");
    }
}
void RomiolSCNT::btnScopeBack()
{
    bool CONTINUOUS = ui->RADIO__SCOPE_CONTINUOUS->isChecked(); //连动: true; 点动: false
    float displacement = ui->EDIT_SCOPE_DISP->text().toDouble();//点动的距离 [um]
    float vel;                                                  //连动的速度 [um/s]
    if(ui->BTN_SCOPE_CUSTOM_SPEED->isChecked())
        vel = ui->EDIT_SCOPE_SPEED->text().toDouble();
    else if(ui->BTN_SCOPE_HIGH_SPEED->isChecked())
        vel = HIGH_SPEED_STAGE;
    else if(ui->BTN_SCOPE_MEDIUM_SPEED->isChecked())
        vel = MEDIUM_SPEED_STAGE;
    else
        vel = LOW_SPEED_STAGE;

    if(!CONTINUOUS)//点动，只发送一条指令
    {
        emit sgnMoveStageY(displacement, CONTINUOUS, vel);
        return;
    }
    QPushButton* theBtn = ui->BTN_SCOPE_BACK;//当前按钮
    QString state = theBtn->text();//用于检测状态
    QList<QPushButton*> btns = findChildren<QPushButton*>();//按钮列表，任务进行时禁用其他按钮
    if(state == strStateBack)
    {
        theBtn->setText(strStatePause);
        ui->BTN_SCOPE_BACK->    setShortcut(Qt::Key_Down);
        foreach(QPushButton* btn, btns){
            if(btn!=theBtn)
                btn->setEnabled(false);
        }
        motorStatus->setText("视野后移");
        emit sgnMoveStageY(displacement, CONTINUOUS, vel);
    }
    else if(state == strStatePause)
    {
        theBtn->setText(strStateBack);
        ui->BTN_SCOPE_BACK->    setShortcut(Qt::Key_Down);
        foreach(QPushButton* btn, btns){
            if(btn!=theBtn)
                btn->setEnabled(true);
        }
        emit sgnStopStage();
        motorStatus->setText("");
    }
}
//调焦设备
void RomiolSCNT::focusOpenPortFailure()
{
    ui->LED_FOCUSDEVICE->setToolTip("调焦设备：无法打开端口");
    ui->LED_FOCUSDEVICE->setStyleSheet("background-color: red;");
    ui->BTN_SCOPE_UP->setEnabled(false);
    ui->BTN_SCOPE_DOWN->setEnabled(false);
}
void RomiolSCNT::focusNoConnection()
{
    ui->LED_FOCUSDEVICE->setToolTip("调焦设备：连接可能中断");
    ui->LED_FOCUSDEVICE->setStyleSheet("background-color: yellow;");
}
void RomiolSCNT::focusStartConnection()
{
    ui->LED_FOCUSDEVICE->setToolTip("调焦设备：正常");
    ui->LED_FOCUSDEVICE->setStyleSheet("background-color: rgb(85, 255, 0);");
    ui->BTN_SCOPE_UP->setEnabled(true);
    ui->BTN_SCOPE_DOWN->setEnabled(true);
}
void RomiolSCNT::scopeUpdateZ()
{
    if(ui->BTN_POS_TYPE->text()=="ABS")
        ui->LCD_SCOPE_Z->display(microscope->getAbsZ());
    else
        ui->LCD_SCOPE_Z->display(microscope->getRelZ());
}
//平台
void RomiolSCNT::stageOpenPortFailure()
{
    ui->LED_STAGE->setToolTip("平台：无法打开端口");
    ui->LED_STAGE->setStyleSheet("background-color: red;");
    ui->BTN_SCOPE_LEFT->    setEnabled(false);
    ui->BTN_SCOPE_RIGHT->   setEnabled(false);
    ui->BTN_SCOPE_FORWARD-> setEnabled(false);
    ui->BTN_SCOPE_BACK->    setEnabled(false);
}
void RomiolSCNT::stageNoConnection()
{
    ui->LED_STAGE->setToolTip("平台：连接可能中断");
    ui->LED_STAGE->setStyleSheet("background-color: yellow;");
}
void RomiolSCNT::stageStartConnection()
{
    ui->LED_STAGE->setToolTip("平台：正常");
    ui->LED_STAGE->setStyleSheet("background-color: rgb(85, 255, 0);");
    ui->BTN_SCOPE_LEFT->    setEnabled(true);
    ui->BTN_SCOPE_RIGHT->   setEnabled(true);
    ui->BTN_SCOPE_FORWARD-> setEnabled(true);
    ui->BTN_SCOPE_BACK->    setEnabled(true);
    microscope->stage->orix = microscope->getAbsX();
    microscope->stage->oriy = microscope->getAbsY();
}
void RomiolSCNT::scopeUpdateXY()
{
    if(ui->BTN_POS_TYPE->text()=="ABS")
    {
        ui->LCD_SCOPE_X->display(microscope->getAbsX());
        ui->LCD_SCOPE_Y->display(microscope->getAbsY());
    }
    else
    {
        ui->LCD_SCOPE_X->display(microscope->getRelX());
        ui->LCD_SCOPE_Y->display(microscope->getRelY());
    }
}
//相机
void RomiolSCNT::cameraNoConnection()
{
    ui->BTN_CAMERA->setIcon(QIcon(":/images/StartRed.png"));
    ui->BTN_CAMERA->setToolTip("连接相机");
}
void RomiolSCNT::cameraStartConnection()
{
    motorStatus->setText("相机已连接");
    ui->BTN_CAMERA->setIcon(QIcon(":/images/Start.png"));
    ui->BTN_CAMERA->setToolTip("关闭相机");
}
void RomiolSCNT::scopeUpdateImage()
{
    PreGray = CurrentGray.clone();
    //图像源选择：相机/图片序列/视频
    QString source = ui->CMB_IMG_SOURCE->currentText();
    if(source=="相机")
    {
        if(microscope->camera==NULL)
            return;
        CurrentGray = microscope->camera->grayImg.clone();
        CurrentBGR = microscope->camera->bgrImg.clone();
    }
    else if(source=="图片")
    {
        if(!imgSourcePath.empty())
        {
            Mat img = imread(imgSourcePath[0].toStdString());
            if(img.empty())
                return;
            cv::resize(img, img, Size(WIDTH, HEIGHT));
//            imgSourcePath.removeFirst();
            if(img.channels()==1)
            {
                CurrentGray = img.clone();
                cvtColor(img, CurrentBGR, COLOR_GRAY2BGR);
            }
            else if(img.channels()==3)
            {
                CurrentBGR = img.clone();
                cvtColor(img, CurrentGray, COLOR_BGR2GRAY);
            }
            else
                qDebug("image format ERROR");
            microscope->camera->frame = CurrentBGR.clone();
        }
        else
        {
//            playSourceTimer->stop();
//            ui->BTN_PLAY_SOURCE->setToolTip("Off");
//            ui->BTN_PLAY_SOURCE->setStyleSheet("");
//            return;
        }
    }
    else if(source=="视频")
    {
        Mat img;
        if(isPlaying)
            videoCapSource>>img;
        if(!img.empty())
        {
            cv::resize(img, img, Size(WIDTH, HEIGHT));
            if(img.channels()==1)
            {
                CurrentGray = img.clone();
                cvtColor(img, CurrentBGR, COLOR_GRAY2BGR);
            }
            else if(img.channels()==3)
            {
                CurrentBGR = img.clone();
                cvtColor(img, CurrentGray, COLOR_BGR2GRAY);
            }
            else
                qDebug("image format ERROR");
            microscope->camera->frame = CurrentBGR.clone();
        }
    }
    Image = CurrentBGR.clone();
    //比例尺
    if(ui->BTN_SCALAR_BAR->isChecked())
    {
        int len_um = round(10*Pixel2Um)*10;//大约 100 个像素对应的微米数
        int len = len_um/Pixel2Um;
        putText(Image, to_string(len_um)+"um", Point(WIDTH/16, 7*HEIGHT/8),  FONT_HERSHEY_SIMPLEX, 1, 255, 1);
        line(Image, Point(WIDTH/16, 7.3*HEIGHT/8), Point(WIDTH/16+len, 7.3*HEIGHT/8), 255, 3);
    }
    //显示物体信息
    vector<vector<Point>> contours;
    QImage qimg;
    contours.push_back(currentOocyte->contourZP);
    contours.push_back(currentOocyte->contour);
    contours.push_back(holdPipette->contour);
    contours.push_back(injectPipette->contour);
    for(int i=0; i<oocytesInView->oocytes.size(); ++i)
        contours.push_back(oocytesInView->oocytes[i].contour);
    for(int i=0; i<allSomas->oocytes.size(); ++i)
        contours.push_back(allSomas->somas[i].contour);
    if(ui->BTN_CUR_OOCYTE->isChecked() && currentOocyte->isDetected)
    {
        if(!contours[0].empty())
            drawContours(Image, contours, 0, Scalar(200, 30, 200), 1);
        if(!contours[1].empty())
            drawContours(Image, contours, 1, Scalar(255, 0, 255), 2);
//        ellipse(Image, currentOocyte->eZP, Scalar(0, 0, 255), 2);
        for(int i=0; i<currentOocyte->pairs.size(); ++i)
            if(currentOocyte->isPairValid[i])
                line(Image, currentOocyte->innerPoints[currentOocyte->pairs[i].x], currentOocyte->innerPoints[currentOocyte->pairs[i].y], 200*currentOocyte->weights[i]);

        // for(size_t i=0; i<currentOocyte->innerPoints.size(); ++i)
        //     circle(Image, currentOocyte->innerPoints[i], 1, Scalar(50, 50, 255), 2);
    }
    if(ui->BTN_CUR_OOCYTE->isChecked() && currentOocyte->isPBDetected)
        rectangle(Image, currentOocyte->rectPB, 100, 2);
    if(ui->BTN_CUR_SOMA->isChecked() && currentSoma->isDetected)
        rectangle(Image, currentSoma->roi, Scalar(100, 100, 200), 2);
    if(ui->BTN_HOLD_PIPETTE->isChecked() && holdPipette->isDetected && !contours[2].empty())
        drawContours(Image, contours, 2, Scalar(200, 100, 0), 2);
    if(ui->BTN_INJECT_PIPETTE->isChecked() && injectPipette->isDetected && !contours[3].empty())
    {
        drawContours(Image, contours, 3, Scalar(0, 200, 128), 2);
        circle(Image, injectPipette->tip, 3, Scalar(0, 0, 255));
        line(Image, Point(injectPipette->xOCI, injectPipette->ym), Point(injectPipette->xOCI, injectPipette->yM), Scalar(0, 128, 255), 3);
        line(Image, Point(injectPipette->x0OCI, injectPipette->ym), Point(injectPipette->x0OCI, injectPipette->yM), Scalar(0, 0, 255), 2);
        line(Image, Point(injectPipette->xfOCI, injectPipette->ym), Point(injectPipette->xfOCI, injectPipette->yM), Scalar(0, 0, 255), 2);

        line(Image, Point(injectPipette->xGLI, injectPipette->ym), Point(injectPipette->xGLI, injectPipette->yM), Scalar(128, 128, 128), 2);
    }
    if(ui->BTN_OOCYTES->isChecked())
        for(int i=4; i<4+oocytesInView->oocytes.size(); ++i)
            if(contours[i].empty())
                continue;
            else
                drawContours(Image, contours, i, Scalar(0, 0, 255), 1);
    if(ui->BTN_SOMAS->isChecked())
        for(int i=4+oocytesInView->oocytes.size(); i<contours.size(); ++i)
            if(contours[i].empty())
                continue;
            else
                drawContours(Image, contours, i, Scalar(100, 100, 100), 1);
    if(ui->BTN_COLOR->toolTip()=="灰色")
    {
        cvtColor(Image, Image, COLOR_BGR2GRAY);
        qimg = QImage(Image.data, Image.cols,Image.rows,Image.step,QImage::Format_Grayscale8);
    }
    else
        qimg = QImage(Image.data, Image.cols,Image.rows,Image.step,QImage::Format_BGR888);
    ui->LABEL_IMAGE->setPixmap(QPixmap::fromImage(qimg));
}
void RomiolSCNT::btnCapture()
{
    emit sgnCaptureFrame(FILE_PATH);
}
void RomiolSCNT::btnVideo()
{
    if(ui->BTN_VIDEO->toolTip()=="录像")
    {
        emit sgnStartCameraRecord(FILE_PATH);
        ui->BTN_VIDEO->setToolTip("停止录像");
        ui->BTN_PAUSE_RECORD->setEnabled(true);
        ui->BTN_VIDEO->setIcon(QIcon(":/images/VideoRed.png"));

        ui->BTN_PAUSE_RECORD->setToolTip("暂停录像");
        ui->BTN_PAUSE_RECORD->setIcon(QIcon(":/images/Pause.png"));
    }
    else if(ui->BTN_VIDEO->toolTip()=="停止录像")
    {
        emit sgnEndCameraRecord();
        ui->BTN_VIDEO->setToolTip("录像");
        ui->BTN_PAUSE_RECORD->setEnabled(false);
        ui->BTN_VIDEO->setIcon(QIcon(":/images/Video.png"));
    }
}
void RomiolSCNT::btnPauseRecord()
{
    if(ui->BTN_PAUSE_RECORD->toolTip()=="暂停录像")
    {
        emit sgnPauseCameraRecord();
        ui->BTN_PAUSE_RECORD->setToolTip("继续录像");
        ui->BTN_PAUSE_RECORD->setIcon(QIcon(":/images/PauseRed.png"));
    }
    else if(ui->BTN_PAUSE_RECORD->toolTip()=="继续录像")
    {
        emit sgnContinueCameraRecord();
        ui->BTN_PAUSE_RECORD->setToolTip("暂停录像");
        ui->BTN_PAUSE_RECORD->setIcon(QIcon(":/images/Pause.png"));
    }
}
void RomiolSCNT::btnSelPath()
{
    FILE_PATH = QFileDialog::getExistingDirectory(this, tr("保存路径"),
                                                          FILE_PATH,
                                                          QFileDialog::ShowDirsOnly
                                                          | QFileDialog::DontResolveSymlinks);
}
void RomiolSCNT::btnSelImgSource()
{
    if(ui->CMB_IMG_SOURCE->currentText()=="视频")
        imgSourcePath = QFileDialog::getOpenFileNames(this, "选择图像源文件","", "video(*.mp4 *.avi *.wmv)");
    else if(ui->CMB_IMG_SOURCE->currentText()=="图片")
        imgSourcePath = QFileDialog::getOpenFileNames(this, "选择图像源文件","", "image(*.jpg *.png *.bmp)");
    if(ui->CMB_IMG_SOURCE->currentText()=="视频" && !imgSourcePath.empty())
        videoCapSource = VideoCapture(imgSourcePath[0].toStdString());
}
void RomiolSCNT::btnPlaySource()
{
    if(ui->BTN_PLAY_SOURCE->toolTip()=="Off")
    {
        ui->BTN_PLAY_SOURCE->setToolTip("On");
        ui->BTN_PLAY_SOURCE->setStyleSheet("color: rgb(10, 150, 0);");
        playSourceTimer->start(1000/ui->TXT_PLAY_SOURCE_FPS->text().toInt());
        isPlaying = true;
    }
    else
    {
        ui->BTN_PLAY_SOURCE->setToolTip("Off");
        ui->BTN_PLAY_SOURCE->setStyleSheet("");
        isPlaying = false;
    }
}
void RomiolSCNT::imgSourceChanged(QString source)
{
    if(source=="图片")
    {
        imgSourcePath = QFileDialog::getOpenFileNames(this, "选择图像源文件","F:/", "image(*.jpg *.png *.bmp)");
    }
    else if(source=="视频")
    {
        imgSourcePath = QFileDialog::getOpenFileNames(this, "选择图像源文件","F:/", "video(*.mp4 *.avi *.wmv)");
        if(!imgSourcePath.empty())
            videoCapSource = VideoCapture(imgSourcePath[0].toStdString());
    }
}
// --------------------吸持装置--------------------
//左操作臂
void RomiolSCNT::btnLManipUp()
{
    bool CONTINUOUS = ui->RADIO_MANIP_CONTINUOUS->isChecked();//连动: true; 点动: false
    float displacement = ui->EDIT_MANIP_DISP->text().toDouble();//点动的距离 [um]
    float vel = ui->EDIT_MANIP_SPEED->text().toDouble();//连动的速度 [um/s]

    if(!CONTINUOUS)//点动，只发送一条指令
    {
        emit sgnMoveLManipZ(-displacement, -vel, CONTINUOUS);// 操作臂正方向为向下
    }
    else
    {
        QPushButton* theBtn = ui->BTN_LEFT_MANIP_UP;//当前按钮
        QString state = theBtn->text();//用于检测状态
        QList<QPushButton*> btns = findChildren<QPushButton*>();//按钮列表，任务进行时禁用其他按钮
        if(state == strStateUp)
        {
            theBtn->setText(strStatePause);
            foreach(QPushButton* btn, btns){
                if(btn!=theBtn)
                    btn->setEnabled(false);
            }
            motorStatus->setText("左操作臂上移");
            emit sgnMoveLManipZ(-displacement, -vel, CONTINUOUS);// 操作臂正方向为向下
        }
        else if(state == strStatePause)
        {
            theBtn->setText(strStateUp);
            foreach(QPushButton* btn, btns){
                if(btn!=theBtn)
                    btn->setEnabled(true);
            }
            emit sgnStopLManip();
            motorStatus->setText("");
        }
    }
}
void RomiolSCNT::btnLManipDown()
{
    bool CONTINUOUS = ui->RADIO_MANIP_CONTINUOUS->isChecked();//连动: true; 点动: false
    float displacement = ui->EDIT_MANIP_DISP->text().toDouble();//点动的距离 [um]
    float vel = ui->EDIT_MANIP_SPEED->text().toDouble();//连动的速度 [um/s]

    if(!CONTINUOUS)//点动，只发送一条指令
    {
        emit sgnMoveLManipZ(displacement, vel, CONTINUOUS);// 操作臂正方向为向下
    }
    else
    {
        QPushButton* theBtn = ui->BTN_LEFT_MANIP_DOWN;//当前按钮
        QString state = theBtn->text();//用于检测状态
        QList<QPushButton*> btns = findChildren<QPushButton*>();//按钮列表，任务进行时禁用其他按钮
        if(state == strStateDown)
        {
            theBtn->setText(strStatePause);
            foreach(QPushButton* btn, btns){
                if(btn!=theBtn)
                    btn->setEnabled(false);
            }
            motorStatus->setText("左操作臂下移");
            emit sgnMoveLManipZ(displacement, vel, CONTINUOUS);// 操作臂正方向为向下
        }
        else if(state == strStatePause)
        {
            theBtn->setText(strStateDown);
            foreach(QPushButton* btn, btns){
                if(btn!=theBtn)
                    btn->setEnabled(true);
            }
            emit sgnStopLManip();
            motorStatus->setText("");
        }
    }
}
void RomiolSCNT::btnLManipLeft()
{
    bool CONTINUOUS = ui->RADIO_MANIP_CONTINUOUS->isChecked();//连动: true; 点动: false
    float displacement = ui->EDIT_MANIP_DISP->text().toDouble();//点动的距离 [um]
    float vel = ui->EDIT_MANIP_SPEED->text().toDouble();//连动的速度 [um/s]

    if(!CONTINUOUS)//点动，只发送一条指令
    {
        emit sgnMoveLManipX(-displacement, -vel, CONTINUOUS);
        return;
    }
    else
    {
        QPushButton* theBtn = ui->BTN_LEFT_MANIP_LEFT;//当前按钮
        QString state = theBtn->text();//用于检测状态
        QList<QPushButton*> btns = findChildren<QPushButton*>();//按钮列表，任务进行时禁用其他按钮
        if(state == strStateLeft)
        {
            theBtn->setText(strStatePause);
            foreach(QPushButton* btn, btns){
                if(btn!=theBtn)
                    btn->setEnabled(false);
            }
            motorStatus->setText("左操作臂左移");
            emit sgnMoveLManipX(-displacement, -vel, CONTINUOUS);
        }
        else if(state == strStatePause)
        {
            theBtn->setText(strStateLeft);
            foreach(QPushButton* btn, btns){
                if(btn!=theBtn)
                    btn->setEnabled(true);
            }
            emit sgnStopLManip();
            motorStatus->setText("");
        }
    }
}
void RomiolSCNT::btnLManipRight()
{
    bool CONTINUOUS = ui->RADIO_MANIP_CONTINUOUS->isChecked();//连动: true; 点动: false
    float displacement = ui->EDIT_MANIP_DISP->text().toDouble();//点动的距离 [um]
    float vel = ui->EDIT_MANIP_SPEED->text().toDouble();//连动的速度 [um/s]

    if(!CONTINUOUS)//点动，只发送一条指令
    {
        emit sgnMoveLManipX(displacement, vel, CONTINUOUS);
        return;
    }
    else
    {
        QPushButton* theBtn = ui->BTN_LEFT_MANIP_RIGHT;//当前按钮
        QString state = theBtn->text();//用于检测状态
        QList<QPushButton*> btns = findChildren<QPushButton*>();//按钮列表，任务进行时禁用其他按钮
        if(state == strStateRight)
        {
            theBtn->setText(strStatePause);
            foreach(QPushButton* btn, btns){
                if(btn!=theBtn)
                    btn->setEnabled(false);
            }
            motorStatus->setText("左操作臂右移");
            emit sgnMoveLManipX(displacement, vel, CONTINUOUS);
        }
        else if(state == strStatePause)
        {
            theBtn->setText(strStateRight);
            foreach(QPushButton* btn, btns){
                if(btn!=theBtn)
                    btn->setEnabled(true);
            }
            emit sgnStopLManip();
            motorStatus->setText("");
        }
    }
}
void RomiolSCNT::btnLManipForward()
{
    bool CONTINUOUS = ui->RADIO_MANIP_CONTINUOUS->isChecked();//连动: true; 点动: false
    float displacement = ui->EDIT_MANIP_DISP->text().toDouble();//点动的距离 [um]
    float vel = ui->EDIT_MANIP_SPEED->text().toDouble();//连动的速度 [um/s]

    if(!CONTINUOUS)//点动，只发送一条指令
    {
        emit sgnMoveLManipY(-displacement, -vel, CONTINUOUS);
        return;
    }
    else
    {
        QPushButton* theBtn = ui->BTN_LEFT_MANIP_FORWARD;//当前按钮
        QString state = theBtn->text();//用于检测状态
        QList<QPushButton*> btns = findChildren<QPushButton*>();//按钮列表，任务进行时禁用其他按钮
        if(state == strStateForward)
        {
            theBtn->setText(strStatePause);
            foreach(QPushButton* btn, btns){
                if(btn!=theBtn)
                    btn->setEnabled(false);
            }
            motorStatus->setText("左操作臂前移");
            emit sgnMoveLManipY(-displacement, -vel, CONTINUOUS);
        }
        else if(state == strStatePause)
        {
            theBtn->setText(strStateForward);
            foreach(QPushButton* btn, btns){
                if(btn!=theBtn)
                    btn->setEnabled(true);
            }
            emit sgnStopLManip();
            motorStatus->setText("");
        }
    }
}
void RomiolSCNT::btnLManipBack()
{
    bool CONTINUOUS = ui->RADIO_MANIP_CONTINUOUS->isChecked();//连动: true; 点动: false
    float displacement = ui->EDIT_MANIP_DISP->text().toDouble();//点动的距离 [um]
    float vel = ui->EDIT_MANIP_SPEED->text().toDouble();//连动的速度 [um/s]

    if(!CONTINUOUS)//点动，只发送一条指令
    {
        emit sgnMoveLManipY(displacement, vel, CONTINUOUS);
        return;
    }
    else
    {
        QPushButton* theBtn = ui->BTN_LEFT_MANIP_BACK;//当前按钮
        QString state = theBtn->text();//用于检测状态
        QList<QPushButton*> btns = findChildren<QPushButton*>();//按钮列表，任务进行时禁用其他按钮
        if(state == strStateBack)
        {
            theBtn->setText(strStatePause);
            foreach(QPushButton* btn, btns){
                if(btn!=theBtn)
                    btn->setEnabled(false);
            }
            motorStatus->setText("左操作臂后移");
            emit sgnMoveLManipY(displacement, vel, CONTINUOUS);
        }
        else if(state == strStatePause)
        {
            theBtn->setText(strStateBack);
            foreach(QPushButton* btn, btns){
                if(btn!=theBtn)
                    btn->setEnabled(true);
            }
            emit sgnStopLManip();
            motorStatus->setText("");
        }
    }
}
void RomiolSCNT::lManipOpenPortFailure()
{
    ui->LED_LEFT_MANIP->setToolTip("无法打开端口");
    ui->LED_LEFT_MANIP->setStyleSheet("background-color: red;");
    ui->BTN_LEFT_MANIP_UP->setEnabled(false);
    ui->BTN_LEFT_MANIP_DOWN->setEnabled(false);
    ui->BTN_LEFT_MANIP_LEFT->setEnabled(false);
    ui->BTN_LEFT_MANIP_RIGHT->setEnabled(false);
    ui->BTN_LEFT_MANIP_FORWARD->setEnabled(false);
    ui->BTN_LEFT_MANIP_BACK->setEnabled(false);
}
void RomiolSCNT::lManipNoConnection()
{
    ui->LED_LEFT_MANIP->setToolTip("连接可能中断");
    ui->LED_LEFT_MANIP->setStyleSheet("background-color: yellow;");
}
void RomiolSCNT::lManipStartConnection()
{
    ui->LED_LEFT_MANIP->setToolTip("正常");
    ui->LED_LEFT_MANIP->setStyleSheet("background-color: rgb(85, 255, 0);");
    ui->BTN_LEFT_MANIP_UP->setEnabled(true);
    ui->BTN_LEFT_MANIP_DOWN->setEnabled(true);
    ui->BTN_LEFT_MANIP_LEFT->setEnabled(true);
    ui->BTN_LEFT_MANIP_RIGHT->setEnabled(true);
    ui->BTN_LEFT_MANIP_FORWARD->setEnabled(true);
    ui->BTN_LEFT_MANIP_BACK->setEnabled(true);
}
void RomiolSCNT::lManipUpdatePos()
{
    if(ui->BTN_POS_TYPE->text()=="ABS")
    {
        ui->LCD_LEFT_MANIP_X->display(holder->manipulator->getAbsX());
        ui->LCD_LEFT_MANIP_Y->display(holder->manipulator->getAbsY());
        ui->LCD_LEFT_MANIP_Z->display(holder->manipulator->getAbsZ());
    }
    else
    {
        ui->LCD_LEFT_MANIP_X->display(holder->manipulator->getRelX());
        ui->LCD_LEFT_MANIP_Y->display(holder->manipulator->getRelY());
        ui->LCD_LEFT_MANIP_Z->display(holder->manipulator->getRelZ());
    }
}
//气压传感器
void RomiolSCNT::lPSensorOpenPortFailure()
{
    ui->LED_HOLDER_PRESSURE->setToolTip("无法打开端口");
    ui->LED_HOLDER_PRESSURE->setStyleSheet("background-color: red;");
}
void RomiolSCNT::lPSensorNoConnection()
{
    ui->LED_HOLDER_PRESSURE->setToolTip("连接可能中断");
    ui->LED_HOLDER_PRESSURE->setStyleSheet("background-color: yellow;");
}
void RomiolSCNT::lPSensorStartConnection()
{
    ui->LED_HOLDER_PRESSURE->setToolTip("正常");
    ui->LED_HOLDER_PRESSURE->setStyleSheet("background-color: rgb(85, 255, 0);");
}
void RomiolSCNT::lPSensorupdatePressure()
{
    ui->LCD_HOLDER_PRESSURE->display(holder->pSensor->getPressure());
}
//左注射器电机
void RomiolSCNT::holderMotorStartConnection()
{
    ui->LED_HOLDER->setToolTip("正常");
    ui->LED_HOLDER->setStyleSheet("background-color: rgb(85, 255, 0);");
}
void RomiolSCNT::btnHolderSuck()
{
    bool CONTINUOUS = ui->RADIO_SYRINGE_CONTINUOUS->isChecked();//连动: true; 点动: false
    float displacement = ui->EDIT_HOLD_SYRINGE_DISP->text().toDouble();//点动的距离 [um]
    float vel = ui->EDIT_HOLD_SYRINGE_SPEED->text().toDouble();//连动的速度 [um/s]

    if(!CONTINUOUS)//点动，只发送一条指令
        emit sgnLSMotorSuck(displacement, vel, CONTINUOUS);
    else
    {
        QPushButton* theBtn = ui->BTN_HOLDER_SUCK;//当前按钮
        QString state = theBtn->text();//用于检测状态
        QList<QPushButton*> btns = findChildren<QPushButton*>();//按钮列表，任务进行时禁用其他按钮
        if(state == strStateLeft)
        {
            theBtn->setText(strStatePause);
            ui->BTN_HOLDER_SUCK->setShortcut(Qt::Key_Left);
            foreach(QPushButton* btn, btns){
                if(btn!=theBtn)
                    btn->setEnabled(false);
            }
            motorStatus->setText("吸持针吸取");
            emit sgnLSMotorSuck(displacement, vel, CONTINUOUS);
        }
        else if(state == strStatePause)
        {
            theBtn->setText(strStateLeft);
            ui->BTN_HOLDER_SUCK->setShortcut(Qt::Key_Left);
            foreach(QPushButton* btn, btns){
                if(btn!=theBtn)
                    btn->setEnabled(true);
            }
            emit sgnStopLSMotor();
            motorStatus->setText("");
        }
    }
}
void RomiolSCNT::btnHolderBlow()
{
    bool CONTINUOUS = ui->RADIO_SYRINGE_CONTINUOUS->isChecked();//连动: true; 点动: false
    float displacement = ui->EDIT_HOLD_SYRINGE_DISP->text().toDouble();//点动的距离 [um]
    float vel = ui->EDIT_HOLD_SYRINGE_SPEED->text().toDouble();//连动的速度 [um/s]

    if(!CONTINUOUS)//点动，只发送一条指令
        emit sgnLSMotorBlow(displacement, vel, CONTINUOUS);
    else
    {
        QPushButton* theBtn = ui->BTN_HOLDER_BLOW;//当前按钮
        QString state = theBtn->text();//用于检测状态
        QList<QPushButton*> btns = findChildren<QPushButton*>();//按钮列表，任务进行时禁用其他按钮
        if(state == strStateRight)
        {
            theBtn->setText(strStatePause);
            ui->BTN_HOLDER_BLOW->setShortcut(Qt::Key_Right);
            foreach(QPushButton* btn, btns){
                if(btn!=theBtn)
                    btn->setEnabled(false);
            }
            motorStatus->setText("注射器注入");
            emit sgnLSMotorBlow(displacement, vel, CONTINUOUS);
        }
        else if(state == strStatePause)
        {
            theBtn->setText(strStateRight);
            ui->BTN_HOLDER_BLOW->setShortcut(Qt::Key_Right);
            foreach(QPushButton* btn, btns){
                if(btn!=theBtn)
                    btn->setEnabled(true);
            }
            emit sgnStopLSMotor();
            motorStatus->setText("");
        }
    }
}
void RomiolSCNT::holderUpdatePos()
{
    ui->LCD_HOLDER_MOTOR_VEL->display(holder->syringeMotor->getVel());
    if(ui->BTN_POS_TYPE->text()=="ABS")
        ui->LCD_HOLDER->display(holder->syringeMotor->getAbsPos());
    else
        ui->LCD_HOLDER->display(holder->syringeMotor->getRelPos());
}
void RomiolSCNT::lSyringeMotorNoConnection()
{
    ui->LED_HOLDER->setToolTip("连接可能中断");
    ui->LED_HOLDER->setStyleSheet("background-color: yellow;");
}
// --------------------注射装置--------------------
//右操作臂
void RomiolSCNT::btnRManipUp()
{
    bool CONTINUOUS = ui->RADIO_MANIP_CONTINUOUS->isChecked();//连动: true; 点动: false
    float displacement = ui->EDIT_MANIP_DISP->text().toDouble();//点动的距离 [um]
    float vel = ui->EDIT_MANIP_SPEED->text().toDouble();//连动的速度 [um/s]

    if(!CONTINUOUS)//点动，只发送一条指令
        emit sgnMoveRManipZ(-displacement, -vel, CONTINUOUS);// 操作臂正方向为向下
    else
    {
        QPushButton* theBtn = ui->BTN_RIGHT_MANIP_UP;//当前按钮
        QString state = theBtn->text();//用于检测状态
        QList<QPushButton*> btns = findChildren<QPushButton*>();//按钮列表，任务进行时禁用其他按钮
        if(state == strStateUp)
        {
            theBtn->setText(strStatePause);
            foreach(QPushButton* btn, btns){
                if(btn!=theBtn)
                    btn->setEnabled(false);
            }
            motorStatus->setText("右操作臂上移");
            emit sgnMoveRManipZ(-displacement, -vel, CONTINUOUS);// 操作臂正方向为向下
        }
        else if(state == strStatePause)
        {
            theBtn->setText(strStateUp);
            foreach(QPushButton* btn, btns){
                if(btn!=theBtn)
                    btn->setEnabled(true);
            }
            emit sgnStopRManip();
            motorStatus->setText("");
        }
    }
}
void RomiolSCNT::btnRManipDown()
{
    bool CONTINUOUS = ui->RADIO_MANIP_CONTINUOUS->isChecked();//连动: true; 点动: false
    float displacement = ui->EDIT_MANIP_DISP->text().toDouble();//点动的距离 [um]
    float vel = ui->EDIT_MANIP_SPEED->text().toDouble();//连动的速度 [um/s]

    if(!CONTINUOUS)//点动，只发送一条指令
        emit sgnMoveRManipZ(displacement, vel, CONTINUOUS);// 操作臂正方向为向下
    else
    {
        QPushButton* theBtn = ui->BTN_RIGHT_MANIP_DOWN;//当前按钮
        QString state = theBtn->text();//用于检测状态
        QList<QPushButton*> btns = findChildren<QPushButton*>();//按钮列表，任务进行时禁用其他按钮
        if(state == strStateDown)
        {
            theBtn->setText(strStatePause);
            foreach(QPushButton* btn, btns){
                if(btn!=theBtn)
                    btn->setEnabled(false);
            }
            motorStatus->setText("右操作臂下移");
            emit sgnMoveRManipZ(displacement, vel, CONTINUOUS);// 操作臂正方向为向下
        }
        else if(state == strStatePause)
        {
            theBtn->setText(strStateDown);
            foreach(QPushButton* btn, btns){
                if(btn!=theBtn)
                    btn->setEnabled(true);
            }
            emit sgnStopRManip();
            motorStatus->setText("");
        }
    }
}
void RomiolSCNT::btnRManipLeft()
{
    bool CONTINUOUS = ui->RADIO_MANIP_CONTINUOUS->isChecked();//连动: true; 点动: false
    float displacement = ui->EDIT_MANIP_DISP->text().toDouble();//点动的距离 [um]
    float vel = ui->EDIT_MANIP_SPEED->text().toDouble();//连动的速度 [um/s]

    if(!CONTINUOUS)//点动，只发送一条指令
        emit sgnMoveRManipX(-displacement, -vel, CONTINUOUS);
    else
    {
        QPushButton* theBtn = ui->BTN_RIGHT_MANIP_LEFT;//当前按钮
        QString state = theBtn->text();//用于检测状态
        QList<QPushButton*> btns = findChildren<QPushButton*>();//按钮列表，任务进行时禁用其他按钮
        if(state == strStateLeft)
        {
            theBtn->setText(strStatePause);
            foreach(QPushButton* btn, btns){
                if(btn!=theBtn)
                    btn->setEnabled(false);
            }
            motorStatus->setText("右操作臂左移");
            emit sgnMoveRManipX(-displacement, -vel, CONTINUOUS);
        }
        else if(state == strStatePause)
        {
            theBtn->setText(strStateLeft);
            foreach(QPushButton* btn, btns){
                if(btn!=theBtn)
                    btn->setEnabled(true);
            }
            emit sgnStopRManip();
            motorStatus->setText("");
        }
    }
}
void RomiolSCNT::btnRManipRight()
{
    bool CONTINUOUS = ui->RADIO_MANIP_CONTINUOUS->isChecked();//连动: true; 点动: false
    float displacement = ui->EDIT_MANIP_DISP->text().toDouble();//点动的距离 [um]
    float vel = ui->EDIT_MANIP_SPEED->text().toDouble();//连动的速度 [um/s]

    if(!CONTINUOUS)//点动，只发送一条指令
        emit sgnMoveRManipX(displacement, vel, CONTINUOUS);
    else
    {
        QPushButton* theBtn = ui->BTN_RIGHT_MANIP_RIGHT;//当前按钮
        QString state = theBtn->text();//用于检测状态
        QList<QPushButton*> btns = findChildren<QPushButton*>();//按钮列表，任务进行时禁用其他按钮
        if(state == strStateRight)
        {
            theBtn->setText(strStatePause);
            foreach(QPushButton* btn, btns){
                if(btn!=theBtn)
                    btn->setEnabled(false);
            }
            motorStatus->setText("右操作臂右移");
            emit sgnMoveRManipX(displacement, vel, CONTINUOUS);
        }
        else if(state == strStatePause)
        {
            theBtn->setText(strStateRight);
            foreach(QPushButton* btn, btns){
                if(btn!=theBtn)
                    btn->setEnabled(true);
            }
            emit sgnStopRManip();
            motorStatus->setText("");
        }
    }
}
void RomiolSCNT::btnRManipForward()
{
    bool CONTINUOUS = ui->RADIO_MANIP_CONTINUOUS->isChecked();//连动: true; 点动: false
    float displacement = ui->EDIT_MANIP_DISP->text().toDouble();//点动的距离 [um]
    float vel = ui->EDIT_MANIP_SPEED->text().toDouble();//连动的速度 [um/s]

    if(!CONTINUOUS)//点动，只发送一条指令
        emit sgnMoveRManipY(-displacement, -vel, CONTINUOUS);
    else
    {
        QPushButton* theBtn = ui->BTN_RIGHT_MANIP_FORWARD;//当前按钮
        QString state = theBtn->text();//用于检测状态
        QList<QPushButton*> btns = findChildren<QPushButton*>();//按钮列表，任务进行时禁用其他按钮
        if(state == strStateForward)
        {
            theBtn->setText(strStatePause);
            foreach(QPushButton* btn, btns){
                if(btn!=theBtn)
                    btn->setEnabled(false);
            }
            motorStatus->setText("右操作臂前移");
            emit sgnMoveRManipY(-displacement, -vel, CONTINUOUS);
        }
        else if(state == strStatePause)
        {
            theBtn->setText(strStateForward);
            foreach(QPushButton* btn, btns){
                if(btn!=theBtn)
                    btn->setEnabled(true);
            }
            emit sgnStopRManip();
            motorStatus->setText("");
        }
    }
}
void RomiolSCNT::btnRManipBack()
{
    bool CONTINUOUS = ui->RADIO_MANIP_CONTINUOUS->isChecked();//连动: true; 点动: false
    float displacement = ui->EDIT_MANIP_DISP->text().toDouble();//点动的距离 [um]
    float vel = ui->EDIT_MANIP_SPEED->text().toDouble();//连动的速度 [um/s]

    if(!CONTINUOUS)//点动，只发送一条指令
        emit sgnMoveRManipY(displacement, vel, CONTINUOUS);
    else
    {
        QPushButton* theBtn = ui->BTN_RIGHT_MANIP_BACK;//当前按钮
        QString state = theBtn->text();//用于检测状态
        QList<QPushButton*> btns = findChildren<QPushButton*>();//按钮列表，任务进行时禁用其他按钮
        if(state == strStateBack)
        {
            theBtn->setText(strStatePause);
            foreach(QPushButton* btn, btns){
                if(btn!=theBtn)
                    btn->setEnabled(false);
            }
            motorStatus->setText("右操作臂后移");
            emit sgnMoveRManipY(displacement, vel, CONTINUOUS);
        }
        else if(state == strStatePause)
        {
            theBtn->setText(strStateBack);
            foreach(QPushButton* btn, btns){
                if(btn!=theBtn)
                    btn->setEnabled(true);
            }
            emit sgnStopRManip();
            motorStatus->setText("");
        }
    }
}
void RomiolSCNT::rManipOpenPortFailure()
{
    ui->LED_RIGHT_MANIP->setToolTip("无法打开端口");
    ui->LED_RIGHT_MANIP->setStyleSheet("background-color: red;");
    ui->BTN_RIGHT_MANIP_UP->setEnabled(false);
    ui->BTN_RIGHT_MANIP_DOWN->setEnabled(false);
    ui->BTN_RIGHT_MANIP_LEFT->setEnabled(false);
    ui->BTN_RIGHT_MANIP_RIGHT->setEnabled(false);
    ui->BTN_RIGHT_MANIP_FORWARD->setEnabled(false);
    ui->BTN_RIGHT_MANIP_BACK->setEnabled(false);
}
void RomiolSCNT::rManipNoConnection()
{
    ui->LED_RIGHT_MANIP->setToolTip("连接可能中断");
    ui->LED_RIGHT_MANIP->setStyleSheet("background-color: yellow;");
}
void RomiolSCNT::rManipStartConnection()
{
    ui->LED_RIGHT_MANIP->setToolTip("正常");
    ui->LED_RIGHT_MANIP->setStyleSheet("background-color: rgb(85, 255, 0);");
    ui->BTN_RIGHT_MANIP_UP->setEnabled(true);
    ui->BTN_RIGHT_MANIP_DOWN->setEnabled(true);
    ui->BTN_RIGHT_MANIP_LEFT->setEnabled(true);
    ui->BTN_RIGHT_MANIP_RIGHT->setEnabled(true);
    ui->BTN_RIGHT_MANIP_FORWARD->setEnabled(true);
    ui->BTN_RIGHT_MANIP_BACK->setEnabled(true);
}
void RomiolSCNT::rManipUpdatePos()
{
    if(ui->BTN_POS_TYPE->text()=="ABS")
    {
        ui->LCD_RIGHT_MANIP_X->display(injector->manipulator->getAbsX());
        ui->LCD_RIGHT_MANIP_Y->display(injector->manipulator->getAbsY());
        ui->LCD_RIGHT_MANIP_Z->display(injector->manipulator->getAbsZ());
    }
    else
    {
        ui->LCD_RIGHT_MANIP_X->display(injector->manipulator->getRelX());
        ui->LCD_RIGHT_MANIP_Y->display(injector->manipulator->getRelY());
        ui->LCD_RIGHT_MANIP_Z->display(injector->manipulator->getRelZ());
    }
}
//气压传感器
void RomiolSCNT::rPSensorOpenPortFailure()
{
    ui->LED_INJECTOR_PRESSURE->setToolTip("无法打开端口");
    ui->LED_INJECTOR_PRESSURE->setStyleSheet("background-color: red;");
}
void RomiolSCNT::rPSensorNoConnection()
{
    ui->LED_INJECTOR_PRESSURE->setToolTip("连接可能中断");
    ui->LED_INJECTOR_PRESSURE->setStyleSheet("background-color: yellow;");
}
void RomiolSCNT::rPSensorStartConnection()
{
    ui->LED_INJECTOR_PRESSURE->setToolTip("正常");
    ui->LED_INJECTOR_PRESSURE->setStyleSheet("background-color: rgb(85, 255, 0);");
}
void RomiolSCNT::rPSensorupdatePressure()
{
    ui->LCD_INJECTOR_PRESSURE->display(injector->pSensor->getPressure());
}
//注射器电机
void RomiolSCNT::btnInjectorSuck()
{
    bool CONTINUOUS = ui->RADIO_SYRINGE_CONTINUOUS->isChecked();//连动: true; 点动: false
    float displacement = ui->EDIT_INJECT_SYRINGE_DISP->text().toDouble();//点动的距离 [um]
    float vel = ui->EDIT_INJECT_SYRINGE_SPEED->text().toDouble();//连动的速度 [um/s]

    if(!CONTINUOUS)//点动，只发送一条指令
        emit sgnRSMotorSuck(displacement, vel, CONTINUOUS);
    else
    {
        QPushButton* theBtn = ui->BTN_INJECTOR_SUCK;//当前按钮
        QString state = theBtn->text();//用于检测状态
        QList<QPushButton*> btns = findChildren<QPushButton*>();//按钮列表，任务进行时禁用其他按钮
        if(state == strStateRight)
        {
            theBtn->setText(strStatePause);
            ui->BTN_INJECTOR_SUCK->setShortcut(Qt::Key_Right);
            foreach(QPushButton* btn, btns){
                if(btn!=theBtn)
                    btn->setEnabled(false);
            }
            motorStatus->setText("注射器吸取");
            emit sgnRSMotorSuck(displacement, vel, CONTINUOUS);
        }
        else if(state == strStatePause)
        {
            theBtn->setText(strStateRight);
            ui->BTN_INJECTOR_SUCK->setShortcut(Qt::Key_Right);
            foreach(QPushButton* btn, btns){
                if(btn!=theBtn)
                    btn->setEnabled(true);
            }
            emit sgnStopRSMotor();
            motorStatus->setText("");
        }
    }
}
void RomiolSCNT::btnInjectorBlow()
{
    bool CONTINUOUS = ui->RADIO_SYRINGE_CONTINUOUS->isChecked();//连动: true; 点动: false
    float displacement = ui->EDIT_INJECT_SYRINGE_DISP->text().toDouble();//点动的距离 [um]
    float vel = ui->EDIT_INJECT_SYRINGE_SPEED->text().toDouble();//连动的速度 [um/s]

    if(!CONTINUOUS)//点动，只发送一条指令
        emit sgnRSMotorBlow(displacement, vel, CONTINUOUS);
    else
    {
        QPushButton* theBtn = ui->BTN_INJECTOR_BLOW;//当前按钮
        QString state = theBtn->text();//用于检测状态
        QList<QPushButton*> btns = findChildren<QPushButton*>();//按钮列表，任务进行时禁用其他按钮
        if(state == strStateLeft)
        {
            theBtn->setText(strStatePause);
            ui->BTN_INJECTOR_BLOW->setShortcut(Qt::Key_Left);
            foreach(QPushButton* btn, btns){
                if(btn!=theBtn)
                    btn->setEnabled(false);
            }
            motorStatus->setText("注射器注入");
            emit sgnRSMotorBlow(displacement, vel, CONTINUOUS);
        }
        else if(state == strStatePause)
        {
            theBtn->setText(strStateLeft);
            ui->BTN_INJECTOR_BLOW->setShortcut(Qt::Key_Left);
            foreach(QPushButton* btn, btns){
                if(btn!=theBtn)
                    btn->setEnabled(true);
            }
            emit sgnStopRSMotor();
            motorStatus->setText("");
        }
    }
}
void RomiolSCNT::injectorMotorStartConnection()
{
    ui->LED_INJECTOR->setToolTip("正常");
    ui->LED_INJECTOR->setStyleSheet("background-color: rgb(85, 255, 0);");
}
void RomiolSCNT::injectorUpdatePos()
{
    ui->LCD_INJECTOR_MOTOR_VEL->display(injector->syringeMotor->getVel());
    if(ui->BTN_POS_TYPE->text()=="ABS")
        ui->LCD_INJECTOR->display(injector->syringeMotor->getAbsPos());
    else
        ui->LCD_INJECTOR->display(injector->syringeMotor->getRelPos());
}
void RomiolSCNT::rSyringeMotorNoConnection()
{
    ui->LED_INJECTOR->setToolTip("连接可能中断");
    ui->LED_INJECTOR->setStyleSheet("background-color: yellow;");
}

// ************************************* 工作流（多线程） ************************************
void RomiolSCNT::btnStepBuildMap()
{
    if(isWorking)
    {
        workFlowStatus->setText("工作中！请稍候...");
        stepBuildMap->ON = false;
        return;
    }
    workFlowStatus->setText("构建卵细胞区域地图开始");
    GMAP_HEIGHT_UM = ui->TXT_GLOBALMAP_ROWS->text().toInt()*HEIGHT*Pixel2Um;
    GMAP_WIDTH_UM  = ui->TXT_GLOBALMAP_COLS->text().toInt()*WIDTH* Pixel2Um;
    GMAP_HEIGHT = ui->TXT_GLOBALMAP_ROWS->text().toInt()*HEIGHT;
    GMAP_WIDTH  = ui->TXT_GLOBALMAP_COLS->text().toInt()*WIDTH;
    thumbnailTimer->start(50);
    ui->BTN_STEP_BUILD_MAP->setChecked(true);
    //任务初始化
    threadWorkFlow.start();
    emit sgnStepBuildMap();
}
void RomiolSCNT::finishStepBuildMap()
{
    globalMap->isValid = true;
    workFlowStatus->setText("构建卵细胞区域地图完成");
    ui->BTN_STEP_BUILD_MAP->setChecked(false);
    finishStep();
}
void RomiolSCNT::btnStepFetchOocyte()
{
    if(isWorking)
    {
        workFlowStatus->setText("工作中！请稍候...");
        stepFetchOocyte->ON = false;
        return;
    }
    workFlowStatus->setText("换卵细胞开始");
    ui->BTN_STEP_FETCH_OOCYTE->setChecked(true);
    ui->BTN_STEP_FETCH_OOCYTE->setStyleSheet(RUNNING_QSS);
    //任务初始化
    threadWorkFlow.start();
    emit sgnStepFetchOocyte();
}
void RomiolSCNT::finishStepFetchOocyte()
{
    workFlowStatus->setText("换卵细胞完成");
    ui->BTN_STEP_FETCH_OOCYTE->setStyleSheet(FINISHED_QSS);
    ui->BTN_STEP_FETCH_OOCYTE->setChecked(false);
    finishStep();
}
void RomiolSCNT::btnStepRotateOutPlane()
{
    if(isWorking)
    {
        workFlowStatus->setText("工作中！请稍候...");
        stepRotateOutPlane->ON = false;
        return;
    }
    workFlowStatus->setText("平面外旋转卵细胞开始");
    ui->BTN_STEP_ROTATE_OUT_PLANE->setChecked(true);
    ui->BTN_STEP_ROTATE_OUT_PLANE->setStyleSheet(RUNNING_QSS);
    //任务初始化
    threadWorkFlow.start();
    emit sgnStepRotateOutPlane();
}
void RomiolSCNT::finishStepRotateOutPlane()
{
    workFlowStatus->setText("平面外旋转卵细胞完成");
    ui->BTN_STEP_ROTATE_OUT_PLANE->setStyleSheet(FINISHED_QSS);
    ui->BTN_STEP_ROTATE_OUT_PLANE->setChecked(false);
    finishStep();
}
void RomiolSCNT::btnStepRotateInPlane()
{
    if(isWorking)
    {
        workFlowStatus->setText("工作中！请稍候...");
        stepRotateInPlane->ON = false;
        return;
    }
    workFlowStatus->setText("平面内旋转卵细胞开始");
    ui->BTN_STEP_ROTATE_IN_PLANE->setChecked(true);
    ui->BTN_STEP_ROTATE_IN_PLANE->setStyleSheet(RUNNING_QSS);
    //任务初始化
    threadWorkFlow.start();
    emit sgnStepRotateInPlane();
}
void RomiolSCNT::finishStepRotateInPlane()
{
    workFlowStatus->setText("平面内旋转卵细胞完成");
    ui->BTN_STEP_ROTATE_IN_PLANE->setStyleSheet(FINISHED_QSS);
    ui->BTN_STEP_ROTATE_IN_PLANE->setChecked(false);
    finishStep();
}
void RomiolSCNT::btnStepPenetrate()
{
    if(isWorking)
    {
        workFlowStatus->setText("工作中！请稍候...");
        stepPenetrate->ON = false;
        return;
    }
    workFlowStatus->setText("刺入卵细胞开始");
    ui->BTN_STEP_PENETRATE->setChecked(true);
    ui->BTN_STEP_PENETRATE->setStyleSheet(RUNNING_QSS);
    //任务初始化
    threadWorkFlow.start();
    emit sgnStepPenetrate();
}
void RomiolSCNT::finishStepPenetrate()
{
    workFlowStatus->setText("刺入卵细胞完成");
    ui->BTN_STEP_PENETRATE->setStyleSheet(FINISHED_QSS);
    ui->BTN_STEP_PENETRATE->setChecked(false);
    finishStep();
}
void RomiolSCNT::btnStepEnucleation()
{
    if(isWorking)
    {
        workFlowStatus->setText("工作中！请稍候...");
        stepEnucleation->ON = false;
        return;
    }
    workFlowStatus->setText("去核开始");
    ui->BTN_STEP_ENUCLEATION->setChecked(true);
    ui->BTN_STEP_ENUCLEATION->setStyleSheet(RUNNING_QSS);
    //任务初始化
    threadWorkFlow.start();
    emit sgnStepEnucleation();
}
void RomiolSCNT::finishStepEnucleation()
{
    workFlowStatus->setText("去核完成");
    ui->BTN_STEP_ENUCLEATION->setStyleSheet(FINISHED_QSS);
    ui->BTN_STEP_ENUCLEATION->setChecked(false);
    finishStep();
}
void RomiolSCNT::btnStepExpel()
{
    if(isWorking)
    {
        workFlowStatus->setText("工作中！请稍候...");
        stepExpel->ON = false;
        return;
    }
    workFlowStatus->setText("吐核开始");
    ui->BTN_STEP_EXPEL->setChecked(true);
    ui->BTN_STEP_EXPEL->setStyleSheet(RUNNING_QSS);
    //任务初始化
    threadWorkFlow.start();
    emit sgnStepExpel();
}
void RomiolSCNT::finishStepExpel()
{
    workFlowStatus->setText("吐核完成");
    ui->BTN_STEP_EXPEL->setStyleSheet(FINISHED_QSS);
    ui->BTN_STEP_EXPEL->setChecked(false);
    finishStep();
}
void RomiolSCNT::btnStepReleaseOocyte()
{
    if(isWorking)
    {
        workFlowStatus->setText("工作中！请稍候...");
        stepReleaseOocyte->ON = false;
        return;
    }
    workFlowStatus->setText("释放卵细胞开始");
    ui->BTN_STEP_RELEASE_OOCYTE->setChecked(true);
    ui->BTN_STEP_RELEASE_OOCYTE->setStyleSheet(RUNNING_QSS);
    //任务初始化
    threadWorkFlow.start();
    emit sgnStepReleaseOocyte();
}
void RomiolSCNT::finishStepReleaseOocyte()
{
    workFlowStatus->setText("释放卵细胞完成");
    ui->BTN_STEP_RELEASE_OOCYTE->setStyleSheet(FINISHED_QSS);
    ui->BTN_STEP_RELEASE_OOCYTE->setChecked(false);
    finishStep();
}
void RomiolSCNT::btnStepFetchSoma()
{
    if(isWorking)
    {
        workFlowStatus->setText("工作中！请稍候...");
        stepFetchSoma->ON = false;
        return;
    }
    workFlowStatus->setText("取体细胞开始");
    ui->BTN_STEP_FETCH_SOMA->setChecked(true);
    ui->BTN_STEP_FETCH_SOMA->setStyleSheet(RUNNING_QSS);
    //任务初始化
    threadWorkFlow.start();
    emit sgnStepFetchSoma();
}
void RomiolSCNT::finishStepFetchSoma()
{
    workFlowStatus->setText("取体细胞完成");
    ui->BTN_STEP_FETCH_SOMA->setStyleSheet(FINISHED_QSS);
    ui->BTN_STEP_FETCH_SOMA->setChecked(false);
    finishStep();
}
void RomiolSCNT::btnStepInjectSoma()
{
    if(isWorking)
    {
        workFlowStatus->setText("工作中！请稍候...");
        stepInjectSoma->ON = false;
        return;
    }
    workFlowStatus->setText("注入体细胞开始");
    ui->BTN_STEP_INJECT_SOMA->setChecked(true);
    ui->BTN_STEP_INJECT_SOMA->setStyleSheet(RUNNING_QSS);
    //任务初始化
    threadWorkFlow.start();
    emit sgnStepInjectSoma();
}
void RomiolSCNT::finishStepInjectSoma()
{
    workFlowStatus->setText("注入体细胞完成");
    ui->BTN_STEP_INJECT_SOMA->setStyleSheet(FINISHED_QSS);
    ui->BTN_STEP_INJECT_SOMA->setChecked(false);
    finishStep();
}

void RomiolSCNT::btnStepFetchOocyteForSoma()
{
    if(isWorking)
    {
        workFlowStatus->setText("工作中！请稍候...");
        stepFetchOocyteForSoma->ON = false;
        return;
    }
    workFlowStatus->setText("换卵细胞 开始");
    ui->BTN_STEP_FETCH_OOCYTE_FOR_SOMA->setChecked(true);
    ui->BTN_STEP_FETCH_OOCYTE_FOR_SOMA->setStyleSheet(RUNNING_QSS);
    //任务初始化
    threadWorkFlow.start();
    emit sgnStepFetchOocyteForSoma();
}
void RomiolSCNT::finishStepFetchOocyteForSoma()
{
    workFlowStatus->setText("换卵细胞 完成");
    ui->BTN_STEP_FETCH_OOCYTE_FOR_SOMA->setStyleSheet(FINISHED_QSS);
    ui->BTN_STEP_FETCH_OOCYTE_FOR_SOMA->setChecked(false);
    finishStep();
}
void RomiolSCNT::btnStepRotateForSoma()
{
    if(isWorking)
    {
        workFlowStatus->setText("工作中！请稍候...");
        stepRotateForSoma->ON = false;
        return;
    }
    workFlowStatus->setText("旋转卵细胞 开始");
    ui->BTN_STEP_ROTATE_FOR_SOMA->setChecked(true);
    ui->BTN_STEP_ROTATE_FOR_SOMA->setStyleSheet(RUNNING_QSS);
    //任务初始化
    threadWorkFlow.start();
    emit sgnStepRotateForSoma();
}
void RomiolSCNT::finishStepRotateForSoma()
{
    workFlowStatus->setText("旋转卵细胞 完成");
    ui->BTN_STEP_ROTATE_FOR_SOMA->setStyleSheet(FINISHED_QSS);
    ui->BTN_STEP_ROTATE_FOR_SOMA->setChecked(false);
    finishStep();
}
void RomiolSCNT::btnStepPenetrateForSoma()
{
    if(isWorking)
    {
        workFlowStatus->setText("工作中！请稍候...");
        stepPenetrateForSoma->ON = false;
        return;
    }
    workFlowStatus->setText("刺入卵细胞 开始");
    ui->BTN_STEP_PENETRATE_FOR_SOMA->setChecked(true);
    ui->BTN_STEP_PENETRATE_FOR_SOMA->setStyleSheet(RUNNING_QSS);
    //任务初始化
    threadWorkFlow.start();
    emit sgnStepPenetrateForSoma();
}
void RomiolSCNT::finishStepPenetrateForSoma()
{
    workFlowStatus->setText("刺入卵细胞 完成");
    ui->BTN_STEP_PENETRATE_FOR_SOMA->setStyleSheet(FINISHED_QSS);
    ui->BTN_STEP_PENETRATE_FOR_SOMA->setChecked(false);
    finishStep();
}


void RomiolSCNT::finishStep()
{
    for(auto btn:ui->workflowButtons->buttons())
    {
        if(btn->styleSheet()!=FINISHED_QSS)
            btn->setStyleSheet(NORMAL_QSS);
    }
    // stopAllMotors();
    switch (nextStep) {
    case BUILDMAP:
        ui->BTN_STEP_BUILD_MAP->setStyleSheet(READY_QSS);
        ui->BTN_STEP_BUILD_MAP->setChecked(false);
        break;
    case FETCHOOCYTE:
        ui->BTN_STEP_FETCH_OOCYTE->setStyleSheet(READY_QSS);
        ui->BTN_STEP_FETCH_OOCYTE->setChecked(false);
        break;
    case ROTATEOUTPLANE:
        ui->BTN_STEP_ROTATE_OUT_PLANE->setStyleSheet(READY_QSS);
        ui->BTN_STEP_ROTATE_OUT_PLANE->setChecked(false);
        break;
    case ROTATEINPLANE:
        ui->BTN_STEP_ROTATE_IN_PLANE->setStyleSheet(READY_QSS);
        ui->BTN_STEP_ROTATE_IN_PLANE->setChecked(false);
        break;
    case PENETRATE:
        ui->BTN_STEP_PENETRATE->setStyleSheet(READY_QSS);
        ui->BTN_STEP_PENETRATE->setChecked(false);
        break;
    case ENUCLEATION:
        ui->BTN_STEP_ENUCLEATION->setStyleSheet(READY_QSS);
        ui->BTN_STEP_ENUCLEATION->setChecked(false);
        break;
    case EXPEL:
        ui->BTN_STEP_EXPEL->setStyleSheet(READY_QSS);
        ui->BTN_STEP_EXPEL->setChecked(false);
        break;
    case FETCHSOMA:
        ui->BTN_STEP_FETCH_SOMA->setStyleSheet(READY_QSS);
        ui->BTN_STEP_FETCH_SOMA->setChecked(false);
        break;
    case FETCHOOCYTEFORSOMA:
        ui->BTN_STEP_FETCH_OOCYTE_FOR_SOMA->setStyleSheet(READY_QSS);
        ui->BTN_STEP_FETCH_OOCYTE_FOR_SOMA->setChecked(false);
        break;
    case ROTATEFORSOMA:
        ui->BTN_STEP_ROTATE_FOR_SOMA->setStyleSheet(READY_QSS);
        ui->BTN_STEP_ROTATE_FOR_SOMA->setChecked(false);
        break;
    case PENETRATEFORSOMA:
        ui->BTN_STEP_PENETRATE_FOR_SOMA->setStyleSheet(READY_QSS);
        ui->BTN_STEP_PENETRATE_FOR_SOMA->setChecked(false);
        break;
    case INJECTSOMA:
        ui->BTN_STEP_INJECT_SOMA->setStyleSheet(READY_QSS);
        ui->BTN_STEP_INJECT_SOMA->setChecked(false);
        break;
    case RELEASEOOCYTE:
        ui->BTN_STEP_RELEASE_OOCYTE->setStyleSheet(READY_QSS);
        ui->BTN_STEP_RELEASE_OOCYTE->setChecked(false);
        break;
    default:
        break;
    }
}
void RomiolSCNT::btnAutoClone()
{
    GMAP_HEIGHT_UM = ui->TXT_GLOBALMAP_ROWS->text().toInt()*HEIGHT*Pixel2Um;
    GMAP_WIDTH_UM  = ui->TXT_GLOBALMAP_COLS->text().toInt()*WIDTH* Pixel2Um;
    GMAP_HEIGHT = ui->TXT_GLOBALMAP_ROWS->text().toInt()*HEIGHT;
    GMAP_WIDTH  = ui->TXT_GLOBALMAP_COLS->text().toInt()*WIDTH;
    thumbnailTimer->start(50);
    if(ui->BTN_AUTOCLONE->isChecked())
    {
        if(isWorking)
        {
            workFlowStatus->setText("工作中！请稍候...");
            return;
        }
        workFlowStatus->setText("自动化克隆开始");
        ui->BTN_AUTOCLONE->setStyleSheet(RUNNING_QSS);
        threadWorkFlow.start();
        emit sgnAutoClone(nextStep);//开启下一步，进入循环
    }
    else
    {//出现问题时暂停自动化克隆流程，应当尽量避免，terminate可能引起内存错误，因此在各个步骤中应当特别注意变量当前值的有效性
        workFlowStatus->setText("自动化克隆暂停");
        ui->BTN_AUTOCLONE->setStyleSheet(READY_QSS);
        autoClone->ON = false;
        stepBuildMap->ON = false;
        stepFetchOocyte->ON = false;
        stepRotateOutPlane->ON = false;
        stepRotateInPlane->ON = false;
        stepPenetrate->ON = false;
        stepEnucleation->ON = false;
        stepFetchSoma->ON = false;
        stepReleaseOocyte->ON = false;
        stepExpel->ON = false;
        stepFetchSoma->ON = false;
        stepFetchOocyteForSoma->ON = false;
        stepRotateForSoma->ON = false;
        stepPenetrateForSoma->ON = false;
        stepInjectSoma->ON = false;
        // threadWorkFlow.terminate();
        // isWorking = false;
    }
}
void RomiolSCNT::finishAutoClone()
{
    workFlowStatus->setText("自动化克隆完成");
    ui->BTN_AUTOCLONE->setStyleSheet(FINISHED_QSS);
    ui->BTN_AUTOCLONE->setChecked(false);
}
void RomiolSCNT::dealFault(QString msg)
{
    QObject* obj = QObject::sender();
    WorkFlow* worker = qobject_cast<WorkFlow*>(obj);
    QMessageBox* msgBox = new QMessageBox(QMessageBox::Icon::Information, "info", msg,QMessageBox::Yes|QMessageBox::No);//参数 Icon icon, const QString &title, const QString &text,
    msgBox->setModal(false);//设置为非模态
    msgBox->setAttribute(Qt::WA_DeleteOnClose); //关掉消息框后删除指针
    msgBox->move(100, 100);
    msgBox->show();
    connect(msgBox, &QMessageBox::destroyed, [worker](){worker->isDealingFault=false;});
    if(msg=="是否继续对下一个细胞抽核？")
    {
        connect(msgBox, &QMessageBox::accepted, [this](){areAllOocytesEnucleated=false;ui->BTN_FINISH_ENU_OOCYTE->setChecked(false);});
        connect(msgBox, &QMessageBox::rejected, [this](){areAllOocytesEnucleated=true;ui->BTN_FINISH_ENU_OOCYTE->setChecked(true);});
    }
    else if(msg=="是否继续吸取体细胞？")
    {
        connect(msgBox, &QMessageBox::accepted, [this](){areAllSomasFetched=false;ui->BTN_FINISH_FETCH_SOMA->setChecked(false);});
        connect(msgBox, &QMessageBox::rejected, [this](){areAllSomasFetched=true;ui->BTN_FINISH_FETCH_SOMA->setChecked(true);});
    }
    else if(msg=="是否继续对下一个细胞注核？")
    {
        connect(msgBox, &QMessageBox::accepted, [this](){areAllSomasInjected=false;ui->BTN_FINISH_INJ_SOMA->setChecked(false);});
        connect(msgBox, &QMessageBox::rejected, [this](){areAllSomasInjected=true;ui->BTN_FINISH_INJ_SOMA->setChecked(true);});
    }
    else if(msg=="是否结束自动操作？")
        connect(msgBox, &QMessageBox::accepted, [](){nextStep = END;});
    else if(msg=="平面外拨动完成，是否进行平面内拨动")
    {
        connect(msgBox, &QMessageBox::accepted, [](){nextStep=ROTATEINPLANE;});
        connect(msgBox, &QMessageBox::rejected, [](){nextStep=PENETRATE;});
    }
    else if(msg=="换卵是否完成？（去核循环）")
    {
        connect(msgBox, &QMessageBox::accepted, [](){nextStep=ROTATEOUTPLANE;});
        connect(msgBox, &QMessageBox::rejected, [](){nextStep=FETCHOOCYTE;});
    }
    else if(msg=="换卵是否完成？（注核循环）")
    {
        connect(msgBox, &QMessageBox::accepted, [](){nextStep=ROTATEFORSOMA;});
        connect(msgBox, &QMessageBox::rejected, [](){nextStep=FETCHOOCYTEFORSOMA;});
    }
}
void RomiolSCNT::btnRestart()
{
    nextStep = BUILDMAP;
    ui->BTN_STEP_BUILD_MAP->setStyleSheet(READY_QSS);
    ui->BTN_STEP_BUILD_MAP->setChecked(false);
    areAllOocytesEnucleated = false;
    areAllSomasFetched = false;
    areAllSomasInjected = false;
    isWorking = false;
    stepFetchSoma->isReady = false;

    ui->BTN_STEP_BUILD_MAP->setChecked(false);
    ui->BTN_STEP_FETCH_OOCYTE->setStyleSheet(NORMAL_QSS);
    ui->BTN_STEP_ROTATE_OUT_PLANE->setStyleSheet(NORMAL_QSS);
    ui->BTN_STEP_ROTATE_IN_PLANE->setStyleSheet(NORMAL_QSS);
    ui->BTN_STEP_PENETRATE->setStyleSheet(NORMAL_QSS);
    ui->BTN_STEP_ENUCLEATION->setStyleSheet(NORMAL_QSS);
    ui->BTN_STEP_EXPEL->setStyleSheet(NORMAL_QSS);
    ui->BTN_STEP_FETCH_SOMA->setStyleSheet(NORMAL_QSS);
    ui->BTN_STEP_FETCH_OOCYTE_FOR_SOMA->setStyleSheet(NORMAL_QSS);
    ui->BTN_STEP_ROTATE_FOR_SOMA->setStyleSheet(NORMAL_QSS);
    ui->BTN_STEP_PENETRATE_FOR_SOMA->setStyleSheet(NORMAL_QSS);
    ui->BTN_STEP_INJECT_SOMA->setStyleSheet(NORMAL_QSS);
    ui->BTN_STEP_RELEASE_OOCYTE->setStyleSheet(NORMAL_QSS);
    ui->BTN_AUTOCLONE->setStyleSheet(NORMAL_QSS);

    ui->BTN_STEP_BUILD_MAP->setChecked(false);
    ui->BTN_STEP_FETCH_OOCYTE->setChecked(false);
    ui->BTN_STEP_ROTATE_OUT_PLANE->setChecked(false);
    ui->BTN_STEP_ROTATE_IN_PLANE->setChecked(false);
    ui->BTN_STEP_PENETRATE->setChecked(false);
    ui->BTN_STEP_ENUCLEATION->setChecked(false);
    ui->BTN_STEP_EXPEL->setChecked(false);
    ui->BTN_STEP_FETCH_SOMA->setChecked(false);
    ui->BTN_STEP_FETCH_OOCYTE_FOR_SOMA->setChecked(false);
    ui->BTN_STEP_ROTATE_FOR_SOMA->setChecked(false);
    ui->BTN_STEP_PENETRATE_FOR_SOMA->setChecked(false);
    ui->BTN_STEP_INJECT_SOMA->setChecked(false);
    ui->BTN_STEP_RELEASE_OOCYTE->setChecked(false);
    ui->BTN_AUTOCLONE->setChecked(false);
}

void RomiolSCNT::finishFunc(QString str)
{
    if(str=="ADJ_INJ_GLI")
        ui->BTN_ADJ_INJ_GLI->setChecked(false);
    else if(str=="ADJ_HOLD_P")
        ui->BTN_ADJ_HOLD_P->setChecked(false);
    else if(str=="ADJ_INJ_P")
        ui->BTN_ADJ_INJ_P->setChecked(false);
    else if(str=="WITHDRAW_INJ")
        ui->BTN_WITHDRAW_INJ->setChecked(false);
    else if(str=="CO_UP")
        ui->BTN_CO_UP->setChecked(false);
    else if(str=="CO_DOWN")
        ui->BTN_CO_DOWN->setChecked(false);
    else if(str=="NEXT_OOCYTE")
        ui->BTN_NEXT_OOCYTE->setChecked(false);
    else if(str=="FIND_OOCYTE")
        ui->BTN_FIND_OOCYTE->setChecked(false);
    else if(str=="HOLD_OOCYTE")
        ui->BTN_HOLD_OOCYTE->setChecked(false);
    else if(str=="RELEASE_OOCYTE")
        ui->BTN_RELEASE_OOCYTE->setChecked(false);
    else if(str=="FOCUS_OOCYTE")
        ui->BTN_FOCUS_OOCYTE->setChecked(false);
    else if(str=="FOCUS_HOLD")
        ui->BTN_FOCUS_HOLD->setChecked(false);
    else if(str=="FOCUS_INJ")
        ui->BTN_FOCUS_INJ->setChecked(false);
}
void RomiolSCNT::cfgTrajParas(Controller* &controller)
{
    double preTrajMode, curTrajMode;    string strMode;
    QDialog* qdlgChoose = new QDialog(this);    QVBoxLayout* mainLayoutChoose = new QVBoxLayout(qdlgChoose);
    QButtonGroup* groupTraj = new QButtonGroup(this);
    QRadioButton* cBoxSETPOINT = new QRadioButton(this);
    QRadioButton* cBoxSIGMOID = new QRadioButton(this);
    QRadioButton* cBoxSINE = new QRadioButton(this);
    QRadioButton* cBoxACCDEC = new QRadioButton(this);
    QRadioButton* cBoxSLOPE = new QRadioButton(this);
    QPushButton* applyButtonChoose = new QPushButton("Apply");

    QDialog* qdlgPara = new QDialog(this);    vector<QLabel*> labels;    vector<QLineEdit*> lineEdits;    vector<QHBoxLayout *> boxLayouts;
    QVBoxLayout* mainLayoutPara = new QVBoxLayout(qdlgPara);
    QPushButton* applyButtonPara = new QPushButton("Apply");
    QPushButton* cancelButtonPara = new QPushButton("Cancel");

    groupTraj->setExclusive(true);
    cBoxSETPOINT->setText("SETPOINT");
    cBoxSIGMOID->setText("SIGMOID");
    cBoxSINE->setText("SINE");
    cBoxACCDEC->setText("ACCDEC");
    cBoxSLOPE->setText("SLOPE");
    groupTraj->addButton(cBoxSETPOINT);
    groupTraj->addButton(cBoxSIGMOID);
    groupTraj->addButton(cBoxSINE);
    groupTraj->addButton(cBoxACCDEC);
    groupTraj->addButton(cBoxSLOPE);
    mainLayoutChoose->addWidget(cBoxSETPOINT);
    mainLayoutChoose->addWidget(cBoxSIGMOID);
    mainLayoutChoose->addWidget(cBoxSINE);
    mainLayoutChoose->addWidget(cBoxACCDEC);
    mainLayoutChoose->addWidget(cBoxSLOPE);
    mainLayoutChoose->addWidget(applyButtonChoose);
    for(int i=0; i<controller->m; ++i)
    {
        preTrajMode=controller->trajParas[i]["MODE"];      //TODO: 所有维度的轨迹设置不同的模式
        if(preTrajMode==TRAJ_MODE_SETPOINT)
            cBoxSETPOINT->setChecked(true);
        else if(preTrajMode==TRAJ_MODE_SIGMOID)
            cBoxSIGMOID->setChecked(true);
        else if(preTrajMode==TRAJ_MODE_SINE)
            cBoxSINE->setChecked(true);
        else if(preTrajMode==TRAJ_MODE_ACC_DEC)
            cBoxACCDEC->setChecked(true);
        else if(preTrajMode==TRAJ_MODE_SLOPE)
            cBoxSLOPE->setChecked(true);
        connect(applyButtonChoose, &QPushButton::clicked, [&]{
            QString str=groupTraj->checkedButton()->text();
            if(str=="SETPOINT")
                curTrajMode=TRAJ_MODE_SETPOINT;
            else if(str=="SIGMOID")
                curTrajMode=TRAJ_MODE_SIGMOID;
            else if(str=="SINE")
                curTrajMode=TRAJ_MODE_SINE;
            else if(str=="ACCDEC")
                curTrajMode=TRAJ_MODE_ACC_DEC;
            else if(str=="SLOPE")
                curTrajMode=TRAJ_MODE_SLOPE;
            strMode = str.toStdString();
            qdlgChoose->close();
        });
        qdlgChoose->setWindowFlags(windowFlags()|Qt::FramelessWindowHint);
        qdlgChoose->setModal(true);    qdlgChoose->show();    qdlgChoose->exec();
        controller->trajParas[i]["MODE"] = curTrajMode;
        for(int j=0; j<trajMaps[strMode].size(); ++j)
        {
            string paraName = trajMaps[strMode][j];
            double val = 0;
            if(curTrajMode==preTrajMode)
                val = controller->trajParas[i][paraName];
            labels.push_back(new QLabel(QString::fromStdString(paraName)));
            lineEdits.push_back(new QLineEdit(QString::number(val, 'g', 6)));
            boxLayouts.push_back(new QHBoxLayout ());
            boxLayouts.back()->addWidget(labels.back());
            boxLayouts.back()->addWidget(lineEdits.back());
            mainLayoutPara->addLayout(boxLayouts.back());
        }
        mainLayoutPara->addWidget(applyButtonPara);
        mainLayoutPara->addWidget(cancelButtonPara);
        for(int j=0; j<labels.size(); ++j)
        {
            string paraName = labels[j]->text().toStdString();
            connect(lineEdits[j], &QLineEdit::textChanged, [controller, paraName, i](QString str){controller->trajParas[i][paraName] = str.toFloat();});
        }
        if(i==controller->m-1)
            connect(applyButtonPara, &QPushButton::clicked, [controller, qdlgPara]{
                controller->setTrajParas(controller->trajParas);
                qdlgPara->close();
            });
        else
            connect(applyButtonPara, &QPushButton::clicked, [qdlgPara]{
                qdlgPara->close();
            });
        connect(cancelButtonPara, &QPushButton::clicked, [qdlgPara]{qdlgPara->close();});
        qdlgPara->setModal(true);    qdlgPara->show();    qdlgPara->exec();
        for(int j=0; j<labels.size(); ++j)
        {
            labels[j]->setParent(NULL);
            lineEdits[j]->setParent(NULL);
        }
        labels.clear(); lineEdits.clear();  boxLayouts.clear();
    }
    delete qdlgChoose, mainLayoutChoose, groupTraj, cBoxSETPOINT, cBoxSIGMOID, cBoxSINE, cBoxACCDEC, applyButtonChoose, qdlgPara, mainLayoutPara, applyButtonPara, cancelButtonPara;
}
void RomiolSCNT::cfgControlParas(Controller* &controller)
{
    //控制器选择
    QString preControllerName, curControllerName="";
    QDialog* qdlgChoose = new QDialog(this);    QVBoxLayout* mainLayoutChoose = new QVBoxLayout(qdlgChoose);
    //添加新的控制器，需要在下面 6 个地方进行添加
    QButtonGroup* group = new QButtonGroup(this);
    QRadioButton* cBoxPID = new QRadioButton(this);
    QRadioButton* cBoxGPC = new QRadioButton(this);
    QRadioButton* cBoxCGPC = new QRadioButton(this);
    QRadioButton* cBoxFOASMC = new QRadioButton(this);
    QRadioButton* cBoxFOASMC_RBF = new QRadioButton(this);
    QRadioButton* cBoxMRFOASMC_RBF = new QRadioButton(this);
    QRadioButton* cBoxHinfSMCFOI = new QRadioButton(this);
    QRadioButton* cBoxFuzzy_FOASMC_RBF_PI = new QRadioButton(this);
    QRadioButton* cBoxZDT = new QRadioButton(this);
    QRadioButton* cBoxTEST = new QRadioButton(this);
    QRadioButton* cBoxADRC = new QRadioButton(this);
    QRadioButton* cBoxPNEUMATIC = new QRadioButton(this);
    QPushButton* applyButtonChoose = new QPushButton("Apply");
    cBoxPID->setText("PID");
    cBoxGPC->setText("GPC");
    cBoxCGPC->setText("CGPC");
    cBoxFOASMC->setText("FOASMC");
    cBoxFOASMC_RBF->setText("FOASMC_RBF");
    cBoxMRFOASMC_RBF->setText("MRFOASMC_RBF");
    cBoxHinfSMCFOI->setText("HinfSMCFOI");
    cBoxFuzzy_FOASMC_RBF_PI->setText("Fuzzy_FOASMC_RBF_PI");
    cBoxZDT->setText("ZDT");
    cBoxTEST->setText("TEST");
    cBoxADRC->setText("ADRC");
    cBoxPNEUMATIC->setText("PNEUMATIC");
    group->setExclusive(true);
    group->addButton(cBoxPID);
    group->addButton(cBoxGPC);
    group->addButton(cBoxCGPC);
    group->addButton(cBoxFOASMC);
    group->addButton(cBoxFOASMC_RBF);
    group->addButton(cBoxMRFOASMC_RBF);
    group->addButton(cBoxHinfSMCFOI);
    group->addButton(cBoxFuzzy_FOASMC_RBF_PI);
    group->addButton(cBoxZDT);
    group->addButton(cBoxTEST);
    group->addButton(cBoxADRC);
    group->addButton(cBoxPNEUMATIC);
    mainLayoutChoose->addWidget(cBoxPID);
    mainLayoutChoose->addWidget(cBoxGPC);
    mainLayoutChoose->addWidget(cBoxCGPC);
    mainLayoutChoose->addWidget(cBoxFOASMC);
    mainLayoutChoose->addWidget(cBoxFOASMC_RBF);
    mainLayoutChoose->addWidget(cBoxMRFOASMC_RBF);
    mainLayoutChoose->addWidget(cBoxHinfSMCFOI);
    mainLayoutChoose->addWidget(cBoxFuzzy_FOASMC_RBF_PI);
    mainLayoutChoose->addWidget(cBoxZDT);
    mainLayoutChoose->addWidget(cBoxTEST);
    mainLayoutChoose->addWidget(cBoxADRC);
    mainLayoutChoose->addWidget(cBoxPNEUMATIC);
    mainLayoutChoose->addWidget(applyButtonChoose);
    if(controller->name=="FOASMC")
    {
        preControllerName="FOASMC";
        cBoxFOASMC->setChecked(true);
    }
    else if(controller->name=="PID")
    {
        preControllerName="PID";
        cBoxPID->setChecked(true);
    }
    else if(controller->name=="GPC")
    {
        preControllerName="GPC";
        cBoxGPC->setChecked(true);
    }
    else if(controller->name=="CGPC")
    {
        preControllerName="CGPC";
        cBoxCGPC->setChecked(true);
    }
    else if(controller->name=="FOASMC_RBF")
    {
        preControllerName="FOASMC_RBF";
        cBoxFOASMC_RBF->setChecked(true);
    }
    else if(controller->name=="MRFOASMC_RBF")
    {
        preControllerName="MRFOASMC_RBF";
        cBoxMRFOASMC_RBF->setChecked(true);
    }
    else if(controller->name=="HinfSMCFOI")
    {
        preControllerName="HinfSMCFOI";
        cBoxHinfSMCFOI->setChecked(true);
    }
    else if(controller->name=="Fuzzy_FOASMC_RBF_PI")
    {
        preControllerName="Fuzzy_FOASMC_RBF_PI";
        cBoxFuzzy_FOASMC_RBF_PI->setChecked(true);
    }
    else if(controller->name=="ZDT")
    {
        preControllerName="ZDT";
        cBoxZDT->setChecked(true);
    }
    else if(controller->name=="TEST")
    {
        preControllerName="TEST";
        cBoxTEST->setChecked(true);
    }
    else if(controller->name=="ADRC")
    {
        preControllerName="ADRC";
        cBoxADRC->setChecked(true);
    }
    else if(controller->name=="PNEUMATIC")
    {
        preControllerName="PNEUMATIC";
        cBoxPNEUMATIC->setChecked(true);
    }
    connect(applyButtonChoose, &QPushButton::clicked, [&]{
        curControllerName = group->checkedButton()->text();
        qdlgChoose->close();
    });
    qdlgChoose->setWindowFlags(windowFlags()|Qt::FramelessWindowHint);
    qdlgChoose->setModal(true);    qdlgChoose->show();    qdlgChoose->exec();


    QDialog* qdlgPara = new QDialog(this);
    vector<QLabel*> labels;
    vector<QLineEdit*> lineEdits;
    vector<QHBoxLayout *> boxLayouts;
    QVBoxLayout* mainLayout = new QVBoxLayout(qdlgPara);
    QPushButton* applyButton = new QPushButton("Apply");
    QPushButton* cancelButton = new QPushButton("Cancel");

    map<string, QVariant> qParas;
    QString sValue;

    //设置新的控制器
    if(curControllerName!=preControllerName)
    {
        vector<map<string, double>> trajParas;
        trajParas.assign(controller->trajParas.begin(), controller->trajParas.end());
        delete controller;
        if(curControllerName=="FOASMC")
            controller = new Controller_FOASMC;
        else if(curControllerName=="PID")
            controller = new Controller_PID;
        else if(curControllerName=="GPC")
            controller = new Controller_GPC;
        else if(curControllerName=="CGPC")
            controller = new Controller_CGPC(1,2);
        else if(curControllerName=="FOASMC_RBF")
            controller = new Controller_FOASMC_RBF;
        else if(curControllerName=="MRFOASMC_RBF")
            controller = new Controller_MRFOASMC_RBF;
        else if(curControllerName=="HinfSMCFOI")
            controller = new Controller_HinfSMCFOI;
        else if(curControllerName=="Fuzzy_FOASMC_RBF_PI")
            controller = new Controller_FUZZY_FOASMC_RBF_PI;
        else if(curControllerName=="ZDT")
            controller = new Controller_ZDT;
        else if(curControllerName=="TEST")
            controller = new Controller_TEST;
        else if(curControllerName=="ADRC")
            controller = new Controller_ADRC;
        else if(curControllerName=="PNEUMATIC")
            controller = new Controller_PNEUMATIC;
        controller->setTrajParas(trajParas);
    }
    qParas = controller->qParas;
    for(auto &t : qParas)
    {
        labels.push_back(new QLabel(t.first.c_str()));
        // 行输入控件
        QString sType = t.second.typeName();
        QString tmp = t.second.toString();
        if(sType=="QList<double>")
        {
            auto qlist = t.second.toList();
            sValue = QString("[");
            for(int i=0; i<qlist.size()-1; ++i)
                sValue = sValue + QString::number(qlist[i].toFloat(), 'g', 6) + QString(" ");
            sValue = sValue + QString::number(qlist.back().toFloat(), 'g', 6) + QString("]");
            lineEdits.push_back(new QLineEdit(sValue));
        }
        else if(sType=="QString")
            lineEdits.push_back(new QLineEdit(t.second.toString()));
        else if(sType=="bool" || sType=="int")
            lineEdits.push_back(new QLineEdit(t.second.toString()));
        else// if(sType=="double")
            lineEdits.push_back(new QLineEdit(QString::number(t.second.toFloat(), 'g', 6)));
        // 水平布局控件
        boxLayouts.push_back(new QHBoxLayout ());
        boxLayouts.back()->addWidget(labels.back());
        boxLayouts.back()->addWidget(lineEdits.back());
        mainLayout->addLayout(boxLayouts.back());
    }
    mainLayout->addWidget(applyButton);
    mainLayout->addWidget(cancelButton);
    for(int i=0; i<lineEdits.size(); ++i)
        connect(lineEdits[i], &QLineEdit::textChanged, [controller, labels, i](QString str){
            QString sType = controller->qParas[labels[i]->text().toStdString()].typeName();
            if(sType=="QList<double>")
            {
                QStringList sList;
                QVariant qVar;
                QList<double> qList;
                str.remove(0, 1);                str.chop(1);
                sList = str.split(QRegularExpression("\t|\n|\r\n|\r| |,|;"));
                for(qsizetype i=0; i<sList.size(); ++i)
                    qList.push_back(sList[i].toFloat());
                qVar.setValue(qList);
                controller->qParas[labels[i]->text().toStdString()] = qVar;
            }
            else if(sType=="QString")
                controller->qParas[labels[i]->text().toStdString()] = str;
            else if(sType=="bool")
                controller->qParas[labels[i]->text().toStdString()] = QVariant(str).toBool();
            else if(sType=="int")
                controller->qParas[labels[i]->text().toStdString()] = str.toInt();
            else// if(sType=="double")
                controller->qParas[labels[i]->text().toStdString()]=str.toFloat();
        });
    connect(applyButton, &QPushButton::clicked, [controller, qdlgPara]{
        controller->setParas(controller->qParas);
        qdlgPara->close();
    });

    connect(cancelButton, &QPushButton::clicked, [qdlgPara]{qdlgPara->close();});
    qdlgPara->setModal(true);
    qdlgPara->show();
    qdlgPara->exec();
}

void RomiolSCNT::btnObjectDetector()
{
    currentOocyte->reset();
    holdPipette  ->reset();
    injectPipette->reset();
    oocytesInView->reset();
    allSomas     ->reset();
    if(CurrentGray.empty())
        return;

    Mat bgrImg;
    map<string, vector<Rect>> results;
    vector<Rect> oocytesRects, holdRects, injectRects, ociRects, gliRects, pbRects, somaRects;
    vector<Rect> curOocyRects, curHoldRects, curInjRects, curPbRects;

    bgrImg = CurrentBGR.clone();
    if(isUsingSocket)
        results = objectDetector->getRects(bgrImg);
    else
        results = yolo->getRects(bgrImg);

    holdRects = results["Holder"];
    injectRects = results["Injector"];
    oocytesRects = results["Oocyte"];
    pbRects = results["Polarbody"];
    pbRects.insert(pbRects.end(), results["polar_body"].begin(), results["polar_body"].end());
    ociRects = results["Cytoplasm-Interface"];
    gliRects = results["GL-Interface"];
    holdRects.insert(holdRects.end(), injectRects.begin(), injectRects.end()); // TODO: 吸持针误检为注射针
    somaRects = results["Soma"];

    curOocyRects = results["oocyte_current"];
    curHoldRects = results["holder_current"];
    curInjRects = results["injector_current"];
    curPbRects = results["polarbody_current"];

    holdPipette->find(bgrImg, curHoldRects, false);
    currentOocyte->find(bgrImg, curOocyRects, false);
    currentOocyte->findPB(bgrImg, curPbRects, false);// TODO: pbRects
    injectPipette->find(bgrImg, curInjRects, false);
    injectPipette->findOCI(bgrImg,0,ociRects, false);
    injectPipette->findGLI(bgrImg,gliRects, false);
    oocytesInView->find(bgrImg, oocytesRects, false);
    allSomas->find(bgrImg, somaRects, false);

    // allOocytes->find(bgrImg, cv::Point2f(microscope->getRelX(), microscope->getRelY()));
    // allSomas->find(bgrImg);

    // injectPipette->detectGasColumn(bgrImg);
    // injectPipette->detectBubble(CurrentGray);
    // if(currentSoma->isDetected)
    //     currentSoma->track(CurrentGray);
    // currentSoma->find(CurrentGray.clone());
    // injectPipette->findGLI(bgrImg);
    // injectPipette->findSoma(bgrImg);
}
// **************************************** 辅助功能 ****************************************
void RomiolSCNT::btnPosType()               //改变位置显示格式：绝对/相对
{
    if(ui->BTN_POS_TYPE->text()=="ABS")
    {
        ui->BTN_POS_TYPE->setText("REL");
        ui->BTN_POS_TYPE->setToolTip("相对位置");
    }
    else
    {
        ui->BTN_POS_TYPE->setText("ABS");
        ui->BTN_POS_TYPE->setToolTip("绝对位置");
    }
}
void RomiolSCNT::btnSetOrigin()             //设置当前位置为坐标原点
{
    microscope->stage->orix = microscope->getAbsX();
    microscope->stage->oriy = microscope->getAbsY();
    microscope->oriz = microscope->getAbsZ();
    holder->manipulator->orix = holder->manipulator->getAbsX();
    holder->manipulator->oriy = holder->manipulator->getAbsY();
    holder->manipulator->oriz = holder->manipulator->getAbsZ();
    holder->syringeMotor->ori = holder->syringeMotor->getAbsPos();
    injector->manipulator->orix = injector->manipulator->getAbsX();
    injector->manipulator->oriy = injector->manipulator->getAbsY();
    injector->manipulator->oriz = injector->manipulator->getAbsZ();
    injector->syringeMotor->ori = injector->syringeMotor->getAbsPos();
}
void RomiolSCNT::btnColor()                 //图像显示颜色切换 彩色/灰色
{

    if(this->ui->BTN_COLOR->toolTip()=="彩色"){this->ui->BTN_COLOR->setToolTip("灰色"); this->ui->BTN_COLOR->setIcon(QIcon(":/images/Gray.png"));}
    else{this->ui->BTN_COLOR->setToolTip("彩色"); this->ui->BTN_COLOR->setIcon(QIcon(":/images/Colorful.png"));}
}
void RomiolSCNT::btnShowAll()
{
    if(ui->BTN_SHOW_ALL->isChecked())
    {
        ui->BTN_CUR_OOCYTE->setChecked(true);
        ui->BTN_CUR_SOMA->setChecked(true);
        ui->BTN_HOLD_PIPETTE->setChecked(true);
        ui->BTN_INJECT_PIPETTE->setChecked(true);
        ui->BTN_OOCYTES->setChecked(true);
        ui->BTN_SOMAS->setChecked(true);
    }
    else
    {
        ui->BTN_CUR_OOCYTE->setChecked(false);
        ui->BTN_CUR_SOMA->setChecked(false);
        ui->BTN_HOLD_PIPETTE->setChecked(false);
        ui->BTN_INJECT_PIPETTE->setChecked(false);
        ui->BTN_OOCYTES->setChecked(false);
        ui->BTN_SOMAS->setChecked(false);
    }
}

void RomiolSCNT::recordData()
{
    injectPipette->findOCI(CurrentGray, currentOocyte->center.x);
    currentOocyte->calcStrain(CurrentGray);
    injectPipette->findGLI(CurrentGray);
    recordfile<<injector->pSensor->pressure<<"\t"
               <<injector->syringeMotor->getRelPos()<<"\t"
               <<injector->syringeMotor->raw_vel<<"\t"
               <<(injectPipette->xOCI-injectPipette->x0OCI)*Pixel2Um<<"\t"
               <<currentOocyte->strain<<"\t"
               <<injectPipette->xGLI<<endl;

    //Create picture save pathname
    // QDateTime dateTime(QDateTime::currentDateTime());
    // QString time = dateTime.toString("hh-mm-ss-zzz");
    // QString filename = QString("/%1.jpg").arg(time);
    // QString full_path = datasetPath + filename;
    // cv::imwrite(full_path.toStdString(), CurrentBGR);
    // recordfile<<QString("/%1.jpg").arg(time).toStdString()
    //            <<microscope->getAbsX()<<"\t"<<microscope->getAbsY()<<"\t"<<microscope->getAbsZ()<<"\t"
    //            <<holder->manipulator->getAbsX()<<"\t"<<holder->manipulator->getAbsY()<<"\t"<<holder->manipulator->getAbsZ()<<"\t"
    //            <<injector->manipulator->getAbsX()<<"\t"<<injector->manipulator->getAbsY()<<"\t"<<injector->manipulator->getAbsZ()<<"\t"<<endl;
}

void RomiolSCNT::updateThumbnail()          //更新显示全局缩略图
{
    REGION_THUMB_WIDTH  = WIDTH_UM* GTHUMB_WIDTH /GMAP_WIDTH_UM;
    REGION_THUMB_HEIGHT = HEIGHT_UM*GTHUMB_HEIGHT/GMAP_HEIGHT_UM;
    REGION_OX_UM = microscope->getRelX()-WIDTH_UM/2 - (oocyteArea.x-WIDTH* globalMap->pixel2Um/2);
    REGION_OY_UM = microscope->getRelY()-HEIGHT_UM/2- (oocyteArea.y-HEIGHT*globalMap->pixel2Um/2);
    REGION_THUMB_OX = REGION_OX_UM*GTHUMB_WIDTH /GMAP_WIDTH_UM;
    REGION_THUMB_OY = REGION_OY_UM*GTHUMB_HEIGHT/GMAP_HEIGHT_UM;
    cv::Mat thumbnailGlobalMap = cv::Mat(Size(GTHUMB_WIDTH, GTHUMB_HEIGHT), CV_8UC1);
    cv::resize(globalMap->mapImage, thumbnailGlobalMap, Size(GTHUMB_WIDTH, GTHUMB_HEIGHT));

    for(int x=0; x<thumbnailGlobalMap.cols; ++x)
        for(int y=0; y<thumbnailGlobalMap.rows; ++y)
            if(x>=REGION_THUMB_OX && x-REGION_THUMB_OX<= REGION_THUMB_WIDTH && y>=REGION_THUMB_OY && y-REGION_THUMB_OY<= REGION_THUMB_HEIGHT)
                continue;
            else
                thumbnailGlobalMap.at<uchar>(y, x) *= 0.8;
    cv::rectangle(thumbnailGlobalMap, Point(REGION_THUMB_OX, REGION_THUMB_OY), Point(REGION_THUMB_OX+REGION_THUMB_WIDTH, REGION_THUMB_OY+REGION_THUMB_HEIGHT), 255, 2);
    cvtColor(thumbnailGlobalMap, thumbnailGlobalMap, COLOR_GRAY2BGR);
    QImage qmap = QImage(thumbnailGlobalMap.data, thumbnailGlobalMap.cols, thumbnailGlobalMap.rows, thumbnailGlobalMap.step, QImage::Format_BGR888);
    if(GMAP_WIDTH/GMAP_HEIGHT>WIDTH/HEIGHT)
        ui->LABEL_GMAP->setFixedSize(GTHUMB_WIDTH, GTHUMB_WIDTH*GMAP_HEIGHT/GMAP_WIDTH);
    else
        ui->LABEL_GMAP->setFixedSize(GTHUMB_HEIGHT*GMAP_WIDTH/GMAP_HEIGHT, GTHUMB_HEIGHT);
    ui->LABEL_GMAP->setPixmap(QPixmap::fromImage(qmap));
}
void RomiolSCNT::menuHumanAssistance()
{
    humanAssistance.move(100, 300);
    humanAssistance.show();
    humanAssistance.exec();
}
//事件处理
bool RomiolSCNT::event(QEvent *event)
{
    //快捷键对应：
    //    Alt + 2-7: workflow 2-7
    //    ↑↓←→ PgUp, PgDn: 操作臂、显微镜 前后左右上下移动 (通过l/r/m绑定快捷键后)
    if(event->type()==QEvent::KeyPress)//键盘动作响应
    {
        QKeyEvent* keyEvent = static_cast<QKeyEvent*>(event);
        //按下回车键后，如果当前焦点 (focus) 位于某个部件(显微镜、左/右操作手臂)，则高亮该部件模块，并将快捷键与操作按钮绑定，解绑其他部件的操作按钮
        //其他快捷键之间完成响应操作
        if(keyEvent->modifiers()==Qt::AltModifier)
        {
            switch (keyEvent->key()) {
            case Qt::Key_1:
                btnStepBuildMap();
                break;
            case Qt::Key_2:
                btnStepFetchOocyte();
                break;
            case Qt::Key_3:
                btnStepRotateOutPlane();
                break;
            case Qt::Key_4:
                btnStepPenetrate();
                break;
            case Qt::Key_5:
                btnStepEnucleation();
                break;
            case Qt::Key_6:
                btnStepFetchSoma();
                break;
            case Qt::Key_7:
                btnStepInjectSoma();
                break;
            default:
                break;
            }
        }
        switch (keyEvent->key())
        {
        case Qt::Key_M:
            ui->GROUP_MICROSCOPE->setStyleSheet("background-color: rgba(0, 255, 127, 0.3);");
            ui->GROUP_LEFT_MANIP->setStyleSheet("background-color: none;");
            ui->GROUP_RIGHT_MANIP->setStyleSheet("background-color: none");
            ui->GROUP_INJECTOR->    setStyleSheet("background-color: none");
            ui->GROUP_HOLDER->      setStyleSheet("background-color: none");

            ui->BTN_INJECTOR_BLOW-> setShortcut(Qt::Key_No);
            ui->BTN_INJECTOR_SUCK-> setShortcut(Qt::Key_No);
            ui->BTN_HOLDER_SUCK->   setShortcut(Qt::Key_No);
            ui->BTN_HOLDER_BLOW->   setShortcut(Qt::Key_No);

            ui->BTN_SCOPE_UP->      setShortcut(Qt::Key_PageUp);
            ui->BTN_SCOPE_DOWN->    setShortcut(Qt::Key_PageDown);
            ui->BTN_SCOPE_LEFT->    setShortcut(Qt::Key_Left);
            ui->BTN_SCOPE_RIGHT->   setShortcut(Qt::Key_Right);
            ui->BTN_SCOPE_FORWARD-> setShortcut(Qt::Key_Up);
            ui->BTN_SCOPE_BACK->    setShortcut(Qt::Key_Down);

            ui->BTN_LEFT_MANIP_UP->setShortcut(Qt::Key_No);
            ui->BTN_LEFT_MANIP_DOWN->setShortcut(Qt::Key_No);
            ui->BTN_LEFT_MANIP_LEFT->setShortcut(Qt::Key_No);
            ui->BTN_LEFT_MANIP_RIGHT->setShortcut(Qt::Key_No);
            ui->BTN_LEFT_MANIP_FORWARD->setShortcut(Qt::Key_No);
            ui->BTN_LEFT_MANIP_BACK->setShortcut(Qt::Key_No);
            ui->BTN_RIGHT_MANIP_UP->setShortcut(Qt::Key_No);
            ui->BTN_RIGHT_MANIP_DOWN->setShortcut(Qt::Key_No);
            ui->BTN_RIGHT_MANIP_LEFT->setShortcut(Qt::Key_No);
            ui->BTN_RIGHT_MANIP_RIGHT->setShortcut(Qt::Key_No);
            ui->BTN_RIGHT_MANIP_FORWARD->setShortcut(Qt::Key_No);
            ui->BTN_RIGHT_MANIP_BACK->setShortcut(Qt::Key_No);
            break;
        case Qt::Key_L:
            ui->GROUP_MICROSCOPE->setStyleSheet("background-color: none;");
            ui->GROUP_LEFT_MANIP->setStyleSheet("background-color: rgba(0, 255, 127, 0.3);");
            ui->GROUP_RIGHT_MANIP->setStyleSheet("background-color: none;");
            ui->GROUP_INJECTOR->    setStyleSheet("background-color: none");
            ui->GROUP_HOLDER->      setStyleSheet("background-color: none");

            ui->BTN_INJECTOR_BLOW-> setShortcut(Qt::Key_No);
            ui->BTN_INJECTOR_SUCK-> setShortcut(Qt::Key_No);
            ui->BTN_HOLDER_SUCK->   setShortcut(Qt::Key_No);
            ui->BTN_HOLDER_BLOW->   setShortcut(Qt::Key_No);

            ui->BTN_SCOPE_UP->      setShortcut(Qt::Key_No);
            ui->BTN_SCOPE_DOWN->    setShortcut(Qt::Key_No);
            ui->BTN_SCOPE_LEFT->    setShortcut(Qt::Key_No);
            ui->BTN_SCOPE_RIGHT->   setShortcut(Qt::Key_No);
            ui->BTN_SCOPE_FORWARD-> setShortcut(Qt::Key_No);
            ui->BTN_SCOPE_BACK->    setShortcut(Qt::Key_No);

            ui->BTN_LEFT_MANIP_UP->setShortcut(Qt::Key_PageUp);
            ui->BTN_LEFT_MANIP_DOWN->setShortcut(Qt::Key_PageDown);
            ui->BTN_LEFT_MANIP_LEFT->setShortcut(Qt::Key_Left);
            ui->BTN_LEFT_MANIP_RIGHT->setShortcut(Qt::Key_Right);
            ui->BTN_LEFT_MANIP_FORWARD->setShortcut(Qt::Key_Up);
            ui->BTN_LEFT_MANIP_BACK->setShortcut(Qt::Key_Down);

            ui->BTN_RIGHT_MANIP_UP->setShortcut(Qt::Key_No);
            ui->BTN_RIGHT_MANIP_DOWN->setShortcut(Qt::Key_No);
            ui->BTN_RIGHT_MANIP_LEFT->setShortcut(Qt::Key_No);
            ui->BTN_RIGHT_MANIP_RIGHT->setShortcut(Qt::Key_No);
            ui->BTN_RIGHT_MANIP_FORWARD->setShortcut(Qt::Key_No);
            ui->BTN_RIGHT_MANIP_BACK->setShortcut(Qt::Key_No);
            break;
        case Qt::Key_R:
            ui->GROUP_MICROSCOPE->setStyleSheet("background-color: none;");
            ui->GROUP_LEFT_MANIP->setStyleSheet("background-color: none;");
            ui->GROUP_RIGHT_MANIP->setStyleSheet("background-color: rgba(0, 255, 127, 0.3);");
            ui->GROUP_INJECTOR->    setStyleSheet("background-color: none");
            ui->GROUP_HOLDER->      setStyleSheet("background-color: none");

            ui->BTN_INJECTOR_BLOW-> setShortcut(Qt::Key_No);
            ui->BTN_INJECTOR_SUCK-> setShortcut(Qt::Key_No);
            ui->BTN_HOLDER_SUCK->   setShortcut(Qt::Key_No);
            ui->BTN_HOLDER_BLOW->   setShortcut(Qt::Key_No);

            ui->BTN_SCOPE_UP->      setShortcut(Qt::Key_No);
            ui->BTN_SCOPE_DOWN->    setShortcut(Qt::Key_No);
            ui->BTN_SCOPE_LEFT->    setShortcut(Qt::Key_No);
            ui->BTN_SCOPE_RIGHT->   setShortcut(Qt::Key_No);
            ui->BTN_SCOPE_FORWARD-> setShortcut(Qt::Key_No);
            ui->BTN_SCOPE_BACK->    setShortcut(Qt::Key_No);

            ui->BTN_LEFT_MANIP_UP->     setShortcut(Qt::Key_No);
            ui->BTN_LEFT_MANIP_DOWN->   setShortcut(Qt::Key_No);
            ui->BTN_LEFT_MANIP_LEFT->   setShortcut(Qt::Key_No);
            ui->BTN_LEFT_MANIP_RIGHT->  setShortcut(Qt::Key_No);
            ui->BTN_LEFT_MANIP_FORWARD->setShortcut(Qt::Key_No);
            ui->BTN_LEFT_MANIP_BACK->   setShortcut(Qt::Key_No);

            ui->BTN_RIGHT_MANIP_UP->    setShortcut(Qt::Key_PageUp);
            ui->BTN_RIGHT_MANIP_DOWN->  setShortcut(Qt::Key_PageDown);
            ui->BTN_RIGHT_MANIP_LEFT->  setShortcut(Qt::Key_Left);
            ui->BTN_RIGHT_MANIP_RIGHT-> setShortcut(Qt::Key_Right);
            ui->BTN_RIGHT_MANIP_FORWARD->setShortcut(Qt::Key_Up);
            ui->BTN_RIGHT_MANIP_BACK->  setShortcut(Qt::Key_Down);
            break;
        case Qt::Key_Escape:
            ui->GROUP_MICROSCOPE->setStyleSheet("background-color: none;");
            ui->GROUP_LEFT_MANIP->setStyleSheet("background-color: none;");
            ui->GROUP_RIGHT_MANIP->setStyleSheet("background-color: none");
            ui->GROUP_INJECTOR->    setStyleSheet("background-color: none");
            ui->GROUP_HOLDER->      setStyleSheet("background-color: none");

            ui->BTN_INJECTOR_BLOW-> setShortcut(Qt::Key_No);
            ui->BTN_INJECTOR_SUCK-> setShortcut(Qt::Key_No);
            ui->BTN_HOLDER_SUCK->   setShortcut(Qt::Key_No);
            ui->BTN_HOLDER_BLOW->   setShortcut(Qt::Key_No);

            ui->BTN_SCOPE_UP->      setShortcut(Qt::Key_No);
            ui->BTN_SCOPE_DOWN->    setShortcut(Qt::Key_No);
            ui->BTN_SCOPE_LEFT->    setShortcut(Qt::Key_No);
            ui->BTN_SCOPE_RIGHT->   setShortcut(Qt::Key_No);
            ui->BTN_SCOPE_FORWARD-> setShortcut(Qt::Key_No);
            ui->BTN_SCOPE_BACK->    setShortcut(Qt::Key_No);

            ui->BTN_LEFT_MANIP_UP->     setShortcut(Qt::Key_No);
            ui->BTN_LEFT_MANIP_DOWN->   setShortcut(Qt::Key_No);
            ui->BTN_LEFT_MANIP_LEFT->   setShortcut(Qt::Key_No);
            ui->BTN_LEFT_MANIP_RIGHT->  setShortcut(Qt::Key_No);
            ui->BTN_LEFT_MANIP_FORWARD->setShortcut(Qt::Key_No);
            ui->BTN_LEFT_MANIP_BACK->   setShortcut(Qt::Key_No);

            ui->BTN_RIGHT_MANIP_UP->    setShortcut(Qt::Key_No);
            ui->BTN_RIGHT_MANIP_DOWN->  setShortcut(Qt::Key_No);
            ui->BTN_RIGHT_MANIP_LEFT->  setShortcut(Qt::Key_No);
            ui->BTN_RIGHT_MANIP_RIGHT-> setShortcut(Qt::Key_No);
            ui->BTN_RIGHT_MANIP_FORWARD->setShortcut(Qt::Key_No);
            ui->BTN_RIGHT_MANIP_BACK->  setShortcut(Qt::Key_No);

            ui->BTN_COLOR->setFocus();
            break;
        case Qt::Key_H:
            ui->GROUP_HOLDER->setStyleSheet("background-color: rgba(0, 255, 127, 0.3);");
            ui->GROUP_MICROSCOPE->  setStyleSheet("background-color: none");
            ui->GROUP_LEFT_MANIP->  setStyleSheet("background-color: none;");
            ui->GROUP_RIGHT_MANIP-> setStyleSheet("background-color: none");
            ui->GROUP_INJECTOR->    setStyleSheet("background-color: none");

            ui->BTN_HOLDER_SUCK->   setShortcut(Qt::Key_Left);
            ui->BTN_HOLDER_BLOW->   setShortcut(Qt::Key_Right);

            ui->BTN_INJECTOR_BLOW-> setShortcut(Qt::Key_No);
            ui->BTN_INJECTOR_SUCK-> setShortcut(Qt::Key_No);
            ui->BTN_SCOPE_UP->      setShortcut(Qt::Key_No);
            ui->BTN_SCOPE_DOWN->    setShortcut(Qt::Key_No);
            ui->BTN_SCOPE_LEFT->    setShortcut(Qt::Key_No);
            ui->BTN_SCOPE_RIGHT->   setShortcut(Qt::Key_No);
            ui->BTN_SCOPE_FORWARD-> setShortcut(Qt::Key_No);
            ui->BTN_SCOPE_BACK->    setShortcut(Qt::Key_No);
            ui->BTN_LEFT_MANIP_UP->setShortcut(Qt::Key_No);
            ui->BTN_LEFT_MANIP_DOWN->setShortcut(Qt::Key_No);
            ui->BTN_LEFT_MANIP_LEFT->setShortcut(Qt::Key_No);
            ui->BTN_LEFT_MANIP_RIGHT->setShortcut(Qt::Key_No);
            ui->BTN_LEFT_MANIP_FORWARD->setShortcut(Qt::Key_No);
            ui->BTN_LEFT_MANIP_BACK->setShortcut(Qt::Key_No);
            ui->BTN_RIGHT_MANIP_UP->setShortcut(Qt::Key_No);
            ui->BTN_RIGHT_MANIP_DOWN->setShortcut(Qt::Key_No);
            ui->BTN_RIGHT_MANIP_LEFT->setShortcut(Qt::Key_No);
            ui->BTN_RIGHT_MANIP_RIGHT->setShortcut(Qt::Key_No);
            ui->BTN_RIGHT_MANIP_FORWARD->setShortcut(Qt::Key_No);
            ui->BTN_RIGHT_MANIP_BACK->setShortcut(Qt::Key_No);
            break;
        case Qt::Key_I:
            ui->GROUP_INJECTOR->setStyleSheet("background-color: rgba(0, 255, 127, 0.3);");
            ui->GROUP_MICROSCOPE->  setStyleSheet("background-color: none");
            ui->GROUP_LEFT_MANIP->  setStyleSheet("background-color: none;");
            ui->GROUP_RIGHT_MANIP-> setStyleSheet("background-color: none");
            ui->GROUP_HOLDER->      setStyleSheet("background-color: none");

            ui->BTN_INJECTOR_BLOW->   setShortcut(Qt::Key_Left);
            ui->BTN_INJECTOR_SUCK->   setShortcut(Qt::Key_Right);

            ui->BTN_HOLDER_SUCK-> setShortcut(Qt::Key_No);
            ui->BTN_HOLDER_BLOW-> setShortcut(Qt::Key_No);
            ui->BTN_SCOPE_UP->      setShortcut(Qt::Key_No);
            ui->BTN_SCOPE_DOWN->    setShortcut(Qt::Key_No);
            ui->BTN_SCOPE_LEFT->    setShortcut(Qt::Key_No);
            ui->BTN_SCOPE_RIGHT->   setShortcut(Qt::Key_No);
            ui->BTN_SCOPE_FORWARD-> setShortcut(Qt::Key_No);
            ui->BTN_SCOPE_BACK->    setShortcut(Qt::Key_No);
            ui->BTN_LEFT_MANIP_UP->setShortcut(Qt::Key_No);
            ui->BTN_LEFT_MANIP_DOWN->setShortcut(Qt::Key_No);
            ui->BTN_LEFT_MANIP_LEFT->setShortcut(Qt::Key_No);
            ui->BTN_LEFT_MANIP_RIGHT->setShortcut(Qt::Key_No);
            ui->BTN_LEFT_MANIP_FORWARD->setShortcut(Qt::Key_No);
            ui->BTN_LEFT_MANIP_BACK->setShortcut(Qt::Key_No);
            ui->BTN_RIGHT_MANIP_UP->setShortcut(Qt::Key_No);
            ui->BTN_RIGHT_MANIP_DOWN->setShortcut(Qt::Key_No);
            ui->BTN_RIGHT_MANIP_LEFT->setShortcut(Qt::Key_No);
            ui->BTN_RIGHT_MANIP_RIGHT->setShortcut(Qt::Key_No);
            ui->BTN_RIGHT_MANIP_FORWARD->setShortcut(Qt::Key_No);
            ui->BTN_RIGHT_MANIP_BACK->setShortcut(Qt::Key_No);
            break;
        case Qt::Key_Alt&&Qt::Key_5:
            btnStepEnucleation();
            break;
        case Qt::Key_Return:
            stepBuildMap->ON = false;
            stepFetchOocyte->ON = false;
            stepRotateOutPlane->ON = false;
            stepRotateInPlane->ON = false;
            stepPenetrate->ON = false;
            stepEnucleation->ON = false;
            stepFetchSoma->ON = false;
            stepReleaseOocyte->ON = false;
            stepExpel->ON = false;
            stepFetchSoma->ON = false;
            stepFetchOocyteForSoma->ON = false;
            stepRotateForSoma->ON = false;
            stepPenetrateForSoma->ON = false;
            stepInjectSoma->ON = false;

            functions->ON = false;
            break;
        case Qt::Key_Enter:
            stepBuildMap->ON = false;
            stepFetchOocyte->ON = false;
            stepRotateOutPlane->ON = false;
            stepRotateInPlane->ON = false;
            stepPenetrate->ON = false;
            stepEnucleation->ON = false;
            stepFetchSoma->ON = false;
            stepReleaseOocyte->ON = false;
            stepExpel->ON = false;
            stepFetchSoma->ON = false;
            stepFetchOocyteForSoma->ON = false;
            stepRotateForSoma->ON = false;
            stepPenetrateForSoma->ON = false;
            stepInjectSoma->ON = false;

            functions->ON = false;
            break;


        case Qt::Key_F1:
            ui->BTN_X4->setChecked(true);
            microscope->changeObjective(x4);
            break;
        case Qt::Key_F2:
            ui->BTN_X10->setChecked(true);
            microscope->changeObjective(x10);
            break;
        case Qt::Key_F3:
            ui->BTN_X20->setChecked(true);
            microscope->changeObjective(x20);
            break;
        case Qt::Key_F4:
            ui->BTN_X40->setChecked(true);
            microscope->changeObjective(x40);
            break;
        case Qt::Key_F5:
            if(ui->BTN_AUTOCLONE->isChecked())
                ui->BTN_AUTOCLONE->setChecked(false);
            else
                ui->BTN_AUTOCLONE->setChecked(true);

            btnAutoClone();
            break;
        case Qt::Key_F6:
            btnObjectDetector();
            break;
        case Qt::Key_F7:
            if(!ui->BTN_SHOW_ALL->isChecked())
                ui->BTN_SHOW_ALL->setChecked(true);
            else
                ui->BTN_SHOW_ALL->setChecked(false);
            btnShowAll();
            break;
        case Qt::Key_F8:
            if(!ui->BTN_SOCKET_CONNECT->isChecked())
            {
                emit sgnConnectSocket();
                ui->BTN_SOCKET_CONNECT->setChecked(true);
                ui->BTN_SOCKET_CONNECT->setText("断\n开");
            }
            else
            {
                emit sgnCloseSocket();
                ui->BTN_SOCKET_CONNECT->setChecked(false);
                ui->BTN_SOCKET_CONNECT->setText("连\n接");
            }
            break;
        case Qt::Key_F9:
            ui->BTN_MOVE_UP->setChecked(true);
            emit sgnMoveLManipZ(-areaMoveHeight, 1000, false);
            emit sgnMoveRManipZ(-areaMoveHeight, 1000, false);
            ui->BTN_MOVE_UP->setChecked(false);
            break;
        case Qt::Key_F10:
            ui->BTN_MOVE_DOWN->setChecked(true);
            emit sgnMoveLManipZ(areaMoveHeight, 1000, false);
            emit sgnMoveRManipZ(areaMoveHeight, 1000, false);
            ui->BTN_MOVE_DOWN->setChecked(false);
            break;
        case Qt::Key_F12:
            btnRestart();
            break;
        case Qt::Key_Delete:
            btnReset();
            break;

        case Qt::Key_Slash:
            ui->BTN_SCOPE_LOW_SPEED->setChecked(true);
            break;
        case Qt::Key_Asterisk:
            ui->BTN_SCOPE_MEDIUM_SPEED->setChecked(true);
            break;
        case Qt::Key_Minus:
            ui->BTN_SCOPE_HIGH_SPEED->setChecked(true);
            break;
        case Qt::Key_Plus:
            ui->BTN_SCOPE_CUSTOM_SPEED->setChecked(true);
            break;

        default:
            break;
        }
    }
    else if(event->type()==QEvent::MouseButtonPress)//鼠标动作响应: 停止调焦电机运动、选择物体轮廓点
    {
        QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);
        QPointF pos = mouseEvent->globalPosition() - ui->LABEL_IMAGE->mapToGlobal(QPointF(0, 0));//鼠标在当前图像中的位置
        if(pos.x()>=0 && pos.x()<WIDTH && pos.y()>=0 && pos.y()<HEIGHT)
        {
            emit sgnStopFocus();//连动调焦中，单击停止电机运动
            if(humanAssistance.isSelecting)
                selectPoints.push_back(cv::Point(pos.x(), pos.y()));
        }
    }
    else if(event->type()==QEvent::MouseButtonRelease)
    {
        QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);
        QPointF pos = mouseEvent->globalPosition() - ui->LABEL_GMAP->mapToGlobal(QPointF(0, 0));//鼠标在全局缩略图中的位置
        //鼠标点击目标位置，移动平台使目标位置在视野中心；假设切换镜头后视野中心位置不变
        if(pos.x()>=0 && pos.x()<=ui->LABEL_GMAP->size().width() &&
            pos.y()>=0 && pos.y()<=ui->LABEL_GMAP->size().height()&&globalMap->isValid)
            emit sgnGotoArea(Point2f(oocyteArea.x+(pos.x()*GMAP_WIDTH /ui->LABEL_GMAP->size().width ()-WIDTH /2)*globalMap->pixel2Um,
                                                  oocyteArea.y+(pos.y()*GMAP_HEIGHT/ui->LABEL_GMAP->size().height()-HEIGHT/2)*globalMap->pixel2Um));
    }
    else if(event->type()==QEvent::Wheel)
    {
        QWheelEvent* wheelEvent = static_cast<QWheelEvent*>(event);
//        QPointF pos = wheelEvent->position() - ui->LABEL_IMAGE->pos() - QPointF(0, 22);//首栏高度
        QPointF pos = wheelEvent->globalPosition() - ui->LABEL_IMAGE->mapToGlobal(QPointF(0, 0));
        if(pos.x()>=0 && pos.x()<=ui->LABEL_IMAGE->width() && pos.y()>=0 && pos.y()<=ui->LABEL_IMAGE->height())
        {
            int delta = wheelEvent->angleDelta().y();
            emit sgnMoveFocusZ(0, true, 10.0*delta/120);
            // microscope->moveZ(0, true, 10.0*delta/120);
        }
    }
    return QMainWindow::event(event);
}
void RomiolSCNT::mouseMoveEvent(QMouseEvent *event)
{
    QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);
    QPointF pos = mouseEvent->globalPosition() - ui->LABEL_IMAGE->mapToGlobal(QPointF(0, 0));//鼠标在当前图像中的位置
    if(pos.x()>=0 && pos.x()<WIDTH && pos.y()>=0 && pos.y()<HEIGHT)
    {
        if(CurrentGray.empty())
            pixelStatus->setText(QString("(%1, %2)").arg(pos.x()).arg(pos.y()));
        else
            pixelStatus->setText(QString("(%1, %2: %3)").arg(pos.x()).arg(pos.y()).arg(CurrentGray.at<uchar>(pos.y(), pos.x())));
    }
}
