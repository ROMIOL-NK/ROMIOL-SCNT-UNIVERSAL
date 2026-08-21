#include "syringemotorgt.h"


void commandhandler(QString command, short error)
{
    // 如果指令执行返回值为非0，说明指令执行错误，向屏幕输出错误结果
    if(error)
        qDebug() << command << " = " << error;
}


SyringeMotorGT::SyringeMotorGT(int ppR, QString name)
{
    DeviceName = name;
    pulsePerRound = ppR;
    isConnected = false;
    b_same_dir = false;                  //设备正方向与规定正方向相同
    ori = 0;
    upperBound = 7.0;
    lowerBound = -7.0;
    //从 qrc 中复制一份配置文件到运行目录
    QFile file(":/config/GTS800.cfg");
    file.copy("googol.cfg");
    queryStateInterval = 50;
    queryStateTimer = new QTimer(this);
    connect(queryStateTimer, &QTimer::timeout, [this](){
        double pos;
        GT_GetAxisEncPos(AXIS, &pos);
        raw_pos = pos/pulsePerRound;
        emit updatePos();
    });
}
SyringeMotorGT::~SyringeMotorGT()
{
    GT_Close();
}

void SyringeMotorGT::connectDevice(QString portName)
{
    sRtn = GT_Open();   // 打开运动控制器
    sRtn = GT_Reset();  // 复位运动控制器
    if(!sRtn)
    {
        emit startConnection();
        queryStateTimer->start(queryStateInterval);
        isConnected = true;
    }
    // 配置运动控制器
    // 注意：配置文件取消了各轴的报警和限位
    char cfg_file[] = "googol.cfg";
    sRtn = GT_LoadConfig(cfg_file); //作为资源文件被加载
    commandhandler((char*)"GT_LoadConfig ", sRtn);
    // 清除各轴的报警和限位
    GT_ClrSts(1, 8);
    sRtn = GT_ZeroPos(AXIS);
    commandhandler("GT_ZeroPos", sRtn);
}
void SyringeMotorGT::move(float disp, float vel, bool CONTINUOUS)
{
    if(CONTINUOUS)
    {
        moveContinuous(vel);
    }
    else
    {
        moveDistance(disp);
    }
}
void SyringeMotorGT::suck(float disp, float vel, bool CONTINUOUS)
{
    move(disp, vel, CONTINUOUS);
}
void SyringeMotorGT::blow(float disp, float vel, bool CONTINUOUS)
{
    move(-disp, -vel, CONTINUOUS);
}
void SyringeMotorGT::moveDistance(float z)
{
    TTrapPrm trap;
    long disp = z*pulsePerRound;
    // 伺服使能
    GT_AxisOn(AXIS);
//    long va;
//    sRtn = GT_GetDo(MC_ENABLE,&va);
//    printf("do =%d\r\n",va);

//    sRtn = GT_GetDo(MC_ENABLE,&va);
//    printf("----do =%d\r\n",va);
//    sRtn = GT_GetDo(MC_GPO,&va);
//    printf("DO........ =%d\r\n",va);
//    va =0xaa;
//    sRtn = GT_SetDo(MC_GPO,va);
//    sRtn = GT_GetDo(MC_GPO,&va);
//    printf("^^^^^^^^^^^^^^^^do =%d\r\n",va);



    // 位置清零
    // sRtn = GT_ZeroPos(AXIS);
    // commandhandler("GT_ZeroPos", sRtn);
    // AXIS轴规划位置清零
    sRtn = GT_SetPrfPos(AXIS, 0);
    commandhandler("GT_SetPrfPos", sRtn);
    // 将AXIS轴设为点位模式
    sRtn = GT_PrfTrap(AXIS);
    commandhandler("GT_PrfTrap", sRtn);
    // 读取点位运动参数
    sRtn = GT_GetTrapPrm(AXIS, &trap);
    commandhandler("GT_GetTrapPrm", sRtn);
    trap.acc = 0.25;
    trap.dec = 0.125;
    trap.smoothTime = 25;
    // 设置点位运动参数
    sRtn = GT_SetTrapPrm(AXIS, &trap);
    commandhandler("GT_SetTrapPrm", sRtn);
    // 设置AXIS轴的目标位置
    sRtn = GT_SetPos(AXIS, disp);
    commandhandler("GT_SetPos", sRtn);
    // 设置AXIS轴的目标速度
    sRtn = GT_SetVel(AXIS, 50);
    commandhandler("GT_SetVel", sRtn);
    // 启动AXIS轴的运动
    sRtn = GT_Update(1<<(AXIS-1));
    commandhandler("GT_Update", sRtn);

//    GT_AxisOff(AXIS);
}

void SyringeMotorGT::moveContinuous(double v)
{
    v*=350;
    // 伺服使能
    GT_AxisOn(AXIS);
    GT_PrfJog(AXIS);
    GT_GetJogPrm(AXIS, &jog);
    jog.acc = 0.0625;
    jog.dec = 0.0625;
    GT_SetJogPrm(AXIS, &jog);
    sRtn = GT_SetVel(AXIS, v);      // 设置AXIS轴的目标速度
    sRtn = GT_Update(1<<(AXIS-1));  // 启动AXIS轴的运动
    GT_GetSts(AXIS, &sts);
//    qDebug()<<sts;
}

void SyringeMotorGT::stop()
{
    GT_AxisOff(AXIS);
}
void SyringeMotorGT::close()
{
    GT_Close();
    isConnected = false;
}
