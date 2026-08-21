#include "stepenucleation.h"
using namespace cv;
using namespace std;
StepEnucleation::StepEnucleation()
{
    // enuController = new Controller_MRFOASMC_RBF();
    // enuController = new Controller_CGPC(1, 1);
    enuController = new Controller_FOASMC();
    // enuController = new Controller_ZDT();
    // enuController = new Controller_PID();
    // enuController = new Controller_FUZZY_FOASMC_RBF_PI();
    // enuController->U_name = "p";
    // enuController->uM[0] = 10000;
    // enuController->um[0] = -2000;
    enuController->y_th = 20;
    enuController->u1 = 0.05;
    enuController->saveParas();
    // enuController->qParas["p_plus"]=1000;
    // enuController->qParas["p_minus"]=-1000;
    // enuController->qParas["eta"]=1;
    // enuController->setParas(enuController->qParas);
    //设置控制器参数 模板
    vector<map<string, double>> trajParas(2, map<string, double>());
    trajParas[0]["MODE"] = TRAJ_MODE_SETPOINT;
    trajParas[0]["setPoint"]=150;
    trajParas[1]["MODE"] = TRAJ_MODE_SETPOINT;
    trajParas[1]["setPoint"]=0;
    enuController->setTrajParas(trajParas);


    injP_Controller = new Controller_PID;
    injP_Controller->uM[0]=0.3;
    injP_Controller->um[0]=-0.3;
    injP_Controller->saveParas();
    injP_Controller->qParas["kp"]=1e-4;
    injP_Controller->qParas["kd"]=1e-5;
    injP_Controller->qParas["ki"]=0.0;
    injP_Controller->setParas(injP_Controller->qParas);
}
StepEnucleation::~StepEnucleation(){
    delete enuController;
}
WORKSTEP StepEnucleation::run()
{
    isWorking = true;

    if(isHCI)//人机交互模式
    {
        DEAL_FAULT("请手动完成去核操作！");
        emit funcWithdrawInj();
        if(mode==SCNT_BATCH)
            DEAL_FAULT("是否继续对下一个细胞抽核？");
        //------------------------------------------------------步骤完成---------------------------------------------------
        if(mode==SCNT_SINGLE)
            nextStep = EXPEL;
        else if(areAllOocytesEnucleated)
            nextStep = RELEASEOOCYTE;
        else
            nextStep = FETCHOOCYTE;
        isWorking = false;
        emit finish();
        return nextStep;
    }
    //添加处理代码: 抽取定量物质后，撤针；移动到吐核区域，吐核
    //物体检测
    currentOocyte->find(CurrentGray);
    injectPipette->find(CurrentGray);
    if(!currentOocyte->isDetected)
        DEAL_FAULT("未检测到卵细胞！");
    if(!injectPipette->isDetected)
        DEAL_FAULT("未检测到注射针！");
    //初始化
    injectPipette->mhiOCI.reset();
    enuController->initialize();
    enuController->nz = 2;
    injP_Controller->initialize();
    //设置轨迹参数
    double tarPos = 100*Pixel2Um;// unit: um
    // double tarPos = 0.08*16*pow(currentOocyte->radiusZP,3)/(3*pow(injectPipette->diameter, 2))*Pixel2Um;// unit: um
    // qDebug()<<"tarPos: "<<tarPos<<"\n";
    if(enuController->trajParas[0]["MODE"] == TRAJ_MODE_SETPOINT)
        if(enuController->name=="MRFOASMC_RBF"||enuController->name=="HinfSMCFOI")
            enuController->trajParas[0]["setPoint"]=0;
        else
            enuController->trajParas[0]["setPoint"]=tarPos;
    else if(enuController->trajParas[0]["MODE"] == TRAJ_MODE_SIGMOID)
        enuController->trajParas[0]["a"]=tarPos;
    else if(enuController->trajParas[0]["MODE"] == TRAJ_MODE_ACC_DEC)
        enuController->trajParas[0]["XF"]=tarPos;
    else if(enuController->trajParas[0]["MODE"] == TRAJ_MODE_SINE)
    {
        enuController->trajParas[0]["A"]=tarPos/4;
        enuController->trajParas[0]["B"]=tarPos/2;
    }
    enuController->setTrajParas(enuController->trajParas);
    //控制器循环
    ON = true;
    vector<double> y, u, z;
    while(ON)
    {
        if(injector->pSensor->isConnected&&(!injector->isLiquid)&&injector->pSensor->pressure<=injExtractSafeP)//使用气泵时限制最大去核负压
        {
            emit injSuck(0, 0, true);
            ON = false;
            DEAL_FAULT("细胞状态可能变差，请检查是否可以抽动细胞质");
            break;
        }
        if(CurrentGray.empty())
            break;
        //获取当前时刻
        QTime time = QTime::currentTime().addMSecs(enuController->getDT()*1000);
        //处理
        currentOocyte->calcStrain(CurrentGray);
        injectPipette->findOCI(CurrentGray, currentOocyte->center.x);
        y.clear(); y.push_back(injectPipette->xOCI*Pixel2Um);   y.push_back(currentOocyte->strain);
        z.clear(); z.push_back(injector->pSensor->pressure);    z.push_back(injector->syringeMotor->getAbsPos());
        //默认在控制器外部将初始观测值设置为 0
        if(enuController->y_history.empty())
        {
            injectPipette->x0OCI=y[0]/Pixel2Um;
            injectPipette->xfOCI=(y[0]+enuController->trajParas[0]["setPoint"])/Pixel2Um;
            if(enuController->name=="MRFOASMC_RBF"||enuController->name=="HinfSMCFOI"||enuController->name=="Fuzzy_FOASMC_RBF_PI")
            {
                enuController->y_bias.clear();
                enuController->y_bias.push_back(y[0] + enuController->trajParas[0]["setPoint"]);
            }
            else
            {
                enuController->y_bias.clear();
                enuController->y_bias.push_back(y[0]);
            }
        }
        enuController->set_y(y);
        enuController->set_z(z);
        enuController->get_yd(enuController->k_step*enuController->delta_t);
        enuController->get_u(u);
        enuController->step();
        if(enuController->U_name=="p")
        {
            //内环气压控制
            injBiasP = u[0];
            y.clear(); y.push_back(injector->pSensor->pressure);
            injP_Controller->set_y(y);
            y.clear(); y.push_back(injBalanceP-injBiasP);
            injP_Controller->set_yd(y);
            injP_Controller->get_u(u);
            injP_Controller->step();
            emit injSuck(0, -u[0], true);
        }
        else if(enuController->U_name=="spd")
        {
            emit injSuck(0, u[0], true);
            qDebug()<<u[0];
        }

        // if(injectPipette->xOCI-currentOocyte->xMZP<5)
        // {
        //     enuController->uM[0] = 0.04;
        //     enuController->um[0] = -0.1;
        // }
        // else if(injectPipette->xOCI-currentOocyte->xMZP>=5 && injectPipette->xOCI<WIDTH)
        // {
        //     enuController->uM[0] = 0.01;
        //     enuController->um[0] = -0.01;
        // }
        //等待一段时间进行下一步
        while(QTime::currentTime() < time);
        //满足结束条件，退出
        if(injectPipette->xOCI>=injectPipette->xfOCI)
        {
            break;
            qDebug()<<injectPipette->xOCI<<injectPipette->xfOCI;
        }
    }
    emit injSuck(0, 0, true);
    // if(holder->pSensor->pressure>holdPressure+500)
    //     emit funcAdjHoldP(holdPressure);
    // emit funcWithdrawInj();
    enuController->save_data("OCI_data.txt");
    //------------------------------------------------------步骤完成---------------------------------------------------
    if(mode==SCNT_SINGLE)
        nextStep = EXPEL;
    else if(areAllOocytesEnucleated)
        nextStep = RELEASEOOCYTE;
    else
        nextStep = FETCHOOCYTE;
    isWorking = false;
    emit finish();
    return nextStep;
}
