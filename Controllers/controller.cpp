#include "controller.h"

using namespace std;
Controller::Controller(int n_in, int n_out, int n_observe)
{
    name = "Controller";
    isStage2 = false;
    // 默认参数值
    y_th = -INFINITY;
    u1 = 0.05;
    U_name = "spd";       //默认控制输入为电机速度 其他值: pos（电机位置）, p（内环气压控制）
    p = n_in;
    m = n_out;
    nz = n_observe;
    delta_t = 0.1;
    k_step = 0;
    isSaveLastPara = true;
    isInitialized = false;
    for(int i=0; i<p; ++i)
    {
        uM.push_back(INFINITY);
        um.push_back(-INFINITY);
    }
    y_bias = vector<double>(m, 0);
    setPoint = vector<double>(m, 0);
    for(int i=0; i<m; ++i)
        trajParas.push_back(map<string, double>());

    saveParas();//设置好参数后保存参数
    //加载模块，模块名称为myModule，就是myModule.py文件
    pModule = PyImport_ImportModule("viz");
    //如果加载失败，则返回
    if(!pModule)
    {
        qDebug()<<"Error Load Module";
        return;
    }
    //加载函数greatFunc
    else
    {
        pFuncDrawTraj = PyObject_GetAttrString(pModule, "drawTraj");
        pFuncDrawU = PyObject_GetAttrString(pModule, "drawU");
    }
    //如果失败则返回
    if(!pFuncDrawTraj || !pFuncDrawU)
        qDebug()<<"Error Load Function";
}
void Controller::saveParas()
{
    //参数保存
    QList<double> qlist;
    QVariant qVar;
    qParas["U_name"] = QString(U_name);
    qlist = QList<double>::fromVector(QVector<double>(uM.begin(), uM.end()));
    qVar.setValue(qlist);
    qParas["uM"] = qVar;
    qlist = QList<double>::fromVector(QVector<double>(um.begin(), um.end()));
    qVar.setValue(qlist);
    qParas["um"] = qVar;
    qParas["delta_t"] = delta_t;
    qParas["y_th"] = y_th;
    qParas["u1"] = u1;
    qParas["isSaveLastPara"] = isSaveLastPara;
}
void Controller::setParas(std::map<std::string, QVariant> q)
{
    isInitialized = false;
    qParas = q;
    QList<double> qlist;
    if(qParas.count("U_name")>0)
        U_name = qParas["U_name"].toString();
    if(qParas.count("uM")>0)
    {
        qlist = qParas["uM"].value<QList<double>>();
        copy(qlist.begin(), qlist.end(), uM.begin());
    }
    if(qParas.count("um")>0)
    {
        qlist = qParas["um"].value<QList<double>>();
        copy(qlist.begin(), qlist.end(), um.begin());
    }
    if(qParas.count("delta_t")>0)
        delta_t = qParas["delta_t"].toFloat();
    if(qParas.count("y_th")>0)
        y_th = qParas["y_th"].toFloat();
    if(qParas.count("u1")>0)
        u1 = qParas["u1"].toFloat();
    if(qParas.count("isSaveLastPara")>0)
        isSaveLastPara = qParas["isSaveLastPara"].toBool();
}
void Controller::setTrajParas(std::vector<std::map<std::string, double>> tParas)
{
    trajParas.assign(tParas.begin(), tParas.end());
    for(int i=0; i<m; ++i)
        if(trajParas[i].count("setPoint")>0)
            setPoint[i] = tParas[i]["setPoint"];
}
void Controller::initialize()
{
    y_history.clear();
    yd_history.clear();
    z_history.clear();
    u_real_history.clear();
    u_idea_history.clear();
    // 获取开始时刻毫秒时间戳
    k_step = 0;
    t0 = QDateTime::currentMSecsSinceEpoch();
    mtime.clear();
}
void Controller::get_yd(double t)
{
    yd.clear();
    if(t<0)
        t=k_step*delta_t;
    for(int i=0; i<m; ++i)
    {
        if(trajParas[i]["MODE"]==TRAJ_MODE_SETPOINT)
            yd.push_back(setPoint[i]);
        else if(trajParas[i]["MODE"]==TRAJ_MODE_SIGMOID)
        {
            double a, b, k;
            a = trajParas[i]["a"];
            b = trajParas[i]["b"];
            k = trajParas[i]["k"];
            yd.push_back(a/(1+exp(-k*(t-b))));
        }
        if(trajParas[i]["MODE"]==TRAJ_MODE_SINE)
        {
            double A, w, p, B;
            A = trajParas[i]["A"];
            w = trajParas[i]["w"];
            p = trajParas[i]["p"];
            B = trajParas[i]["B"];
            yd.push_back(A*sin(w*t+p)+B);
        }
        if(trajParas[i]["MODE"]==TRAJ_MODE_ACC_DEC)
        {
            double AM, DM, VM, XF, TF;              //目标轨迹相关参数
            double a, b, c, delta;                  //中间变量
            double t1_1, t1_2, t2_1, t2_2;          //一次|二次切换点
            double tar_pos;
            AM = trajParas[i]["AM"];
            DM = trajParas[i]["DM"];
            VM = trajParas[i]["VM"];
            XF = trajParas[i]["XF"];
            TF = trajParas[i]["TF"];

            a = 0.5*DM*(AM+DM)/AM;
            b = DM*DM*TF/AM;
            c = XF + 0.5*DM*TF*TF*(DM-AM)/AM;
            delta = DM*TF*DM*TF-2*DM*XF*(1+DM/AM);
            //t1, t2 轨迹变化的两个时间点；有两组解
            t2_1 = 0.5*(b+sqrt(delta))/a;
            t2_2 = 0.5*(b-sqrt(delta))/a;
            t1_1 = DM*(TF-t2_1)/AM;
            t1_2 = DM*(TF-t2_2)/AM;

            if(delta<0)
                tar_pos = 0;
            else
            {
                if(0<t1_1 && t1_1<=t2_1 && t2_1<=TF)
                {
                    if(t<t1_1)
                        tar_pos = 0.5*AM*t*t;
                    else if(t<t2_1)
                        tar_pos = 0.5*AM*t1_1*t1_1 + AM*t1_1*(t-t1_1);
                    else if(t<TF)
                        tar_pos = XF - 0.5*DM*(TF-t)*(TF-t);
                    else
                        tar_pos = XF;
                }
                else if(0<t1_2 && t1_2<=t2_2 &&t2_2<=TF)
                {
                    if(t<t1_2)
                        tar_pos = 0.5*AM*t*t;
                    else if(t<t2_2)
                        tar_pos = 0.5*AM*t1_2*t1_2 + AM*t1_2*(t-t1_2);
                    else if(t<TF)
                        tar_pos = XF - 0.5*DM*(TF-t)*(TF-t);
                    else
                        tar_pos = XF;
                }
                else
                    tar_pos = 0;
            }
            yd.push_back(tar_pos);
        }
        else if(trajParas[i]["MODE"]==TRAJ_MODE_SLOPE)
        {
            double VM, XF;
            VM = trajParas[i]["VM"];
            XF = trajParas[i]["XF"];
            if(VM*t<XF)
                yd.push_back(VM*t);
            else
                yd.push_back(XF);
        }
    }
    yd_history.push_back(yd);
}
void Controller::set_yd(vector<double> yd_)
{
    yd.assign(yd_.begin(), yd_.end());
    yd_history.push_back(yd);
}
void Controller::save_data(string filename)
{
    time_t now = time(NULL);        tm* tm_t = localtime(&now); //根据当前时刻创建文件夹保存数据
    string path = string(FILE_PATH.toStdString()+"/"+to_string((long long)tm_t->tm_year+1900)+"-"+to_string((long long)tm_t->tm_mon+1)+"-"+to_string((long long)tm_t->tm_mday)+"-"+to_string((long long)tm_t->tm_hour)+"-"+to_string((long long)tm_t->tm_min))+"/";
    _mkdir(path.c_str());
    ofstream txtfile(path+filename);
    txtfile.setf(ios::scientific, ios::floatfield);   txtfile.precision(4);
    //写入参数
    txtfile<<name.toStdString()<<", Parameters: ";
    for(auto &t : qParas)
    {
        txtfile << t.first << "=";
        if(t.second.typeId()==QMetaType::Int)
            txtfile << t.second.toInt();
        else if(t.second.typeId()==QMetaType::Float)
            txtfile << t.second.toFloat();
        else if(t.second.typeId()==QMetaType::Double)
            txtfile << t.second.toDouble();
        else
        {
            txtfile << "[";
            QList<double> tmp = t.second.value<QList<double>>();
            for(int i=0; i<tmp.size(); ++i)
                txtfile << tmp[i]<<" ";
            txtfile << "]";
        }
        txtfile << ", ";
    }
    txtfile << endl;
    //写入数据 i t_i r1 y1 ... rm ym z1 ... z_nz u_idea1 u_real1 ... u_idea_p u_real_p
    txtfile<<"k_step\tTime\t";
    for(int i=1; i<=m; ++i) txtfile<<"r"<<i<<"\t"<<"y"<<i<<"\t";
    for(int i=1; i<=nz; ++i) txtfile<<"z"<<i<<"\t";
    for(int i=1; i<p; ++i) txtfile<<"u_idea_"<<i<<"\t"<<"u_real_"<<i<<"\t"; txtfile<<"u_idea_"<<p<<"\t"<<"u_real_"<<p<<endl;
    for(int k=0; k<k_step; ++k)
    {
        txtfile<<k<<"\t"<<mtime[k]<<"\t";
        for(int i=0; i<m; ++i)
            txtfile<<yd_history[k][i]<<"\t"<<y_history[k][i]<<"\t";
        for(int i=0; i<nz; ++i)
            txtfile<<z_history[k][i]<<"\t";
        for(int i=0; i<p-1; ++i)
            txtfile<<u_idea_history[k][i]<<"\t"<<u_real_history[k][i]<<"\t";
        txtfile<<u_idea_history[k][p-1]<<"\t"<<u_real_history[k][p-1]<<endl;
    }
    txtfile.close();
}
void Controller::drawTrajectory()
{
    //调用 Python 函数
    PyObject* pyTim = PyList_New(0);           //初始化一个列表: 时间列表
    PyObject* pyPos = PyList_New(0);           //初始化一个列表: 实际位置
    PyObject* pyRef = PyList_New(0);           //初始化一个列表：参考轨迹
    for(size_t i=0; i<k_step; ++i)
    {
        // PyList_Append(pyTim, Py_BuildValue("d", double(i)));
        PyList_Append(pyTim, Py_BuildValue("d", double(1.0*mtime[i]/1000)));
        PyList_Append(pyPos, Py_BuildValue("d", y_history[i][0]));
        PyList_Append(pyRef, Py_BuildValue("d", yd_history[i][0]));
    }
    PyObject* pArgs = PyTuple_New(3);
    PyTuple_SetItem(pArgs, 0, pyTim);
    PyTuple_SetItem(pArgs, 1, pyPos);
    PyTuple_SetItem(pArgs, 2, pyRef);
    PyEval_CallObject(pFuncDrawTraj, pArgs);
}
void Controller::drawU()
{
    //调用 Python 函数
    PyObject* pyTim = PyList_New(0);
    PyObject* pyU = PyList_New(0);
    for(size_t i=0; i<k_step; ++i)
    {
        // PyList_Append(pyTim, Py_BuildValue("d", double(i)));
        PyList_Append(pyTim, Py_BuildValue("d", double(1.0*mtime[i]/1000)));
        PyList_Append(pyU, Py_BuildValue("d", u_real_history[i][0]));
    }
    PyObject* pArgs = PyTuple_New(2);
    PyTuple_SetItem(pArgs, 0, pyTim);
    PyTuple_SetItem(pArgs, 1, pyU);
    PyEval_CallObject(pFuncDrawU, pArgs);
}
void Controller::set_y(vector<double> y_observe)
{
    y.clear();
    for(int i=0; i<m; ++i)
        y.push_back(y_observe[i]-y_bias[i]);
    y_history.push_back(y);
    if(!isStage2 && y_observe[0]-y_history[0][0]-y_bias[0]>=y_th)
        isStage2 = true;
}
void Controller::set_z(vector<double> z_observe)
{
    z.clear();
    for(int i=0; i<z_observe.size(); ++i)
        z.push_back(z_observe[i]);
    z_history.push_back(z);
}
void Controller::get_u(vector<double>&u)
{
    vector<double> ui, ur;
    ui.assign(u.begin(), u.end());
    ur.assign(u.begin(), u.end());
    u_idea_history.push_back(ui);

    for(int i=0; i<p; ++i)
        ur[i] = max(min(ui[i], uM[i]), um[i]);
    if(!isStage2)
        ur[0] = u1;
    u_real.assign(ur.begin(), ur.end());
    u_real_history.push_back(ur);
    u.assign(ur.begin(), ur.end());
}
void Controller::step()
{
    k_step++;
    mtime.push_back(QDateTime::currentMSecsSinceEpoch()-t0);
}
void Controller::loadParasFromJson(QString filePath, QString key)
{
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qDebug() << "无法打开文件:" << filePath;
        return;
    }

    QByteArray jsonData = file.readAll();
    file.close();

    QJsonParseError parseError;
    QJsonDocument jsonDoc = QJsonDocument::fromJson(jsonData, &parseError);

    if (parseError.error != QJsonParseError::NoError) {
        qDebug() << "JSON 解析错误:" << parseError.errorString();
        return;
    }

    // 检查是否为对象
    string s_k;
    if (jsonDoc.isObject()) {
        QJsonObject jsonObj = jsonDoc.object();
        QJsonObject paraObj;
        if(jsonObj.contains(key))
        {
            paraObj = jsonObj.value(key).toObject();
            for(auto k:paraObj.keys())
            {
                s_k = k.toStdString();
                auto v = paraObj.value(k);
                if(v.isDouble())
                    qParas[s_k] = v.toDouble();
                else if(v.isArray())
                {
                    QVariantList qvList = v.toArray().toVariantList();
                    QList<double> qlist;
                    QVariant qVar;
                    for(auto item:qvList)
                        qlist.append(item.toDouble());
                    qVar.setValue(qlist);
                    qParas[s_k] = qVar;
                }
                else if(v.isString())
                    qParas[s_k] = v.toString();
                else if(v.isBool())
                    qParas[s_k] = v.toBool();
            }
        }
    }
    setParas(qParas);
}
