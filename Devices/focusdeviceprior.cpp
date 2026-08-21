#include "focusdeviceprior.h"

FocusDevicePrior::FocusDevicePrior()
{
    b_same_dir = true;                  //设备正方向与规定正方向是否相同
    DeviceName = "Focus Device Prior";
    endflag = "\r";
    queryStateInterval = 100;
    connectionInterval = 1000;

    m_port     = new QSerialPort(this);
    connect(m_port,SIGNAL(readyRead()),this,SLOT(recv()),Qt::QueuedConnection);

    connectionTimer = new QTimer(this);
    queryStateTimer = new QTimer(this);

    connect(connectionTimer, &QTimer::timeout, [this](){emit noConnection();loseConnection=true;connectionTimer->stop();});
    connect(queryStateTimer, &QTimer::timeout, [this](){queue("PZ\r");});
}
FocusDevicePrior::~FocusDevicePrior()
{
    m_port->close();
    delete m_port;
    delete connectionTimer;
    delete queryStateTimer;
}
void FocusDevicePrior::connectDevice(QString portName)
{
    foreach(const QSerialPortInfo &info, QSerialPortInfo::availablePorts())
        if(info.portName()==portName)
            m_port->setPort(info);

    if(m_port->portName()=="")                                         //找不到对应的端口
        qWarning("%s %s not found.", qPrintable(DeviceName), qPrintable(portName));
    else
    {
        m_port->setBaudRate(QSerialPort::Baud9600);
        m_port->setDataBits(QSerialPort::Data8);
        m_port->setStopBits(QSerialPort::OneStop);
        m_port->setParity(QSerialPort::NoParity);
        m_port->setFlowControl(QSerialPort::NoFlowControl);
        m_port->open(QIODevice::ReadWrite);
        if(!m_port->isOpen())                                                    //无法打开端口
            qWarning("%s %s is Occupied", qPrintable(DeviceName), qPrintable(portName));
        else                                                           //初始化设备参数
        {
            loseConnection = true;
            commandList.clear();
            responseTypes.clear();
            setStepW(0.02);
            setInchingSpeed(100);
        }
    }
    if(!m_port->isOpen())
    {
        emit openPortFailure();
        qDebug() << "Can't connect " + DeviceName;
    }
}
void FocusDevicePrior::queue(QByteArray command, bool b_with_response)
{
    commandList.append(command);
    responseTypes.append(b_with_response);
    if(commandList.size()==1)   //如果只有新加入的指令，直接发送
        send();
    if(commandList.size()>=20)
    {//积压后进行重置
        commandList.clear();
        responseTypes.clear();
    }
}
void FocusDevicePrior::send()
{
    if(m_port->isOpen())
    {
        // 此时队列非空
        QByteArray cmd = commandList.first();
        m_port->write(cmd);
        if(responseTypes.first() && !loseConnection && !connectionTimer->isActive())
            connectionTimer->start(connectionInterval);
    }
    else
        emit noConnection();
}
void FocusDevicePrior::recv()
{
    if(loseConnection)
    {
        isConnected = true;
        loseConnection = false;
        queryStateTimer->start(queryStateInterval);
        emit startConnection();
    }
    connectionTimer->stop();

    if(commandList.empty())
        return;
    QByteArray cmd = commandList.first();
    bool b = responseTypes.first();
    if(b)
    {
        RxBuffer = m_port->readAll();
        msg.append(RxBuffer);
        if(msg.endsWith(endflag))
        {
            if(!commandList.empty())
            {
                commandList.pop_front();
                responseTypes.pop_front();
            }
            //针对不同指令，处理返回的数据
            if(cmd=="PZ\r")
            {
                QByteArray tmp(msg);
                std::reverse(tmp.begin(), tmp.end());
                int pos2 = tmp.indexOf("\r", 1);
                if(pos2>0)
                {
                    msg = tmp.left(pos2);
                    std::reverse(msg.begin(), msg.end());
                }
                raw_pos = msg.toFloat()/500;
                if(raw_pos==0)
                    qDebug()<<cmd<<msg;
                emit updateZPos();
            }
            msg.clear();
        }
    }
    if(!commandList.empty())
        send();
}
void FocusDevicePrior::setStepW(double w)
{
    w=w*10;         //1 step= 250 ustep =0.1um
    std::string tmp1 = std::to_string(w);
    QByteArray tmp2 = QByteArray::fromStdString(tmp1);
    QByteArray setW_QByteArray="C, " + tmp2 + "\r";
    queue(setW_QByteArray);
    // qDebug() << "Focus: set step " << w << " um";
}
void FocusDevicePrior::setInchingSpeed(float vel)
{
    inchingSpeed = vel;
    int v = vel/10;   // v (um/s)转换为指令有缩放倍数
    if(v==0)
        v=1;
    std::string tmp1 = std::to_string(v);
    QByteArray tmp2 = QByteArray::fromStdString(tmp1);
    QByteArray setV_QByteArray="SMZ, " + tmp2 + "\r";
    queue(setV_QByteArray);
}
void FocusDevicePrior::move(float z, bool CONTINUOUS, float vel)
{
    if(vel!=0&&inchingSpeed!=abs(vel))
        setInchingSpeed(abs(vel));
    if(CONTINUOUS)
        moveContinuous(vel);
    else
        moveDistance(z);
}
void FocusDevicePrior::moveDistance(float z)//move z [um]; z>0 : Up; z<0 : Down
{
    int zz = z*500; // 1 um = 500 steps
    if(z>0)
    {
        std::string tmp1 = std::to_string(zz);
        QByteArray tmp2 = QByteArray::fromStdString(tmp1);
        QByteArray moveup_QByteArray="U, " + tmp2 + "\r";
        queue(moveup_QByteArray);
    }
    else
    {
        std::string tmp1 = std::to_string(-zz);
        QByteArray tmp2 = QByteArray::fromStdString(tmp1);
        QByteArray movedown_QByteArray="D, "+tmp2+"\r";
        queue(movedown_QByteArray);
    }
}
void FocusDevicePrior::moveContinuous(float v)
{
    int vv=v*20;// 1 um/s = VZ, 20
    std::string tmp1 = std::to_string(vv);
    QByteArray tmp2 = QByteArray::fromStdString(tmp1);
    QByteArray continuousmove_QByteArray="VZ, "+tmp2+"\r";
    queue(continuousmove_QByteArray);
}
void FocusDevicePrior::stop()
{
    QByteArray stop_QByteArray="I\r";
    commandList.clear();
    responseTypes.clear();
    queue(stop_QByteArray);
//    qDebug() << "Trying to Stop Focus!" ;
}
void FocusDevicePrior::close()
{
    queryStateTimer->stop();
    m_port->close();
    connectionTimer->stop();
    queryStateTimer->stop();
    isConnected = false;
}
