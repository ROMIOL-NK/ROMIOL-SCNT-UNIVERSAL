#include "syringemotorvince.h"
SyringeMotorVince::SyringeMotorVince(int ppR, QString name)
{
    DeviceName = name;
    begflag = QByteArray::fromHex("FF");
    endflag = QByteArray::fromHex("FE");
    queryStateInterval = 200;
    connectionInterval = 1000;
    pulsePerRound = ppR;
    inchingSpeed = 0.01;
    b_same_dir = true;                  //设备正方向与规定正方向相同
    ori = 0;
    upperBound = 14.0;
    lowerBound = -14.0;
    m_port     = new QSerialPort(this);
    connectionTimer = new QTimer(this);
    queryStateTimer = new QTimer(this);

    connect(connectionTimer, &QTimer::timeout, [this](){emit noConnection();loseConnection=true;connectionTimer->stop();});
    connect(queryStateTimer, &QTimer::timeout, [this](){queue("1 sts\n");});
    connect(m_port,SIGNAL(readyRead()),this,SLOT(recv()),Qt::QueuedConnection);
}
SyringeMotorVince::~SyringeMotorVince()
{
    m_port->close();
    delete m_port;
    delete connectionTimer;
    delete queryStateTimer;
}

void SyringeMotorVince::connectDevice(QString portName)
{
    if(m_port->isOpen())
        m_port->close();
    if(isConnected)
        close();
    foreach(const QSerialPortInfo &info, QSerialPortInfo::availablePorts())
        if(info.portName()==portName)
            m_port->setPort(info);

    if(m_port->portName()=="")                                         //找不到对应的端口
        qWarning("%s %s not found.", qPrintable(DeviceName), qPrintable(portName));
    else
    {
        m_port->setBaudRate(QSerialPort::Baud9600);
        m_port->open(QIODevice::ReadWrite);
        if(!m_port->isOpen())                                                    //无法打开端口
            qWarning("%s %s is Occupied", qPrintable(DeviceName), qPrintable(portName));
        else                                                           //初始化设备参数
        {
            commandList.clear();
            responseTypes.clear();
            msg.clear();
            loseConnection = true;
            queue("1 ena\n");
        }
    }
    if(!m_port->isOpen())
    {
        emit openPortFailure();
        qDebug() << "Can't connect " + DeviceName;
    }
}
void SyringeMotorVince::move(float r, float vel, bool CONTINUOUS)
{
    int iPos, iSpd; //整数脉冲量 位置、速度
    iPos = r*pulsePerRound;
    iSpd = vel*pulsePerRound;
    if(CONTINUOUS)
    {
        std::string str = "1 mov spd="+std::to_string(iSpd)+"\n";
        queue(QByteArray::fromStdString(str));
    }
    else
    {
        if(iSpd==0)
            iSpd = inchingSpeed*pulsePerRound;//设置为非 0 速度
        std::string str = "1 rmv "+std::to_string(iPos) + " spd="+std::to_string(iSpd)+"\n";
        queue(QByteArray::fromStdString(str));
    }
}
void SyringeMotorVince::suck(float disp, float vel, bool CONTINUOUS)
{
    move(disp, vel, CONTINUOUS);
}
void SyringeMotorVince::blow(float disp, float vel, bool CONTINUOUS)
{
    move(-disp, -vel, CONTINUOUS);
}

void SyringeMotorVince::stop()
{
    // 原逻辑仅将停止命令追加到队尾，队列积压时不能及时停止：
    // queue("1 stp\n");
    // 当前队首命令已经发出，必须保留并等待其回复；否则该回复会被误认为停止命令的回复，造成队列错位。
    if(commandList.empty())
    {
        queue("1 stp\n");
        return;
    }

    QByteArray currentCommand = commandList.first();
    bool currentResponseType = responseTypes.empty()?true:responseTypes.first();
    commandList.clear();
    responseTypes.clear();
    commandList.append(currentCommand);
    responseTypes.append(currentResponseType);
    if(currentCommand!="1 stp\n")
    {
        commandList.append("1 stp\n");
        responseTypes.append(true);
    }
}
void SyringeMotorVince::close()
{
    queryStateTimer->stop();
    stop();
    // queue("1 off\n");
    m_port->close();
    connectionTimer->stop();
    isConnected = false;
}
void SyringeMotorVince::setInchingSpeed(float vel)
{
    inchingSpeed = vel;
}
void SyringeMotorVince::queue(QByteArray command, bool b_with_response)
{
    commandList.append(command);
    responseTypes.append(b_with_response);
    if(commandList.size()==1)   //如果只有新加入的指令，直接发送
        send();
    // if(commandList.size()>=20)
    // {//积压后进行重置
    //     commandList.clear();
    //     responseTypes.clear();
    // }
}
void SyringeMotorVince::send()
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
void SyringeMotorVince::recv()
{
    if(loseConnection)
    {
        queue("1 ena\n");
        isConnected = true;
        loseConnection = false;
        queryStateTimer->start(queryStateInterval);
        emit startConnection();
    }
    connectionTimer->stop();//通讯正常,关闭连接状态监测定时器

    if(commandList.empty())//不知道对应的指令，无法解析返回数据
        return;
    QByteArray cmd = commandList.first();
    RxBuffer = m_port->readAll();
    msg.append(RxBuffer);
    if(msg.size()>21)
        msg.clear();
    if(msg.size()==21 && msg.startsWith(begflag)&&msg.endsWith(endflag))
    {
        if(!commandList.empty())
        {
            commandList.pop_front();
            responseTypes.pop_front();
        }
        //针对不同指令，处理返回的数据
        convert(msg);
        msg.clear();
        if(getRelPos() >= upperBound || getRelPos()<=lowerBound)
        {
            commandList.clear();
            responseTypes.clear();
            queue("1 stp\n");                           //如果位置达到限位，停止运动
        }
        if(!commandList.empty())
            send();
        emit updatePos();
    }
}
int SyringeMotorVince::convert(QByteArray msg)
{
    int num=0, s=8;
    num |= msg[s];
    num <<= 7;
    num |= msg[s+1];
    num <<= 7;
    num |= msg[s+2];
    num <<= 7;
    num |= msg[s+3];
    num <<= 7;
    num |= msg[s+4];
    raw_pos = 1.0*num/pulsePerRound;

    s = 3;
    uint32_t udata;
    udata = msg[s];
    udata <<= 7;
    udata |= msg[s+1];
    udata <<= 7;
    udata |= msg[s+2];
    udata <<= 7;
    udata |= msg[s+3];
    udata <<= 7;
    udata |= msg[s+4];
    raw_vel = *(float*)&udata/pulsePerRound;
    return raw_vel;
}
