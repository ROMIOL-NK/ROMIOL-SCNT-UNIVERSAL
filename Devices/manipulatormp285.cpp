#include "manipulatormp285.h"

ManipulatorMP285::ManipulatorMP285(QString name)
{
    DeviceName = name;
    b_same_xdir = true;                  //设备正方向与规定正方向是否相同
    b_same_ydir = true;                  //设备正方向与规定正方向是否相同
    b_same_zdir = true;                  //设备正方向与规定正方向是否相同
    endflag = "\r";
    queryStateInterval = 200;
    connectionInterval = 2000;

    m_port     = new QSerialPort(this);
    connectionTimer = new QTimer(this);
    queryStateTimer = new QTimer(this);

    connect(connectionTimer, &QTimer::timeout, [this](){emit noConnection();loseConnection=true;connectionTimer->stop();});
    connect(queryStateTimer, &QTimer::timeout, [this](){queue("c\r");});
    connect(m_port,SIGNAL(readyRead()),this,SLOT(recv()),Qt::QueuedConnection);
}
ManipulatorMP285::~ManipulatorMP285()
{
    m_port->close();
    delete m_port;
    delete connectionTimer;
    delete queryStateTimer;
}
void ManipulatorMP285::connectDevice(QString portName)
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
            commandList.clear();
            responseTypes.clear();
            loseConnection = true;
            setVelandRes(3000,0);// 500 um/s,high resolution
        }
    }
    if(!m_port->isOpen())
    {
        emit openPortFailure();
        qDebug() << "Can't connect " + DeviceName;
    }
}
void ManipulatorMP285::queue(QByteArray command, bool b_with_response)
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
void ManipulatorMP285::send()
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
void ManipulatorMP285::recv()
{
    if(loseConnection)
    {
        isConnected = true;
        loseConnection = false;
        queryStateTimer->start(queryStateInterval);
        emit startConnection();
    }
    connectionTimer->stop();//通讯正常,关闭连接状态监测定时器

    if(commandList.empty())
        return;
    QByteArray cmd = commandList.first();
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
        if(cmd=="c\r")
        {
            char* tembuffer = msg.data();
            if(msg.size()==13)
            {
                global_x = *((long*)(&tembuffer[0]));
                global_y = *((long*)(&tembuffer[4]));
                global_z = *((long*)(&tembuffer[8]));
                raw_xpos = global_x/25.0;
                raw_ypos = global_y/25.0;
                raw_zpos = global_z/25.0;
                emit updatePos();
            }
        }
        msg.clear();
        if(!commandList.empty())
            send();
    }
}

void ManipulatorMP285::setVelandRes(long vel,bool res)
{
    // 0000 0000 1000 0000:
    //      →   |👆: Resolution: High
    if (res == 1)//High resolution
        vel = (vel & 0x7FFF) | 0x8000;
    else//low resolution
        vel = vel & 0x7FFF;
    char sCmdSet1 = (unsigned char)((vel & 0xFF00) / 256);
    char sCmdSet2 = (unsigned char)(vel & 0xFF);
    char Buftmp[4];
    Buftmp[0]=0x56;
    //    Buftmp[1]=0xF4;
    //    Buftmp[2]=0x81;// High Resolution,50um/s
    Buftmp[1] = sCmdSet2;
    Buftmp[2] = sCmdSet1;
    Buftmp[3]=0x0D;
    QByteArray setVelandRes_buff=QByteArray(Buftmp,4);
    queue(setVelandRes_buff);
}
void ManipulatorMP285::setInchingSpeed(float v)
{
    setVelandRes(v, 0);
}

void ManipulatorMP285::moveX(float x, float vel, bool CONTINUOUS)        //x轴相对运动
{
    if(vel!=0&&getInchingSpeed()!=abs(vel))
        setInchingSpeed(abs(vel));
    if(!CONTINUOUS)
        move(x, 0, 0);
}

void ManipulatorMP285::moveY(float y, float vel, bool CONTINUOUS)        //y轴相对运动
{
    if(vel!=0&&getInchingSpeed()!=abs(vel))
        setInchingSpeed(abs(vel));
    if(!CONTINUOUS)
        move(0, y, 0);
}

void ManipulatorMP285::moveZ(float z, float vel, bool CONTINUOUS)        //z轴相对运动
{
    if(vel!=0&&getInchingSpeed()!=abs(vel))
        setInchingSpeed(abs(vel));
    if(!CONTINUOUS)
        move(0, 0, z);
}

void ManipulatorMP285::move(float x, float y, float z, float v)                  //任意方向相对运动
{
    if(v!=0 && v!=inchingSpeed)
        setInchingSpeed(v);

    global_x += 25*x;
    global_y += 25*y;
    global_z += 25*z;
    unsigned char sCommand[14];
    memset(sCommand, 0, 14);
    sCommand[0]  = 0x6D;
    sCommand[13] = 0x0D;
    long* plPositionX = (long*)(&sCommand[1]);
    *plPositionX = global_x;
    long* plPositionY = (long*)(&sCommand[5]);
    *plPositionY = global_y;
    long* plPositionZ = (long*)(&sCommand[9]);
    *plPositionZ = global_z;
    QByteArray movetoposition_QByteArray = QByteArray((char*) sCommand, 14);
    queue(movetoposition_QByteArray);
}

void ManipulatorMP285::stop()
{
    unsigned char sCmdSet[2] = { 0x03, 0x0D };
    QByteArray stopmove_buffer = QByteArray::fromRawData((char *)sCmdSet, sizeof(sCmdSet));
    queue(stopmove_buffer);
}
void ManipulatorMP285::close()
{
    m_port->close();
    connectionTimer->stop();
    queryStateTimer->stop();
    msg.clear();
    commandList.clear();
    responseTypes.clear();
    isConnected = false;
}
