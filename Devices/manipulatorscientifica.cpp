#include "manipulatorscientifica.h"
using namespace std;
ManipulatorScientifica::ManipulatorScientifica(QString name)
{
    DeviceName = name;
    b_same_xdir = true;                  //设备x正方向与规定正方向是否相同
    b_same_ydir = true;                  //设备y正方向与规定正方向是否相同
    b_same_zdir = true;                  //设备z正方向与规定正方向是否相同
    endflag = "\r";
    queryStateInterval = 20;
    queryStateCommand = "P\r";
    connectionInterval = 1000;

    m_port     = new QSerialPort(this);
    connectionTimer = new QTimer(this);
    queryStateTimer = new QTimer(this);

    connect(connectionTimer, &QTimer::timeout, [this](){emit noConnection();loseConnection=true;connectionTimer->stop();});
    connect(queryStateTimer, &QTimer::timeout, [this](){queue(queryStateCommand);});
    connect(m_port,SIGNAL(readyRead()),this,SLOT(recv()),Qt::QueuedConnection);
}
ManipulatorScientifica::~ManipulatorScientifica()
{
    m_port->close();
    delete m_port;
    delete connectionTimer;
    delete queryStateTimer;
}
void ManipulatorScientifica::connectDevice(QString portName)
{
    foreach(const QSerialPortInfo &info, QSerialPortInfo::availablePorts())
    if(info.portName()==portName)
        m_port->setPort(info);

    if(m_port->portName()=="")                                         //找不到对应的端口
        qWarning("%s %s not found.", qPrintable(DeviceName), qPrintable(portName));
    else
    {
        m_port->setBaudRate(QSerialPort::Baud38400);
        m_port->open(QIODevice::ReadWrite);
        if(!m_port->isOpen())                                                    //无法打开端口
            qWarning("%s %s is Occupied", qPrintable(DeviceName), qPrintable(portName));
        else                                                           //初始化设备参数
        {
            commandList.clear();
            responseTypes.clear();
            loseConnection = true;
            queue(queryStateCommand);
        }
    }
    if(!m_port->isOpen())
    {
        emit openPortFailure();
        qDebug() << "Can't connect " + DeviceName;
    }
}
void ManipulatorScientifica::queue(QByteArray command, bool b_with_response)
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
void ManipulatorScientifica::send()
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
void ManipulatorScientifica::recv()
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

        char *message = msg.data();
        string msg_s;
        msg_s=message;
        int n=strlen(message);
        int split_0=-1, split_1=0,split_2=0;
        for(int i=0;i<strlen(message);i++)
        {
            if(message[i]=='\t')
            {
                if(split_1==0)
                {
                    split_1=i;
                }
                else
                {
                    split_2=i;
                }
            }
        }
        for(int i=0; i<split_1; ++i)
            if(message[i]=='\r')
                split_0 = i;
        string x_s,y_s,z_s;
        x_s=msg_s.substr(split_0+1,split_1-split_0-1);
        y_s=msg_s.substr(split_1+1,split_2-split_1-1);
        z_s=msg_s.substr(split_2+1,n-split_2-2);
        int x_pos,y_pos,z_pos;
        x_pos=atoi(x_s.c_str());
        y_pos=atoi(y_s.c_str());
        z_pos=atoi(z_s.c_str());
        raw_xpos=((double)x_pos)/100.0;
        raw_ypos=((double)y_pos)/100.0;
        raw_zpos=((double)z_pos)/100.0;
        emit updatePos();

        msg.clear();
        if(!commandList.empty())
            send();
    }
    else if(msg.size()==1&&msg[0]==char(255))
    {
        emit noConnection();
        loseConnection=true;
    }
}

void ManipulatorScientifica::setInchingSpeed(float vel)
{
    inchingSpeed = vel;
    std::string str = "TOP " + std::to_string(int(vel)) + "\r";
    queue(QByteArray::fromStdString(str));
}

void ManipulatorScientifica::moveX(float x, float vel, bool CONTINUOUS)        //x轴相对运动
{
    if(vel>inchingSpeed)
        setInchingSpeed(vel);
    vel*=100;           //指令是百分之一微米
    x*=100;             //指令是百分之一微米
    if(CONTINUOUS)
    {
        std::string str = "VJ " + std::to_string(int(vel)) + " 0 0\r";
        queue(QByteArray::fromStdString(str));
    }
    else
    {
        std::string str = "CREEP X " + std::to_string((int)x) + " " + std::to_string((int)abs(vel)) + "\r";
        queue(QByteArray::fromStdString(str));
    }
}

void ManipulatorScientifica::moveY(float y, float vel, bool CONTINUOUS)        //y轴相对运动
{
    if(vel>inchingSpeed)
        setInchingSpeed(vel);
    vel*=100;           //指令是百分之一微米
    y*=100;             //指令是百分之一微米
    if(CONTINUOUS)
    {
        std::string str = "VJ 0 " + std::to_string(int(vel)) + " 0\r";
        queue(QByteArray::fromStdString(str));
    }
    else
    {
        std::string str = "CREEP Y " + std::to_string((int)y) + " " + std::to_string((int)abs(vel)) + "\r";
        queue(QByteArray::fromStdString(str));
    }
}

void ManipulatorScientifica::moveZ(float z, float vel, bool CONTINUOUS)        //z轴相对运动
{
    if(vel>inchingSpeed)
        setInchingSpeed(vel);
    vel*=100;           //指令是百分之一微米
    z*=100;            //指令是百分之一微米
    if(CONTINUOUS)
    {
        std::string str = "VJ 0 0 " + std::to_string(int(vel)) + "\r";
        queue(QByteArray::fromStdString(str));
    }
    else
    {
        std::string str = "CREEP Z " + std::to_string((int)z) + " " + std::to_string((int)abs(vel)) + "\r";
        queue(QByteArray::fromStdString(str));
    }
}

void ManipulatorScientifica::move(float x, float y, float z, float v)                  //任意方向相对运动
{
    x*=100;    y*=100;    z*=100;
    std::string str = "REL " + std::to_string((int)x) + " " + std::to_string((int)y) + " " + std::to_string((int)z) + " " + "\r";

    if(v!=0 && v!=inchingSpeed)
        setInchingSpeed(v);
    queue(QByteArray::fromStdString(str));

    // if(v!=0 && v!=inchingSpeed)
    // {
    //     inchingSpeed = v;
    //     std::string str = "TOP " + std::to_string(int(v)) + "\r";
    //     m_port->write(QByteArray::fromStdString(str));QThread::msleep(500);
    // }
    // float x0, y0, z0, x1, y1, z1, dx, dy, dz;
    // x0 = getAbsX();
    // y0 = getAbsY();
    // z0 = getAbsZ();

    // int cmd_x = x*100;
    // int cmd_y = y*100;
    // int cmd_z = z*100;

    // std::string str = "REL " + std::to_string(cmd_x) + " " + std::to_string(cmd_y) + " " + std::to_string(cmd_z) + " " + "\r";

    // queryStateTimer->stop();
    // connectionTimer->stop();
    // m_port->clear();
    // commandList.clear();
    // responseTypes.clear();

    // m_port->write(QByteArray::fromStdString(str));
    // QDateTime startTime=  QDateTime::currentDateTime();
    // QTime time = QTime::currentTime().addMSecs(3000);
    // bool isMoveFinished = false;
    // while(!isMoveFinished && QTime::currentTime() < time)
    // {
    //     msg.clear();
    //     m_port->write(queryStateCommand);
    //     while(!m_port->waitForReadyRead(20)&& QTime::currentTime() < time)//只有当有数据可供读取时停止循环 (waitForReadyRead 返回 true)
    //     {
    //     }
    //     RxBuffer = m_port->readAll();
    //     msg.append(RxBuffer);
    //     if(msg.endsWith(endflag) && msg.size()>=19)
    //     {
    //         QRegularExpression re(".+\t.+\t.+\r");
    //         // QRegularExpression re("\\x00\r");
    //         QRegularExpressionMatch reMatch;
    //         QString matched;
    //         reMatch = re.match(msg);
    //         if(!reMatch.hasMatch())
    //             continue;
    //         matched = reMatch.captured(0);

    //         string msg_s = matched.toStdString();
    //         int n=msg_s.size();
    //         int split_0=-1, split_1=0,split_2=0;
    //         for(int i=0;i<n;i++)
    //         {
    //             if(msg_s[i]=='\t')
    //             {
    //                 if(split_1==0)
    //                     split_1=i;
    //                 else
    //                     split_2=i;
    //             }
    //         }
    //         for(int i=0; i<split_1; ++i)
    //             if(msg_s[i]=='\r'||msg_s[i]=='\t')
    //                 split_0 = i;
    //         string x_s,y_s,z_s;
    //         x_s=msg_s.substr(split_0+1,split_1-split_0-1);
    //         y_s=msg_s.substr(split_1+1,split_2-split_1-1);
    //         z_s=msg_s.substr(split_2+1,n-split_2-2);
    //         int x_pos,y_pos,z_pos;
    //         x_pos=atoi(x_s.c_str());
    //         y_pos=atoi(y_s.c_str());
    //         z_pos=atoi(z_s.c_str());
    //         raw_xpos=((double)x_pos)/100.0;
    //         raw_ypos=((double)y_pos)/100.0;
    //         raw_zpos=((double)z_pos)/100.0;

    //         x1 = getAbsX();
    //         y1 = getAbsY();
    //         z1 = getAbsZ();
    //         dx = x1-x0;
    //         dy = y1-y0;
    //         dz = z1-z0;
    //         emit updatePos();
    //         float eps = 0.02;
    //         if(abs(dx-x)<eps && abs(dy-y)<eps && abs(dz-z)<eps)
    //         {
    //             isMoveFinished = true;
    //             QDateTime endTime=  QDateTime::currentDateTime();
    //             qint64 intervalTimeMS = startTime.msecsTo(endTime);
    //             // qDebug()<< "dx = "<<dx<<"; dy = "<<dy<<"; dz = "<<dz;
    //         }
    //     }
    // }
    // qDebug()<<dx-x<<dy-y<<dz-z;
    // if(!isMoveFinished)
    // {
    //     qDebug()<<"Fail!";
    //     // qDebug()<<msg<<int(x0*100)+cmd_x<<int(y0*100)+cmd_y<<int(z0*100)+cmd_z;
    //     // qDebug()<<dx<<x<<dy<<y<<dz<<z;
    // }
    // queryStateTimer->start(queryStateInterval);
    // // QDateTime endTime=  QDateTime::currentDateTime();
    // // qint64 intervalTimeMS = startTime.msecsTo(endTime);
    // // qDebug()<<DeviceName<<" end time: "<<intervalTimeMS<<" ms";
}

void ManipulatorScientifica::stop()
{
    queue("CREEP X 0\r");
    queue("CREEP Y 0\r");
    queue("CREEP Z 0\r");
}
void ManipulatorScientifica::close()
{
    m_port->close();
    connectionTimer->stop();
    queryStateTimer->stop();
    msg.clear();
    commandList.clear();
    responseTypes.clear();
    isConnected = false;
}
