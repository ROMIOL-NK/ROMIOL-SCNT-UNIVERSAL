#include "anglesensoraod.h"

AngleSensorAOD::AngleSensorAOD(QString name)
{
    DeviceName = name;
    queryStateInterval = 50;
    queryStateCommand = QByteArray::fromHex("010300030001740A");
    isConnected = false;
    angle = 0;
    m_port     = new QSerialPort(this);
    timer = new QTimer(this);
    queryStateTimer = new QTimer(this);

    connect(timer, &QTimer::timeout, [this](){emit noConnection();});
    connect(queryStateTimer, &QTimer::timeout, [this](){queue(queryStateCommand);});
    connect(m_port,SIGNAL(readyRead()),this,SLOT(recv()),Qt::QueuedConnection);
}
AngleSensorAOD::~AngleSensorAOD()
{
    m_port->close();
    delete m_port;
    delete timer;
    delete queryStateTimer;
}

double AngleSensorAOD::calculateAngle(QByteArray msg)
{
    if(msg.size() != 7 || msg[0] != 0x01 || msg[1] != 0x03 || msg[2] != 0x02)
    {
        qWarning("Angle Sensor Invalid data frame format!");
        return 0.0;
    }

    unsigned char highByte = msg[3];
    unsigned char lowByte = msg[4];
    unsigned short value = (static_cast<unsigned short>(highByte) << 8) | lowByte;
    double actualAngle = static_cast<double>(value) * 360.0 / 1024.0;
    return actualAngle;
}

void AngleSensorAOD::connectDevice(QString portName)
{
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
            testMode = true;
            queue(queryStateCommand);
        }
    }
    if(!m_port->isOpen())
    {
        emit openPortFailure();
        qDebug() << "Can't connect " + DeviceName;
    }
}
void AngleSensorAOD::close()
{
    queryStateTimer->stop();
    m_port->close();
    isConnected = false;
    timer->stop();
    // emit noConnection();
}
void AngleSensorAOD::queue(QByteArray command, bool b_with_response)
{
    commandList.append(command);
    responseTypes.append(b_with_response);
    if(commandList.size()==1)   //如果只有新加入的指令，直接发送
        send();
}
void AngleSensorAOD::send()
{
    if(m_port->isOpen())
    {
        // 此时队列非空
        QByteArray cmd = commandList.first();
        m_port->write(cmd);
        if(responseTypes.first())
            timer->start(1000);
    }
    else
        emit noConnection();
}
void AngleSensorAOD::recv()
{
//    if(testMode)//测试模式下，接受到返回信息说明连接已经建立
    {
        isConnected = true;
        testMode = false;
        queryStateTimer->start(queryStateInterval);
        emit startConnection();
    }
    timer->stop();//通讯正常,关闭连接状态监测定时器

    if(commandList.empty())
        return;
    QByteArray cmd = commandList.first();
    RxBuffer = m_port->readAll();
    msg.append(RxBuffer);
    if(msg.size()>7)
        msg.clear();
    if(msg.size() == 7 && msg[0] == 0x01 && msg[1] == 0x03 && msg[2] == 0x02)
    {
        if(!commandList.empty())
        {
            commandList.pop_front();
            responseTypes.pop_front();
        }
        angle = calculateAngle(msg);
        // qDebug()<<msg<<" "<<angle;
        msg.clear();
        if(!commandList.empty())
            send();
        currentAngle = angle;
        emit updateAngle();
    }
}
