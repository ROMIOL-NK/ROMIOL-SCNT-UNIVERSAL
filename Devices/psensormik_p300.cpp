#include "psensormik_p300.h"

PSensorMIK_P300::PSensorMIK_P300(QString name)
{
    DeviceName = name;
    queryStateInterval = 50;
    connectionInterval = 1000;
    queryStateCommand = QByteArray::fromHex("010300040001C5CB");
    isConnected = false;
    pressure = 0;
    unit = "Pa";
    m_port     = new QSerialPort(this);
    connectionTimer = new QTimer(this);
    queryStateTimer = new QTimer(this);

    connect(connectionTimer, &QTimer::timeout, [this](){emit noConnection();loseConnection=true;connectionTimer->stop();});
    connect(queryStateTimer, &QTimer::timeout, [this](){queue(queryStateCommand);});
    connect(m_port,SIGNAL(readyRead()),this,SLOT(recv()),Qt::QueuedConnection);
}
PSensorMIK_P300::~PSensorMIK_P300()
{
    m_port->close();
    delete m_port;
    delete connectionTimer;
    delete queryStateTimer;
}

int PSensorMIK_P300::convert(QByteArray msg)
{
    bool ok;
    QByteArray byteArray;
    byteArray.append(msg[3+0]);
    byteArray.append(msg[3+1]);
    int num = byteArray.toHex().toInt(&ok, 16);
    if(ok)
    {
        if(num>32767)
            num -= 65536;
        return num;
    }
    else
        return 0;
}
void PSensorMIK_P300::connectDevice(QString portName)
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
void PSensorMIK_P300::close()
{
    queryStateTimer->stop();
    m_port->close();
    isConnected = false;
    connectionTimer->stop();
    // emit noConnection();
}
void PSensorMIK_P300::queue(QByteArray command, bool b_with_response)
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
void PSensorMIK_P300::send()
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
void PSensorMIK_P300::recv()
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
    //翻译返回数据
    if(msg.size()>7)
        msg.clear();
    if(msg.size()==7 && msg[0]==0x01 && msg[1]==0x03 && msg[2]==0x02)
    {
        if(!commandList.empty())
        {
            commandList.pop_front();
            responseTypes.pop_front();
        }
        //针对不同指令，处理返回的数据
        pressure = 10.0*convert(msg);
        msg.clear();
        if(!commandList.empty())
            send();
        emit updatePressure();
    }
}
