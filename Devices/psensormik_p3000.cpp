#include "psensormik_p3000.h"

PSensorMIK_P3000::PSensorMIK_P3000(QString name)
{
    DeviceName = name;
    queryStateInterval = 50;
    connectionInterval = 1000;
    queryStateCommand = QByteArray::fromHex("01040000000271CB");
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
PSensorMIK_P3000::~PSensorMIK_P3000()
{
    m_port->close();
    delete m_port;
    delete connectionTimer;
    delete queryStateTimer;
}
void PSensorMIK_P3000::setUnit(QString Unit)
{
    if(Unit=="Pa")
        queue(QByteArray::fromHex("01060000000BC80D"));
    else if(Unit=="kPa")
        queue(QByteArray::fromHex("01060000000C89CF"));
    else if(Unit=="atm")
        queue(QByteArray::fromHex("01060000000E080E"));
    else if(Unit=="psi")
        queue(QByteArray::fromHex("01060000000609C8"));
    else if(Unit=="bar")
        queue(QByteArray::fromHex("010600000007C808"));
}
void PSensorMIK_P3000::connectDevice(QString portName)
{
    foreach(const QSerialPortInfo &info, QSerialPortInfo::availablePorts())
        if(info.portName()==portName)
            m_port->setPort(info);

    if(m_port->portName()=="")                                         //找不到对应的端口
        qWarning("%s %s not found.", qPrintable(DeviceName), qPrintable(portName));
    else
    {
        m_port->setBaudRate(QSerialPort::Baud9600);
        m_port->setParity(QSerialPort::EvenParity);
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
void PSensorMIK_P3000::close()
{
    queryStateTimer->stop();
    m_port->close();
    isConnected = false;
    connectionTimer->stop();
    // emit noConnection();
}
void PSensorMIK_P3000::queue(QByteArray command, bool b_with_response)
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
void PSensorMIK_P3000::send()
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
void PSensorMIK_P3000::recv()
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
    if(cmd==QByteArray::fromHex("01040000000271CB"))
    {
        if(msg.size()>=9)//完整接受到返回信息
        {
            // qDebug()<<"long: "<<msg;
            if(msg[0]==0x01 && msg[1]==0x04)//返回信息的格式是否正确
            {
                QByteArray byteArray;
                byteArray.append(msg[3+2]);
                byteArray.append(msg[3+3]);
                byteArray.append(msg[3+0]);
                byteArray.append(msg[3+1]);
                bool sign = byteArray[0]&0x80;  // 32 bit -> IEEE 754 float bit[0]: sign, bit[1-8]: exponent+127, bit[9-31]: mantissa (尾数）
                int exp = ((byteArray[0]&0x7f)<<1) + ((byteArray[1]&0x80)>>7) - 127;//注意 QByteArray[]为 char类型，直接转 int 会产生负数 (>=128时)
                int mantissa = ((byteArray[1]&0x7f)<<16) + ((byteArray[2]&0xff)<<8) + (byteArray[3]&0xff) + (0x01<<23);//这里通过与操作转换为 int
                pressure = mantissa*pow(2, exp-23) * pow(-1, sign);
                emit updatePressure();
                msg.remove(0, 9);
            }
            else
            {
                msg.clear();
            }
            commandList.pop_front();
            responseTypes.pop_front();
            if(!commandList.empty())
                send();
        }
        else
        {
            // qDebug()<<"short: "<<msg;
        }
    }
    else if(cmd==QByteArray::fromHex("010300040001C5CB"))
    {
        if(msg.size()>2)//完整接受到返回信息
        {
            if(msg[0]==0x01 && msg[1]==0x03)//返回信息的格式是否正确
            {
                QByteArray byteArray;
                byteArray.append(msg[2+0]);
                byteArray.append(msg[2+1]);
                pressure = byteArray.toInt();
                emit updatePressure();
                msg.clear();
            }
        }
    }
    else if(cmd==QByteArray::fromHex("01060000000BC80D"))
    {
        if(msg==QByteArray::fromHex("01060000000BC80D"))//完整接受到返回信息
        {
            unit = "Pa";
            emit updateUnit();
            msg.clear();
       }
//        if(msg.size()==7)//完整接受到返回信息
//        {
//            if(msg[0]==0x01 && msg[1]==0x06)//返回信息的格式是否正确
//            {
//                QByteArray byteArray;
//                byteArray.append(msg[3+0]);
//                byteArray.append(msg[3+1]);
//                int index = byteArray.toInt();
//                switch (index) {
//                case 6:
//                    unit = "psi";
//                    break;
//                case 7:
//                    unit = "bar";
//                    break;
//                case 11:
//                    unit = "Pa";
//                    break;
//                case 12:
//                    unit = "kPa";
//                    break;
//                case 14:
//                    unit = "atm";
//                    break;
//                default:
//                    break;
//                }
//                emit updatePressure();
//            }
//            else
//                msg.clear();
//        }
    }
    else if(cmd==QByteArray::fromHex("01060000000C89CF"))
    {
        if(msg==QByteArray::fromHex("01060000000C89CF"))//完整接受到返回信息
        {
            unit = "kPa";
            emit updateUnit();
            msg.clear();
        }
    }
    else if(cmd==QByteArray::fromHex("01060000000E080E"))
    {
        if(msg==QByteArray::fromHex("01060000000E080E"))//完整接受到返回信息
        {
            unit = "atm";
            emit updateUnit();
            msg.clear();
        }
    }
    else if(cmd==QByteArray::fromHex("01060000000609C8"))
    {
        if(msg==QByteArray::fromHex("01060000000609C8"))//完整接受到返回信息
        {
            unit = "psi";
            emit updateUnit();
            msg.clear();
        }
    }
    else if(cmd==QByteArray::fromHex("010600000007C808"))
    {
        if(msg==QByteArray::fromHex("010600000007C808"))//完整接受到返回信息
        {
            unit = "bar";
            emit updateUnit();
            msg.clear();
        }
    }
}
