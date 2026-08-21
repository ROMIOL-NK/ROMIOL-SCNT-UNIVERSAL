#include "ManipulatorScientificaMotion8.h"
using namespace std;
ManipulatorScientificaMotion8::ManipulatorScientificaMotion8(QString name)
{
    DeviceName = name;
    b_same_xdir = true;                  //设备正方向与规定正方向是否相同
    b_same_ydir = false;                  //设备正方向与规定正方向是否相同
    b_same_zdir = false;                  //设备正方向与规定正方向是否相同
    endflag = "\0";
    queryStateInterval = 200;
    QByteArray originalData = QByteArray::fromHex("AA0005");
    queryStateCommand = cobsEncode(originalData);

    m_port     = new QSerialPort(this);
    timer = new QTimer(this);
    queryStateTimer = new QTimer(this);

    connect(timer, &QTimer::timeout, [this](){emit noConnection();});
    connect(queryStateTimer, &QTimer::timeout, [this](){queue(queryStateCommand);});
    connect(m_port,SIGNAL(readyRead()),this,SLOT(recv()),Qt::QueuedConnection);
}
ManipulatorScientificaMotion8::~ManipulatorScientificaMotion8()
{
    m_port->close();
    delete m_port;
    delete timer;
    delete queryStateTimer;
}

qint32 ManipulatorScientificaMotion8::littleEndian4ToUInt(const QByteArray &data, int startIndex)
{
    // 1. 按小端序组合成 32位无符号数（避免符号位提前干扰）
    uint32_t uVal = 0;
    uVal |= static_cast<uint8_t>(data[startIndex]);       // 第1字节（LSB）
    uVal |= static_cast<uint8_t>(data[startIndex + 1]) << 8;
    uVal |= static_cast<uint8_t>(data[startIndex + 2]) << 16;
    uVal |= static_cast<uint8_t>(data[startIndex + 3]) << 24; // 第4字节（MSB，含符号位）

    // 2. 强制转换为 32位有符号数（自动正确处理负数）
    return static_cast<qint32>(uVal);
}

QByteArray ManipulatorScientificaMotion8::convertTo4Bytes(int value)
{
    QByteArray byteArray;

    // 按小端模式依次提取每个字节
    byteArray.append(static_cast<char>(value & 0xFF));
    byteArray.append(static_cast<char>((value >> 8) & 0xFF));
    byteArray.append(static_cast<char>((value >> 16) & 0xFF));
    byteArray.append(static_cast<char>((value >> 24) & 0xFF));

    return byteArray;
}

QByteArray ManipulatorScientificaMotion8::convertTo8Bytes(int value)
{
    QByteArray byteArray;

    // 按小端模式依次提取每个字节（8字节）
    byteArray.append(static_cast<char>(value & 0xFF));
    byteArray.append(static_cast<char>((value >> 8) & 0xFF));
    byteArray.append(static_cast<char>((value >> 16) & 0xFF));
    byteArray.append(static_cast<char>((value >> 24) & 0xFF));
    byteArray.append(static_cast<char>((value >> 32) & 0xFF));
    byteArray.append(static_cast<char>((value >> 40) & 0xFF));
    byteArray.append(static_cast<char>((value >> 48) & 0xFF));
    byteArray.append(static_cast<char>((value >> 56) & 0xFF));

    return byteArray;
}

QByteArray ManipulatorScientificaMotion8::cobsEncode(const QByteArray &data)
{
    if (data.isEmpty()) {
        return QByteArray(1, 0x01);
    }

    QByteArray encoded;
    encoded.resize(data.size() + data.size() / 254 + 2);

    quint8 *dest = reinterpret_cast<quint8*>(encoded.data());
    const quint8 *src = reinterpret_cast<const quint8*>(data.constData());
    const quint8 *end = src + data.size();

    quint8 *code_ptr = dest++;
    quint8 code = 0x01;

    while (src < end) {
        if (*src == 0x00) {
            *code_ptr = code;
            code_ptr = dest++;
            code = 0x01;
        } else {
            *dest++ = *src;
            code++;
            if (code == 0xFF) {
                *code_ptr = code;
                code_ptr = dest++;
                code = 0x01;
            }
        }
        src++;
    }
    *code_ptr = code;
    *dest++ = 0x00;
    encoded.resize(dest - reinterpret_cast<quint8*>(encoded.data()));

    return encoded;
}

QByteArray ManipulatorScientificaMotion8::cobsDecode(const QByteArray &encodedData) {
    if (encodedData.isEmpty()) {
        return QByteArray();
    }
    if (encodedData.at(encodedData.size() - 1) != 0x00) {
        qWarning() << "COBS decode error: Frame does not end with 0x00";
        return QByteArray();
    }

    QByteArray decoded;
    decoded.resize(encodedData.size() - 2);

    const quint8 *src = reinterpret_cast<const quint8*>(encodedData.constData());
    const quint8 *end = src + encodedData.size() - 1; // 不包括最后的0x00
    quint8 *dest = reinterpret_cast<quint8*>(decoded.data());

    while (src < end) {
        quint8 code = *src++;

        if (code == 0x00) {
            qWarning() << "COBS decode error: Invalid zero code byte";
            return QByteArray();
        }
        for (quint8 i = 1; i < code && src < end; i++) {
            *dest++ = *src++;
        }
        if (src < end && code != 0xFF) {
            *dest++ = 0x00;
        }
    }
    decoded.resize(dest - reinterpret_cast<quint8*>(decoded.data()));

    return decoded;
}

void ManipulatorScientificaMotion8::connectDevice(QString portName)
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
            // setInchingSpeed(1000);
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
void ManipulatorScientificaMotion8::queue(QByteArray command, bool b_with_response)
{
    commandList.append(command);
    responseTypes.append(b_with_response);
    if(commandList.size()==1)   //如果只有新加入的指令，直接发送
        send();
}
void ManipulatorScientificaMotion8::send()
{
    if(m_port->isOpen())
    {
        // 此时队列非空
        QByteArray cmd = commandList.first();
        m_port->write(cmd);
        //        m_port->flush();
        if(responseTypes.first())
            timer->start(2000);
    }
    else
        emit noConnection();
}
void ManipulatorScientificaMotion8::recv()
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
    if(!RxBuffer.endsWith(endflag))
        return;
    QByteArray decodedData = cobsDecode(RxBuffer);
    if(!commandList.empty())
    {
        commandList.pop_front();
        responseTypes.pop_front();
    }
    //针对不同指令，处理返回的数据
    if(cmd==queryStateCommand)
    {
        qint32 data1 = littleEndian4ToUInt(decodedData, 11);   // 第12~15字节
        qint32 data2 = littleEndian4ToUInt(decodedData, 18);   // 第19~22字节
        qint32 data3 = littleEndian4ToUInt(decodedData, 25);   // 第26~29字节

        raw_xpos = (double)data3 / 100;
        raw_ypos = (double)data2 / 100;
        raw_zpos = (double)data1 / 100;

        emit updatePos();
    }

    msg.clear();
    if(!commandList.empty())
        send();
}

void ManipulatorScientificaMotion8::setInchingSpeed(float v)
{
    // if(abs(v)!=inchingSpeed && v!=0)
    // {
    //     inchingSpeed = abs(v);
    //     QByteArray originalData = QByteArray::fromHex("AA03050000");
    //     QByteArray moveCommand = cobsEncode(originalData + convertTo8Bytes(v));
    //     queue(moveCommand);
    //     // originalData = QByteArray::fromHex("AA03050001");
    //     // moveCommand = cobsEncode(originalData + convertTo8Bytes(v));
    //     // queue(moveCommand);
    //     // originalData = QByteArray::fromHex("AA03050002");
    //     // moveCommand = cobsEncode(originalData + convertTo8Bytes(v));
    //     // queue(moveCommand);
    // }
}

void ManipulatorScientificaMotion8::moveX(float x, float vel, bool CONTINUOUS)        //x轴相对运动
{
    vel*=100;           //指令是百分之一微米
    x*=100;             //指令是百分之一微米
    if(CONTINUOUS)
    {
        QByteArray originalData = QByteArray::fromHex("AA02010000");
        QByteArray moveCommand = cobsEncode(originalData + convertTo4Bytes(vel));
        queue(moveCommand);
    }
    else
    {
        QByteArray originalData = QByteArray::fromHex("AA02020000");
        QByteArray moveCommand = cobsEncode(originalData + convertTo4Bytes(x));
        queue(moveCommand);
    }
}

void ManipulatorScientificaMotion8::moveY(float y, float vel, bool CONTINUOUS)        //y轴相对运动
{
    vel*=-100;           //指令是百分之一微米
    y*=-100;             //指令是百分之一微米
    if(CONTINUOUS)
    {
        QByteArray originalData = QByteArray::fromHex("AA02010001");
        QByteArray moveCommand = cobsEncode(originalData + convertTo4Bytes(vel));
        queue(moveCommand);
    }
    else
    {
        QByteArray originalData = QByteArray::fromHex("AA02020001");
        QByteArray moveCommand = cobsEncode(originalData + convertTo4Bytes(y));
        queue(moveCommand);
    }
}

void ManipulatorScientificaMotion8::moveZ(float z, float vel, bool CONTINUOUS)        //z轴相对运动
{
    vel*=-100;           //指令是百分之一微米
    z*=-100;            //指令是百分之一微米
    if(CONTINUOUS)
    {
        QByteArray originalData = QByteArray::fromHex("AA02010002");
        QByteArray moveCommand = cobsEncode(originalData + convertTo4Bytes(vel));
        queue(moveCommand);
    }
    else
    {
        QByteArray originalData = QByteArray::fromHex("AA02020002");
        QByteArray moveCommand = cobsEncode(originalData + convertTo4Bytes(z));
        queue(moveCommand);
    }
}

void ManipulatorScientificaMotion8::move(float x, float y, float z, float v)                  //任意方向相对运动
{
    // setInchingSpeed(v);
    x*=100;    y*=-100;    z*=-100;
    QByteArray originalData = QByteArray::fromHex("AA020200");
    QByteArray moveCommand = cobsEncode(originalData + convertTo4Bytes(x) + convertTo4Bytes(y) + convertTo4Bytes(z));
    queue(moveCommand);
}

void ManipulatorScientificaMotion8::stop()
{
    QByteArray originalDatax = QByteArray::fromHex("AA02010000");
    QByteArray moveCommandx = cobsEncode(originalDatax + convertTo4Bytes(0));
    QByteArray originalDatay = QByteArray::fromHex("AA02010001");
    QByteArray moveCommandy = cobsEncode(originalDatay + convertTo4Bytes(0));
    QByteArray originalDataz = QByteArray::fromHex("AA02010002");
    QByteArray moveCommandz = cobsEncode(originalDataz + convertTo4Bytes(0));
    queue(moveCommandx);
    queue(moveCommandy);
    queue(moveCommandz);
}
void ManipulatorScientificaMotion8::close()
{
    m_port->close();
    timer->stop();
    queryStateTimer->stop();
    isConnected = false;
}
