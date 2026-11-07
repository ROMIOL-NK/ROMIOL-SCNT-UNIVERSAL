#include "manipulatorhuayue.h"

#include <QDebug>
#include <QElapsedTimer>
#include <QThread>
#include <QtMath>

#include <limits>

namespace {

const int AxisX = 0;
const int AxisY = 1;
const int AxisZ = 2;

const int DefaultSpeedUmPerSecond = 100;
const int ModbusTimeoutMs = 100;
const int PollIntervalMs = 20;
const int EnableSettleMs = 20;
const int MinimumRequestIntervalMs = 10;
const qint32 ContinuousTargetOffsetUm = 1000000;

// 与厂家 MMS_modbus_QTdemo/modbus/modbusmaster.cpp 完全相同的寄存器。
const quint16 RegModbusEnable = 0x0000;
const quint16 RegSystemCommand = 0x0001;
const quint16 RegSystemStatus = 0x0002;
const quint16 RegAxisMode = 0x0010;
const quint16 RegTargetPositionUm = 0x0020;
const quint16 RegMaximumSpeedUm = 0x0026;
const quint16 RegRealtimeUm = 0x0050;

const quint8 FunctionRead = 0x03;
const quint8 FunctionWriteOne = 0x06;
const quint8 FunctionWriteMultiple = 0x10;

const quint16 CommandStopX = 0x0001;
const quint16 CommandStopY = 0x0002;
const quint16 CommandStopZ = 0x0003;

} // namespace

ManipulatorHuayue::ManipulatorHuayue(QString name, quint8 slaveAddress,
                                     int xDirection, int yDirection,
                                     int zDirection)
    : Manipulator(name),
      m_serial(new QSerialPort(this)),
      m_pollTimer(new QTimer(this)),
      m_slaveAddress(slaveAddress),
      m_pollStatusNext(false),
      m_closing(false)
{
    b_same_xdir = xDirection >= 0;
    b_same_ydir = yDirection >= 0;
    b_same_zdir = zDirection >= 0;
    raw_xpos = raw_ypos = raw_zpos = 0;
    inchingSpeed = DefaultSpeedUmPerSecond;

    // -1 表示模式未知。运动前必须写入并读回确认真实模式。
    m_axisModes[AxisX] = -1;
    m_axisModes[AxisY] = -1;
    m_axisModes[AxisZ] = -1;

    m_pollTimer->setInterval(PollIntervalMs);
    m_pollTimer->setTimerType(Qt::PreciseTimer);
    connect(m_pollTimer, &QTimer::timeout,
            this, &ManipulatorHuayue::pollDevice);
    connect(m_serial, &QSerialPort::errorOccurred,
            this, &ManipulatorHuayue::handleSerialError);
}

ManipulatorHuayue::~ManipulatorHuayue()
{
    m_pollTimer->stop();
    if(m_serial->isOpen())
        m_serial->close();
}

void ManipulatorHuayue::connectDevice(QString portName)
{
    close();

    portName = portName.trimmed();
    if(portName.isEmpty()) {
        qWarning() << DeviceName << "连接失败：串口名称为空";
        emit openPortFailure();
        return;
    }

    // 以下串口参数和厂家 Demo 的 ModbusMaster::open() 一致。
    m_serial->setPortName(portName);
    m_serial->setBaudRate(QSerialPort::Baud115200);
    m_serial->setDataBits(QSerialPort::Data8);
    m_serial->setParity(QSerialPort::NoParity);
    m_serial->setStopBits(QSerialPort::OneStop);
    m_serial->setFlowControl(QSerialPort::NoFlowControl);

    if(!m_serial->open(QIODevice::ReadWrite)) {
        qWarning() << DeviceName << "串口打开失败："
                   << m_serial->errorString();
        emit openPortFailure();
        return;
    }

    m_pollTimer->stop();
    m_pollStatusNext = false;
    m_requestTimer.invalidate();
    m_axisModes[AxisX] = -1;
    m_axisModes[AxisY] = -1;
    m_axisModes[AxisZ] = -1;

    // 主项目没有 Demo 的“开启 Modbus”按钮，因此连接后自动执行同一调用。
    if(!enableModbus(true)) {
        qWarning() << DeviceName << "开启 Modbus 控制失败：" << m_lastError;
        m_serial->close();
        emit openPortFailure();
        return;
    }

    // Demo 中“连接”和“开启 Modbus”是两次人工操作；这里留出短暂稳定时间。
    QThread::msleep(EnableSettleMs);

    qint32 position[3] = {0, 0, 0};
    qint32 speed[3] = {0, 0, 0};
    if(!readPositionsAndSpeedsUm(position, speed)) {
        qWarning() << DeviceName << "首次读取位置失败：" << m_lastError;
        enableModbus(false);
        m_serial->close();
        emit openPortFailure();
        return;
    }

    isConnected = true;
    updatePosition(position);
    m_pollTimer->start();
    qInfo() << DeviceName << "连接成功" << portName;
    emit startConnection();
}

void ManipulatorHuayue::close()
{
    m_pollTimer->stop();
    isConnected = false;

    if(m_serial->isOpen()) {
        m_closing = true;
        enableModbus(false);
        m_serial->close();
        m_closing = false;
    }

    m_axisModes[AxisX] = -1;
    m_axisModes[AxisY] = -1;
    m_axisModes[AxisZ] = -1;
}

void ManipulatorHuayue::modbus_enable(bool is_manual)
{
    if (is_manual)
    {
        if (!enableModbus(false))
        {
            qWarning() << DeviceName << "关闭 Modbus 控制失败：" << m_lastError;
            return;
        }
    }
    if (!is_manual)
    {
        if(!enableModbus(true))
        {
            qWarning() << DeviceName << "开启 Modbus 控制失败：" << m_lastError;
            return;
        }
    }
}

void ManipulatorHuayue::moveX(float x, float vel, bool continuous)
{
    moveAxis(AxisX, x, vel, continuous);
}

void ManipulatorHuayue::moveY(float y, float vel, bool continuous)
{
    moveAxis(AxisY, y, vel, continuous);
}

void ManipulatorHuayue::moveZ(float z, float vel, bool continuous)
{
    moveAxis(AxisZ, z, vel, continuous);
}

void ManipulatorHuayue::moveAxis(int axis, float distance,
                                 float speed, bool continuous)
{
    if(!isConnected || axis < AxisX || axis > AxisZ)
        return;

    bool ok = false;
    if(continuous) {
        const float selectedSpeed = !qFuzzyIsNull(speed)
                ? speed
                : (distance >= 0 ? inchingSpeed : -inchingSpeed);
        bool speedValid = false;
        const qint32 deviceSpeed = protocolValue(
                    applyAxisDirection(selectedSpeed, axis), &speedValid);
        if(!speedValid)
            return;
        ok = moveContinuousAxis(axis, deviceSpeed);
    } else {
        bool distanceValid = false;
        bool speedValid = false;
        const qint32 deviceDistance = protocolValue(
                    applyAxisDirection(distance, axis), &distanceValid);
        const float selectedSpeed = (!qFuzzyIsNull(speed) && qIsFinite(speed))
                ? qAbs(speed) : inchingSpeed;
        const qint32 deviceSpeed = protocolValue(selectedSpeed, &speedValid);
        if(!distanceValid || !speedValid ||
           deviceDistance == 0 || deviceSpeed <= 0) {
            return;
        }
        ok = movePointAxis(axis, deviceDistance, deviceSpeed);
    }

    if(!ok)
        qWarning() << DeviceName << "运动命令失败：" << m_lastError;
}

bool ManipulatorHuayue::movePointAxis(int axis, qint32 distance,
                                      qint32 speed)
{
    // 厂家 Demo 的相对位移寄存器 0x002C 在实机上不能驱动电机；
    // 先读实际位置，将高层的相对点动换算为已验证可用的绝对目标位置。
    // 模式切换沿用 Demo 的做法：写成功即更新缓存，不额外插入模式读回。
    if(m_axisModes[axis] != 0) {
        if(!writeAxisMode(axis, 0)) {
            const QString modeError = m_lastError;
            systemCommand(quint16(CommandStopX + axis));
            m_lastError = modeError;
            return false;
        }
        m_axisModes[axis] = 0;
    }

    qint32 position[3] = {0, 0, 0};
    qint32 currentSpeed[3] = {0, 0, 0};
    if(!readPositionsAndSpeedsUm(position, currentSpeed))
        return false;
    updatePosition(position);

    const qint64 target64 = qint64(position[axis]) + qint64(distance);
    if(target64 < std::numeric_limits<qint32>::min() ||
       target64 > std::numeric_limits<qint32>::max()) {
        setError(QStringLiteral("目标位置超出 32 位范围"));
        return false;
    }
    const qint32 target = qint32(target64);

    // 对应 Demo::execMoveZ：速度 -> 绝对目标位置。
    if(!writeMaxSpeedUm(axis, speed))
        return false;
    if(!writeTargetPositionUm(axis, target)) {
        // 响应丢失时设备可能已经开始运动，停止该轴避免失控。
        const QString targetError = m_lastError;
        systemCommand(quint16(CommandStopX + axis));
        m_lastError = targetError;
        return false;
    }
    return true;
}

bool ManipulatorHuayue::moveContinuousAxis(int axis, qint32 speed)
{
    // 厂家 Demo 中实机可用的“连续运动（坐标点列表）”并不是速度模式，
    // 而是位置模式下不断写绝对目标位置。主项目的连动只需一直运动到用户
    // 再次点击停止，因此沿用同一条已验证路径：设置正速度幅值，再写一个
    // 沿运动方向足够远的绝对目标位置。
    if(m_axisModes[axis] != 0) {
        if(!writeAxisMode(axis, 0)) {
            const QString modeError = QStringLiteral("切换位置模式失败：")
                    + m_lastError;
            systemCommand(quint16(CommandStopX + axis));
            m_lastError = modeError;
            return false;
        }
        m_axisModes[axis] = 0;
    }

    qint32 position[3] = {0, 0, 0};
    qint32 currentSpeed[3] = {0, 0, 0};
    if(!readPositionsAndSpeedsUm(position, currentSpeed)) {
        const QString positionError = QStringLiteral("读取连动起点失败：")
                + m_lastError;
        systemCommand(quint16(CommandStopX + axis));
        m_lastError = positionError;
        return false;
    }
    updatePosition(position);

    const qint64 speedMagnitude64 = qAbs(qint64(speed));
    if(speedMagnitude64 == 0 ||
       speedMagnitude64 > std::numeric_limits<qint32>::max()) {
        setError(QStringLiteral("连动速度无效"));
        return false;
    }
    const qint32 speedMagnitude = qint32(speedMagnitude64);

    const qint64 offset = speed > 0 ? ContinuousTargetOffsetUm
                                    : -qint64(ContinuousTargetOffsetUm);
    const qint64 unclampedTarget = qint64(position[axis]) + offset;
    const qint64 target64 = qMax(qint64(std::numeric_limits<qint32>::min()),
                                 qMin(qint64(std::numeric_limits<qint32>::max()),
                                      unclampedTarget));
    const qint32 target = qint32(target64);

    if(!writeMaxSpeedUm(axis, speedMagnitude)) {
        const QString speedError = QStringLiteral("写入连动速度失败：")
                + m_lastError;
        systemCommand(quint16(CommandStopX + axis));
        m_lastError = speedError;
        return false;
    }
    if(!writeTargetPositionUm(axis, target)) {
        const QString targetError = QStringLiteral("写入连动目标失败：")
                + m_lastError;
        systemCommand(quint16(CommandStopX + axis));
        m_lastError = targetError;
        return false;
    }

    qInfo() << DeviceName << "连动命令已发送：轴" << axis
            << "速度" << speedMagnitude << "um/s，目标" << target << "um";
    return true;
}

void ManipulatorHuayue::move(float x, float y, float z, float speed)
{
    if(!isConnected)
        return;

    bool xValid = false;
    bool yValid = false;
    bool zValid = false;
    bool speedValid = false;
    const qint32 dx = protocolValue(applyAxisDirection(x, AxisX), &xValid);
    const qint32 dy = protocolValue(applyAxisDirection(y, AxisY), &yValid);
    const qint32 dz = protocolValue(applyAxisDirection(z, AxisZ), &zValid);
    const float selectedSpeed = (!qFuzzyIsNull(speed) && qIsFinite(speed))
            ? qAbs(speed) : inchingSpeed;
    const qint32 deviceSpeed = protocolValue(selectedSpeed, &speedValid);
    if(!xValid || !yValid || !zValid || !speedValid || deviceSpeed <= 0)
        return;

    // 与 Demo 一致，只在缓存显示不是位置模式时发送模式切换。
    for(int axis = AxisX; axis <= AxisZ; ++axis) {
        if(m_axisModes[axis] != 0) {
            if(!writeAxisMode(axis, 0)) {
                const QString modeError = m_lastError;
                stop();
                m_lastError = modeError;
                qWarning() << DeviceName << "三轴位置模式切换失败：" << m_lastError;
                return;
            }
            m_axisModes[axis] = 0;
        }
    }

    qint32 position[3] = {0, 0, 0};
    qint32 currentSpeed[3] = {0, 0, 0};
    if(!readPositionsAndSpeedsUm(position, currentSpeed)) {
        qWarning() << DeviceName << "三轴运动前读取位置失败：" << m_lastError;
        return;
    }
    updatePosition(position);

    const qint64 target64[3] = {
        qint64(position[AxisX]) + qint64(dx),
        qint64(position[AxisY]) + qint64(dy),
        qint64(position[AxisZ]) + qint64(dz)
    };
    for(int axis = AxisX; axis <= AxisZ; ++axis) {
        if(target64[axis] < std::numeric_limits<qint32>::min() ||
           target64[axis] > std::numeric_limits<qint32>::max()) {
            qWarning() << DeviceName << "三轴目标位置超出 32 位范围";
            return;
        }
    }

    // 相对位移换算为绝对坐标后，采用 Demo 已验证可用的三轴绝对走位。
    for(int axis = AxisX; axis <= AxisZ; ++axis) {
        if(!writeMaxSpeedUm(axis, deviceSpeed)) {
            const QString speedError = m_lastError;
            stop();
            m_lastError = speedError;
            qWarning() << DeviceName << "三轴速度设置失败：" << m_lastError;
            return;
        }
    }
    if(!writeTargetPosition3Um(qint32(target64[AxisX]),
                               qint32(target64[AxisY]),
                               qint32(target64[AxisZ]))) {
        const QString targetError = m_lastError;
        stop();
        m_lastError = targetError;
        qWarning() << DeviceName << "三轴相对运动失败：" << m_lastError;
    }
}

void ManipulatorHuayue::setInchingSpeed(float speed)
{
    if(qIsFinite(speed) && speed > 0)
        inchingSpeed = speed;
}

void ManipulatorHuayue::stop()
{
    if(!isConnected)
        return;

    // 与 Demo::stopAllMotion 相同，依次发送三轴减速停止。
    bool ok = systemCommand(CommandStopX);
    ok = systemCommand(CommandStopY) && ok;
    ok = systemCommand(CommandStopZ) && ok;
    if(!ok)
        qWarning() << DeviceName << "停止命令发送失败：" << m_lastError;
}

void ManipulatorHuayue::pollDevice()
{
    if(!isConnected || !m_serial->isOpen())
        return;

    // 和 Demo 一致：20 ms 一帧，状态与位置交替读取，各 25 Hz。
    m_pollStatusNext = !m_pollStatusNext;
    bool ok = false;
    if(m_pollStatusNext) {
        quint16 status = 0;
        ok = readGlobalStatus(status);
    } else {
        qint32 position[3] = {0, 0, 0};
        qint32 speed[3] = {0, 0, 0};
        ok = readPositionsAndSpeedsUm(position, speed);
        if(ok)
            updatePosition(position);
    }

    // Demo 遇到单次轮询失败只记录错误，不主动关闭一个仍可通信的串口。
    if(!ok)
        qWarning() << DeviceName << "轮询失败：" << m_lastError;
}

void ManipulatorHuayue::handleSerialError(
        QSerialPort::SerialPortError error)
{
    if(error == QSerialPort::NoError ||
       error == QSerialPort::TimeoutError || m_closing) {
        return;
    }

    const bool fatalError = error == QSerialPort::ResourceError ||
                            error == QSerialPort::DeviceNotFoundError ||
                            error == QSerialPort::PermissionError ||
                            error == QSerialPort::ReadError ||
                            error == QSerialPort::WriteError;
    if(fatalError) {
        qWarning() << DeviceName << "串口断开："
                   << m_serial->errorString();
        disconnectDevice(true);
    }
}

void ManipulatorHuayue::disconnectDevice(bool notifyUi)
{
    const bool wasConnected = isConnected;
    m_pollTimer->stop();
    isConnected = false;
    m_axisModes[AxisX] = -1;
    m_axisModes[AxisY] = -1;
    m_axisModes[AxisZ] = -1;

    m_closing = true;
    if(m_serial->isOpen())
        m_serial->close();
    m_closing = false;

    if(notifyUi && wasConnected)
        emit noConnection();
}

// ---------------- 厂家 Demo 的 ModbusMaster 接口 ----------------

bool ManipulatorHuayue::enableModbus(bool enable)
{
    return write06(RegModbusEnable, enable ? 0xAAAA : 0x0000);
}

bool ManipulatorHuayue::systemCommand(quint16 command)
{
    return write06(RegSystemCommand, command);
}

bool ManipulatorHuayue::readGlobalStatus(quint16 &status)
{
    QVector<quint16> registers;
    if(!read03(RegSystemStatus, 1, registers))
        return false;
    status = registers[0];
    return true;
}

bool ManipulatorHuayue::writeAxisMode(int axis, quint16 mode)
{
    if(axis < AxisX || axis > AxisZ) {
        setError(QStringLiteral("轴号非法"));
        return false;
    }
    return write06(RegAxisMode + axis, mode);
}

bool ManipulatorHuayue::writeMaxSpeedUm(int axis, qint32 speed)
{
    if(axis < AxisX || axis > AxisZ) {
        setError(QStringLiteral("轴号非法"));
        return false;
    }

    quint16 lowWord = 0;
    quint16 highWord = 0;
    int32ToRegisters(speed, lowWord, highWord);
    QVector<quint16> values;
    values << lowWord << highWord;
    return write10(RegMaximumSpeedUm + axis * 2, values);
}

bool ManipulatorHuayue::writeTargetPositionUm(int axis, qint32 position)
{
    if(axis < AxisX || axis > AxisZ) {
        setError(QStringLiteral("轴号非法"));
        return false;
    }

    quint16 lowWord = 0;
    quint16 highWord = 0;
    int32ToRegisters(position, lowWord, highWord);
    QVector<quint16> values;
    values << lowWord << highWord;

    // 32 位目标位置必须在同一帧内完整写入，避免只更新低字或高字。
    return write10(RegTargetPositionUm + axis * 2, values);
}

bool ManipulatorHuayue::writeTargetPosition3Um(qint32 x, qint32 y,
                                               qint32 z)
{
    QVector<quint16> values;
    quint16 lowWord = 0;
    quint16 highWord = 0;
    int32ToRegisters(x, lowWord, highWord);
    values << lowWord << highWord;
    int32ToRegisters(y, lowWord, highWord);
    values << lowWord << highWord;
    int32ToRegisters(z, lowWord, highWord);
    values << lowWord << highWord;
    return write10(RegTargetPositionUm, values);
}

bool ManipulatorHuayue::readPositionsAndSpeedsUm(qint32 position[3],
                                                 qint32 speed[3])
{
    QVector<quint16> registers;
    if(!read03(RegRealtimeUm, 12, registers))
        return false;

    for(int axis = AxisX; axis <= AxisZ; ++axis){
        const quint16 positionLow = registers[axis * 2];
        const quint16 positionHigh = registers[axis * 2 + 1];
        // 部分控制器固件返回负位置时，只在低寄存器中保留 16 位补码，<br/>
        // 高寄存器仍为 0。这里仅兼容这种位置反馈，不影响速度和其他<br/>
        // 按协议返回的 32 位有符号数据。<br/>
        if(positionHigh == 0x0000 && (positionLow & 0x8000))
            position[axis] = qint32(qint16(positionLow));
        else
            position[axis] = registersToInt32(positionLow, positionHigh);
        speed[axis] = registersToInt32(registers[6 + axis * 2],
                                       registers[6 + axis * 2 + 1]);
    }
    return true;
}

// ---------------- 以下代码对应 Demo 的 read03/write06/write10 ----------------

bool ManipulatorHuayue::read03(quint16 startRegister, quint16 count,
                               QVector<quint16> &values)
{
    QByteArray request;
    request.append(char(m_slaveAddress));
    request.append(char(FunctionRead));
    request.append(char(startRegister >> 8));
    request.append(char(startRegister & 0xFF));
    request.append(char(count >> 8));
    request.append(char(count & 0xFF));
    appendCrc(request);

    QByteArray response;
    if(!transceive(request, response, 5 + 2 * count))
        return false;

    if(quint8(response[1]) != FunctionRead ||
       quint8(response[2]) != quint8(2 * count)) {
        setError(QStringLiteral("响应格式错误"));
        return false;
    }

    values.clear();
    for(int i = 0; i < count; ++i) {
        values.append(quint16((quint8(response[3 + i * 2]) << 8) |
                              quint8(response[4 + i * 2])));
    }
    return true;
}

bool ManipulatorHuayue::write06(quint16 reg, quint16 value)
{
    QByteArray request;
    request.append(char(m_slaveAddress));
    request.append(char(FunctionWriteOne));
    request.append(char(reg >> 8));
    request.append(char(reg & 0xFF));
    request.append(char(value >> 8));
    request.append(char(value & 0xFF));
    appendCrc(request);

    QByteArray response;
    if(!transceive(request, response, 8))
        return false;

    if(quint8(response[1]) != FunctionWriteOne) {
        setError(QStringLiteral("响应格式错误"));
        return false;
    }
    return true;
}

bool ManipulatorHuayue::write10(quint16 startRegister,
                                const QVector<quint16> &values)
{
    const quint16 count = quint16(values.size());
    QByteArray request;
    request.append(char(m_slaveAddress));
    request.append(char(FunctionWriteMultiple));
    request.append(char(startRegister >> 8));
    request.append(char(startRegister & 0xFF));
    request.append(char(count >> 8));
    request.append(char(count & 0xFF));
    request.append(char(count * 2));
    for(int i = 0; i < count; ++i) {
        request.append(char(values[i] >> 8));
        request.append(char(values[i] & 0xFF));
    }
    appendCrc(request);

    QByteArray response;
    if(!transceive(request, response, 8))
        return false;

    if(quint8(response[1]) != FunctionWriteMultiple) {
        setError(QStringLiteral("响应格式错误"));
        return false;
    }
    return true;
}

// 阻塞收发仍采用厂家 Demo 的请求格式。这里比 Demo 多做两件事：
// 1. 按协议限制相邻请求的间隔；2. 从串口字节流中寻找一条完整有效帧，
// 避免把迟到的上一帧和当前帧拼在一起做 CRC。
bool ManipulatorHuayue::transceive(const QByteArray &request,
                                   QByteArray &response,
                                   int expectedLength)
{
    response.clear();
    if(!m_serial->isOpen()) {
        setError(QStringLiteral("串口未打开"));
        return false;
    }

    const quint8 requestFunction = quint8(request[1]);
    bool crcFailure = false;

    // CRC 损坏时自动重发一次。当前驱动写入的模式、速度、绝对位置和
    // 停止命令都是可重复执行的，不会因为重发产生额外位移。
    for(int attempt = 0; attempt < 2; ++attempt) {
        if(m_requestTimer.isValid()) {
            const int waitMs = MinimumRequestIntervalMs
                    - int(m_requestTimer.elapsed());
            if(waitMs > 0)
                QThread::msleep(static_cast<unsigned long>(waitMs));
        }

        // 丢弃上一笔事务迟到的残留字节，再开始一笔完整请求。
        m_serial->clear(QSerialPort::Input);
        if(m_serial->write(request) != request.size()) {
            setError(QStringLiteral("串口写入失败"));
            return false;
        }
        m_serial->flush();
        m_requestTimer.restart();

        QByteArray received;
        bool thisAttemptCrcFailure = false;
        QElapsedTimer timeout;
        timeout.start();

        while(timeout.elapsed() < ModbusTimeoutMs) {
            if(m_serial->bytesAvailable() == 0)
                m_serial->waitForReadyRead(10);
            received += m_serial->readAll();

            // received 可能含有残留帧、当前帧，甚至两帧拼接；逐字节寻找
            // 地址和功能码匹配、长度正确且 CRC 正确的当前响应。
            for(int start = 0; start + 2 <= received.size(); ++start) {
                if(quint8(received[start]) != m_slaveAddress)
                    continue;

                const quint8 function = quint8(received[start + 1]);
                int frameLength = 0;
                if(function == quint8(requestFunction | 0x80)) {
                    frameLength = 5;
                } else if(function != requestFunction) {
                    continue;
                } else if(function == FunctionRead) {
                    if(start + 3 > received.size())
                        continue;
                    frameLength = 5 + quint8(received[start + 2]);
                } else {
                    frameLength = 8;
                }

                if(frameLength < 5 || start + frameLength > received.size())
                    continue;

                const QByteArray frame = received.mid(start, frameLength);
                const quint16 crc = crc16(frame.constData(), frameLength - 2);
                if(quint8(frame[frameLength - 2]) != quint8(crc & 0xFF) ||
                   quint8(frame[frameLength - 1]) != quint8(crc >> 8)) {
                    thisAttemptCrcFailure = true;
                    continue;
                }

                if(function & 0x80) {
                    setError(QStringLiteral("从机异常响应，异常码 0x%1")
                             .arg(quint8(frame[2]), 2, 16,
                                  QLatin1Char('0')).toUpper());
                    return false;
                }

                if(frameLength != expectedLength)
                    continue;

                // 0x06 和 0x10 的正常响应应回显本次请求的前 6 字节。
                // 这样不会把上一条写命令的迟到响应误认为本次成功。
                if(function != FunctionRead &&
                   frame.left(6) != request.left(6)) {
                    continue;
                }

                response = frame;
                m_lastError.clear();
                return true;
            }
        }

        crcFailure = crcFailure || thisAttemptCrcFailure;
        if(!thisAttemptCrcFailure)
            break;
    }

    setError(crcFailure ? QStringLiteral("CRC 校验失败")
                        : QStringLiteral("接收响应超时"));
    return false;
}

void ManipulatorHuayue::setError(const QString &message)
{
    m_lastError = message;
}

quint16 ManipulatorHuayue::crc16(const char *data, int length)
{
    quint16 crc = 0xFFFF;
    for(int i = 0; i < length; ++i) {
        crc ^= quint8(data[i]);
        for(int bit = 0; bit < 8; ++bit) {
            if(crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void ManipulatorHuayue::appendCrc(QByteArray &frame)
{
    const quint16 crc = crc16(frame.constData(), frame.size());
    frame.append(char(crc & 0xFF));
    frame.append(char(crc >> 8));
}

qint32 ManipulatorHuayue::registersToInt32(quint16 lowWord,
                                            quint16 highWord)
{
    return qint32((quint32(highWord) << 16) | quint32(lowWord));
}

void ManipulatorHuayue::int32ToRegisters(qint32 value,
                                         quint16 &lowWord,
                                         quint16 &highWord)
{
    lowWord = quint16(quint32(value) & 0xFFFF);
    highWord = quint16(quint32(value) >> 16);
}

qint32 ManipulatorHuayue::protocolValue(float value, bool *ok) const
{
    *ok = qIsFinite(value);
    if(!*ok)
        return 0;

    const qint64 rounded = qRound64(value);
    *ok = rounded >= std::numeric_limits<qint32>::min() &&
          rounded <= std::numeric_limits<qint32>::max();
    return *ok ? qint32(rounded) : 0;
}

float ManipulatorHuayue::applyAxisDirection(float value, int axis) const
{
    const bool sameDirection = axis == AxisX ? b_same_xdir :
                               axis == AxisY ? b_same_ydir : b_same_zdir;
    return sameDirection ? value : -value;
}

void ManipulatorHuayue::updatePosition(const qint32 position[3])
{
    raw_xpos = position[AxisX];
    raw_ypos = position[AxisY];
    raw_zpos = position[AxisZ];
    emit updatePos();
}
