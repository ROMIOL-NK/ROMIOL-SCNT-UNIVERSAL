#ifndef MANIPULATORHUAYUE_H
#define MANIPULATORHUAYUE_H

#include "Base/manipulator.h"

#include <QElapsedTimer>
#include <QSerialPort>
#include <QTimer>
#include <QVector>

// 华粤行三轴显微操作臂。
// 对外只保留项目既有的 Manipulator 接口；内部 Modbus 实现与厂家
// MMS_modbus_QTdemo 的 ModbusMaster 保持相同的组帧、字序和收发顺序。
class ManipulatorHuayue : public Manipulator
{
    Q_OBJECT

public:
    explicit ManipulatorHuayue(QString name = "Manipulator Huayue",
                               quint8 slaveAddress = 0x01,
                               int xDirection = 1,
                               int yDirection = 1,
                               int zDirection = 1);
    ~ManipulatorHuayue();

public slots:
    void connectDevice(QString portName) override;
    void modbus_enable(bool is_manual)   override;
    void moveX(float x, float vel = 0, bool CONTINUOUS = false) override;
    void moveY(float y, float vel = 0, bool CONTINUOUS = false) override;
    void moveZ(float z, float vel = 0, bool CONTINUOUS = false) override;
    void move(float x, float y, float z, float v = 0) override;
    void stop() override;
    void close() override;
    void setInchingSpeed(float speed) override;

private slots:
    void pollDevice();
    void handleSerialError(QSerialPort::SerialPortError error);

private:
    // Manipulator 高层接口到厂家 Demo 接口的薄适配层。
    void moveAxis(int axis, float distance, float speed, bool continuous);
    bool movePointAxis(int axis, qint32 distance, qint32 speed);
    bool moveContinuousAxis(int axis, qint32 speed);

    // 以下接口及实现逐项对应厂家 Demo 的 ModbusMaster。
    bool enableModbus(bool enable);
    bool systemCommand(quint16 command);
    bool readGlobalStatus(quint16 &status);
    bool writeAxisMode(int axis, quint16 mode);
    bool writeMaxSpeedUm(int axis, qint32 speed);
    bool writeTargetPositionUm(int axis, qint32 position);
    bool writeTargetPosition3Um(qint32 x, qint32 y, qint32 z);
    bool readPositionsAndSpeedsUm(qint32 position[3], qint32 speed[3]);

    bool read03(quint16 startRegister, quint16 count,
                QVector<quint16> &values);
    bool write06(quint16 reg, quint16 value);
    bool write10(quint16 startRegister, const QVector<quint16> &values);
    bool transceive(const QByteArray &request, QByteArray &response,
                    int expectedLength);
    void setError(const QString &message);

    static quint16 crc16(const char *data, int length);
    static void appendCrc(QByteArray &frame);
    static qint32 registersToInt32(quint16 lowWord, quint16 highWord);
    static void int32ToRegisters(qint32 value,
                                 quint16 &lowWord, quint16 &highWord);

    qint32 protocolValue(float value, bool *ok) const;
    float applyAxisDirection(float value, int axis) const;
    void updatePosition(const qint32 position[3]);
    void disconnectDevice(bool notifyUi);

    QSerialPort *m_serial;
    QTimer *m_pollTimer;
    quint8 m_slaveAddress;
    int m_axisModes[3];
    bool m_pollStatusNext;
    bool m_closing;
    QElapsedTimer m_requestTimer;
    QString m_lastError;
};

#endif // MANIPULATORHUAYUE_H
