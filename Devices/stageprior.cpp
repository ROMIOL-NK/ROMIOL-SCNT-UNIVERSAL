#include "stageprior.h"
using namespace std;
StagePrior::StagePrior()
{
    b_same_dirX = false;
    b_same_dirY = false;

    DeviceName = "Stage Device";
    endflag = "\r";
    queryStateInterval = 40;
    connectionInterval = 1000;

    m_port     = new QSerialPort(this);
    connectionTimer = new QTimer(this);
    queryStateTimer = new QTimer(this);

    //定时更新位置
    connect(queryStateTimer, &QTimer::timeout, [this](){queue("P\r");});
    connect(connectionTimer, &QTimer::timeout, [this](){emit noConnection();loseConnection=true;connectionTimer->stop();});
    connect(m_port,SIGNAL(readyRead()),this,SLOT(recv()),Qt::QueuedConnection);

}
StagePrior::~StagePrior()
{
    m_port->close();
    delete m_port;
    delete connectionTimer;
    delete queryStateTimer;
}

void StagePrior::connectDevice(QString portName)
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
        if(!m_port->isOpen())                                          //无法打开端口
            qWarning("%s %s is Occupied", qPrintable(DeviceName), qPrintable(portName));
        else                                                           //初始化设备参数
        {
            loseConnection = true;
            commandList.clear();
            responseTypes.clear();
            queue("P\r");
            // setInchingSpeed(100);
        }
    }
    if(!m_port->isOpen())
    {
        emit openPortFailure();
        qDebug() << "Can't connect " + DeviceName;
    }
}

void StagePrior::queue(QByteArray command, bool b_with_response)
{
    commandList.append(command);
    responseTypes.append(b_with_response);
    if(commandList.size()==1)   //如果只有新加入的指令，直接发送
        send();
}

void StagePrior::send()
{
    if(m_port->isOpen())
    {
        // 此时队列非空
        QByteArray cmd = commandList.first();
        if(responseTypes.first() && !loseConnection && !connectionTimer->isActive())
            connectionTimer->start(connectionInterval);
        m_port->write(cmd);
        if(!responseTypes.first())//如果该指令默认没有返回信号
        {
            commandList.pop_front();
            responseTypes.pop_front();
        }
    }
    else
        emit noConnection();
}

void StagePrior::recv()
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
    commandList.pop_front();
    responseTypes.pop_front();
    if(b)
    {
        RxBuffer = m_port->readAll();
        msg.append(RxBuffer);
        msg.replace("R\r", "");//ProScanscan II似乎会发送多余的R\r，删除这些无用字符串
        if(msg.size()>=18)
        {
            //针对不同指令，处理返回的数据
            char *message = msg.data();
            string msg_s;
            msg_s=message;
            int n=strlen(message);
            int split_0=-1, split_1=n-1, split_2=n-1, split_n=n-1;
            for(int i=n-1;i>=0;--i)
            {
                if(message[i]==',')
                {
                    if(split_2==n-1)
                    {
                        split_2=i;
                    }
                    else
                    {
                        split_1=i;
                        break;
                    }
                }
            }
            for(int i=n-1; i>=0; --i)
                if(message[i]==endflag[0])
                {
                    split_n = i;
                    break;
                }
            for(int i=split_n-1; i>=0; --i)
                if(message[i]==endflag[0])
                {
                    split_0 = i;
                    break;
                }
            if(split_0<split_1 && split_1<split_2 && split_2<split_n)
            {
                string x_s,y_s;
                x_s=msg_s.substr(split_0+1,split_1-split_0-1);
                y_s=msg_s.substr(split_1+1,split_2-split_1-1);
                int x_pos,y_pos;
                x_pos=atoi(x_s.c_str());
                y_pos=atoi(y_s.c_str());
                raw_xpos=((double)x_pos)/20.0;
                raw_ypos=((double)y_pos)/20.0;
                if(abs(raw_xpos)<100||abs(raw_ypos)<100)
                    qDebug()<<"Prior Stage "<<msg;
                emit updateXYPos();

                msg = msg.right(n-split_n-1);
            }
        }
        // emit noConnection();            //超过 [3] 秒没有收全响应，认为连接发生中断
    }
    if(!commandList.empty())
        send();
}


void StagePrior::setInchingSpeed(float vel)  //设置点动速度
{
    inchingSpeed = vel;
    std::string tmp1 = std::to_string(int(vel));
    QByteArray tmp2 = QByteArray::fromStdString(tmp1);

    // Report the current Stage (x, y) maximum speed setting m
    QByteArray setV_QByteArray="SMS, " + tmp2 + "\r";
    queue(setV_QByteArray);
}


void StagePrior::moveX(float x, bool CONTINUOUS, float vel)
{
    if(CONTINUOUS)
    {
        int vv=-vel; // VS指令中，Units are in microns/s
        std::string tmp1 = std::to_string(vv);
        QByteArray tmp2 = QByteArray::fromStdString(tmp1);
        QByteArray continuousmove_QByteArray="VS, "+tmp2+", 0"+"\r";
        queue(continuousmove_QByteArray);
    }
    else
    {
        int xx = x*20; // 1 um = 20 steps
        if(x>0)
        {
            std::string tmp1 = std::to_string(-xx);
            QByteArray tmp2 = QByteArray::fromStdString(tmp1);
            QByteArray moveup_QByteArray="R, " + tmp2 + "\r";
            queue(moveup_QByteArray);
        }
        else
        {
            std::string tmp1 = std::to_string(xx);
            QByteArray tmp2 = QByteArray::fromStdString(tmp1);
            QByteArray movedown_QByteArray="L, "+tmp2+"\r";
            queue(movedown_QByteArray);
        }
    }
}
void StagePrior::moveY(float y, bool CONTINUOUS, float vel)
{
    if(CONTINUOUS)
    {
        int vv=-vel; // VS指令中，Units are in microns/s
        std::string tmp1 = std::to_string(vv);
        QByteArray tmp2 = QByteArray::fromStdString(tmp1);
        QByteArray continuousmove_QByteArray="VS, 0, "+tmp2+"\r";
        queue(continuousmove_QByteArray);
    }
    else
    {
        int yy = y*20; // 1 um = 20 steps
        if(y>0)
        {
            std::string tmp1 = std::to_string(-yy);
            QByteArray tmp2 = QByteArray::fromStdString(tmp1);
            QByteArray moveup_QByteArray="F, " + tmp2 + "\r";
            queue(moveup_QByteArray);
        }
        else
        {
            std::string tmp1 = std::to_string(yy);
            QByteArray tmp2 = QByteArray::fromStdString(tmp1);
            QByteArray movedown_QByteArray="B, "+tmp2+"\r";
            queue(movedown_QByteArray);
        }
    }
}
void StagePrior::move(float x, float y, float vel)  // 点动，SUN：注意未测试！！！
{
    // if(vel!=0&&getInchingSpeed()!=abs(vel))
    //     setInchingSpeed(abs(vel));

    // SUN：暂时认定GR运动单位为微米
    int xx = x*20;  // 1 um = 20 steps
    int yy = y*20;  // 1 um = 20 steps

    std::string tmp1_x = std::to_string(-xx);
    std::string tmp1_y = std::to_string(-yy);

    QByteArray tmp2_x = QByteArray::fromStdString(tmp1_x);
    QByteArray tmp2_y = QByteArray::fromStdString(tmp1_y);

    QByteArray move_QByteArray="GR, " + tmp2_x +", "+tmp2_y+"\r";
    queue(move_QByteArray);
}

void StagePrior::stop()
{
    QByteArray stop_QByteArray="VS,0,0\r";
    commandList.clear();
    responseTypes.clear();
    queue(stop_QByteArray);
}

void StagePrior::close()
{
    queryStateTimer->stop();
    m_port->close();
    connectionTimer->stop();
    queryStateTimer->stop();
    isConnected = false;
}


