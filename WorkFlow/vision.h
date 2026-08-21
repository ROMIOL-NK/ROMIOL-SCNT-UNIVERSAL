#ifndef VISION_H
#define VISION_H

#include "public.h"
#include <QObject>
#include <QTcpSocket>

class Vision: public QObject
{
    Q_OBJECT
public:
    Vision();
    ~Vision();

    QTcpSocket* socket;
    QByteArray buffer;

public slots:
    void run();
    void connectServer(QString strIP="localhost", int port=9000);
    void send(QString);
    void close();

signals:
    void sgnRecv(QString);
    void resultReady(std::map<std::string, std::vector<cv::Rect>>);
};

#endif // VISION_H
