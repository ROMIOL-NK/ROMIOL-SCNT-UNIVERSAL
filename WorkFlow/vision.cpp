#include "vision.h"
using namespace std;
using namespace cv;

Vision::Vision()
{
    socket = new QTcpSocket(this);
}
Vision::~Vision()
{
    delete socket;
}


void Vision::connectServer(QString strIP, int port)
{
    socket->connectToHost(strIP, port);
    if(socket->waitForConnected(3000))
        qDebug("Connect Server Success!");
    else
        qDebug()<<socket->errorString();
}
void Vision::send(QString msg)
{
    const char* cMsg = msg.toStdString().c_str();
    socket->write(cMsg, 7);
    socket->flush();
}
void Vision::close()
{
    socket->disconnectFromHost();
    socket->close();
}
void  Vision::run()
{
    map<string, vector<Rect>> results={};

    Mat showImg;
    try
    {
        showImg = imread("F:/.romiol/img.jpg");
    } catch (...) {
        return;
    }
    if(showImg.empty())
        return;
    send(" detect");
    float p;
    int x1,y1,x2,y2;
    QString qbuf;
    string name;
    Rect rect;
    QList<QString> item, items;

    if(socket->waitForReadyRead(1000))
    {
        buffer = socket->readAll();
        qbuf = QString(buffer);
        items = qbuf.split('\n');
        float max_pb_confidence = 0;
        for(int i=0; i<items.size(); ++i)
        {
            if(items[i].size()==0)
                continue;
            item = items[i].split(',');
            name = item.back().toStdString();
            if(results.count(name)<=0)
                results[name] = vector<Rect>();
            p = item.front().toFloat();
            x1 = item[1].toInt();
            y1 = item[2].toInt();
            x2 = item[3].toInt();
            y2 = item[4].toInt();
            rect = Rect(Point(x1,y1), Point(x2,y2));
            if(name!="Polarbody")
                results[name].push_back(rect);
            else
            {
                if(p>max_pb_confidence)
                {
                    if(results[name].empty())
                        results[name].push_back(rect);
                    else
                        results[name][0] = rect;
                    max_pb_confidence = p;
                }
            }

            //显示检测结果
            float confidence = p;
            Rect box = rect;
            Scalar color;
            std::string classString = name + ' ' + std::to_string(confidence).substr(0, 4);
            cv::Size textSize = cv::getTextSize(classString, cv::FONT_HERSHEY_DUPLEX, 1, 2, 0);
            cv::Rect textBox(box.x, box.y - 40, textSize.width + 10, textSize.height + 20);
            if(name=="Holder")
                color = Scalar(255, 0, 0);
            else if(name=="Oocyte")
                color = Scalar(0, 0, 255);
            else if(name=="Injector")
                color = Scalar(0, 255, 0);
            else
                color = Scalar(255,255,255);
            cv::rectangle(showImg, box, color, 2);
            // cv::rectangle(showImg, textBox, color, cv::FILLED);
            cv::putText(showImg, classString, cv::Point(box.x + 5, box.y - 10), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 0, 0), 1, 0);
        }


        imshow("socket", showImg);   waitKey(1);
    }
    emit resultReady(results);
}
