#include "objectdetector.h"
using namespace std;
using namespace cv;
ObjectDetector::ObjectDetector()
{
    // 图像源文件夹
    img_folder = "F:/.romiol/";
    if(!QDir().exists(QString::fromStdString(img_folder)))
        QDir().mkpath(QString::fromStdString(img_folder));
    connect(this, SIGNAL(sgnVision()), vision, SLOT(run()));
    connect(vision, &Vision::resultReady, this, &ObjectDetector::handleResult);
    inUse = false;
}
ObjectDetector::~ObjectDetector()
{
}
map<string, vector<Rect>> ObjectDetector::getRects(Mat input)
{
    if(inUse)
        return map<string, vector<Rect>>();

    inUse = true;
    imwrite(img_folder+"img.jpg", input);
    QEventLoop loop;
    connect(this, &ObjectDetector::resultReceived, &loop, &QEventLoop::quit);

    emit sgnVision(); // 启动工作
    loop.exec(); // 等待结果
    map<string, vector<Rect>> _result;
    copy(this->result.begin(), this->result.end(), inserter(_result, _result.begin()));
    inUse = false;
    return _result;
}
void ObjectDetector::handleResult(map<string, vector<Rect>> _result) {
    this->result.clear();
    copy(_result.begin(), _result.end(), inserter(this->result, this->result.begin()));
    // this->result = _result;
    emit resultReceived(); // 通知事件循环结束
}
