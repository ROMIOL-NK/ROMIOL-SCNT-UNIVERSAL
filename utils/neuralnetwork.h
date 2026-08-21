#ifndef NEURALNETWORK_H
#define NEURALNETWORK_H
#include "public.h"

Q_DECLARE_METATYPE(cv::Mat);
class Yolo: public QObject
{
    Q_OBJECT
public:
    Yolo();
    Yolo(QString name, cv::Size s);
    ~Yolo();
    cv::Size modelShape;    //输入尺寸
private:
    cv::dnn::Net net;       //神经网络
    float modelConfidenceThreshold {0.25};
    float modelScoreThreshold      {0.45};
    float modelNMSThreshold        {0.50};
    std::vector<std::string> classes{"Injector", "Holder", "Oocyte"};
    // std::vector<std::string> classes{"Injector", "Holder", "Oocyte", "Polarbody", "Soma", "Cytoplasm", "Cytoplasm-Interface", "GL-Interface", "Polarbody-likely"};
    std::vector<std::string> classPolar{ "polar_body" };
public slots:
    bool load(const std::string &path, cv::Size shape = {WIDTH, HEIGHT});         //从路径文件中导入神经网络
    std::map<std::string, std::vector<cv::Rect>> getRects(cv::Mat input);    //运行神经网络
    std::map<std::string, std::vector<cv::Rect>> runPolar(cv::Mat input);
};

#endif // NEURALNETWORK_H
