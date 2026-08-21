#include "neuralnetwork.h"
using namespace std;
using namespace cv;

Yolo::Yolo()
{
    QString modelName = "yolov8n-2023";
    QFile file(":/config/"+ modelName + ".onnx");
    file.copy("./"+ modelName + ".onnx");
    modelShape = Size(WIDTH, HEIGHT);
    net = cv::dnn::readNetFromONNX(modelName.toStdString() + ".onnx");
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
}
Yolo::Yolo(QString name, cv::Size s)
{
    qDebug() << "Cuda Device Count: " << cuda::getCudaEnabledDeviceCount();
    QFile file(":/config/" + name + ".onnx");
    file.copy("./" + name + ".onnx");
    modelShape = s;
    net = cv::dnn::readNetFromONNX(name.toStdString() + ".onnx");
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
}
Yolo::~Yolo()
{

}
bool Yolo::load(const std::string &path, Size shape)
{
    modelShape = shape;
    net = cv::dnn::readNetFromONNX(path);
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
    //    net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    //    net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    return true;
}
map<string, vector<Rect>> Yolo::getRects(Mat input)
{
    map<string, vector<Rect>> results;
    if(input.empty())
        return results;
    if(input.channels()==1)
        cvtColor(input, input, COLOR_GRAY2BGR);
    Mat blobInput;
    vector<Mat> outputs;
    Mat modelInput = input.clone();
    resize(modelInput, modelInput, modelShape);
    Mat showImg = input.clone();
    dnn::blobFromImage(modelInput, blobInput, 1.0/255.0, modelShape, cv::Scalar(), true, false);
    net.setInput(blobInput);

    results["Injector"]=vector<Rect>();
    results["Holder"]=vector<Rect>();
    results["Oocyte"]=vector<Rect>();
    results["Polarbody"]=vector<Rect>();
    results["Soma"]=vector<Rect>();
    results["Cytoplasm"]=vector<Rect>();
    results["Cytoplasm-Interface"]=vector<Rect>();
    results["GL-Interface"]=vector<Rect>();
    results["Polarbody-likely"]=vector<Rect>();

    QDateTime startTime=  QDateTime::currentDateTime();
    net.forward(outputs, net.getUnconnectedOutLayersNames());
    QDateTime endTime=  QDateTime::currentDateTime();
    qint64 intervalTimeMS = startTime.msecsTo(endTime);
    // qDebug() << "Neural Network runtime: " << intervalTimeMS << "ms";
    int rows = outputs[0].size[2];
    int dimensions = outputs[0].size[1];

    outputs[0] = outputs[0].reshape(1, dimensions);
    transpose(outputs[0], outputs[0]);

    float *data = (float *)outputs[0].data;
    float x_factor = 1.0*modelInput.cols / modelShape.width;
    float y_factor = 1.0*modelInput.rows / modelShape.height;

    vector<int> class_ids;
    vector<float> confidences;
    vector<Rect> boxes;
    for (int i = 0; i < rows; ++i)
    {
        float *classes_scores = data+4;

        Mat scores(1, classes.size(), CV_32FC1, classes_scores);
        Point class_id;
        double maxClassScore;

        minMaxLoc(scores, 0, &maxClassScore, 0, &class_id);

        if (maxClassScore > modelScoreThreshold)
        {
            confidences.push_back(maxClassScore);
            class_ids.push_back(class_id.x);

            float x = data[0];
            float y = data[1];
            float w = data[2];
            float h = data[3];

            int left = int((x - 0.5 * w) * x_factor);
            int top = int((y - 0.5 * h) * y_factor);

            int width = int(w * x_factor);
            int height = int(h * y_factor);

            boxes.push_back(cv::Rect(left, top, width, height));
        }
        data += dimensions;
    }
    vector<int> nms_result;
    dnn::NMSBoxes(boxes, confidences, modelScoreThreshold, modelNMSThreshold, nms_result);

    for (unsigned long i = 0; i < nms_result.size(); ++i)
    {
        int idx = nms_result[i];
        int class_id = class_ids[idx];
        string className = classes[class_id];
        results[className].push_back(boxes[idx]);

        //显示检测结果
        float confidence = confidences[idx];
        Rect box = boxes[idx];
        Scalar color;
        std::string classString = className + ' ' + std::to_string(confidence).substr(0, 4);
        cv::Size textSize = cv::getTextSize(classString, cv::FONT_HERSHEY_DUPLEX, 1, 2, 0);
        cv::Rect textBox(box.x, box.y - 40, textSize.width + 10, textSize.height + 20);
        if(className=="Holder"||className=="holder_current")
            color = Scalar(255, 0, 0);
        else if(className=="Oocyte"||className=="oocyte_current")
            color = Scalar(0, 0, 255);
        else if(className=="Injector"||className=="injector_current")
            color = Scalar(0, 255, 0);
        else
            color = Scalar(255,255,255);
        cv::rectangle(showImg, box, color, 2);
        cv::rectangle(showImg, textBox, color, cv::FILLED);
        cv::putText(showImg, classString, cv::Point(box.x + 5, box.y - 10), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 0, 0), 2, 0);
    }
    imshow("detection", showImg);   waitKey(1);
    return results;
}
map<string, vector<Rect>> Yolo::runPolar(cv::Mat input)
{
    map<string, vector<Rect>> results;
    if (input.empty())
        return results;
    if (input.channels() == 1)
        cvtColor(input, input, COLOR_GRAY2BGR);
    Mat blobInput;
    vector<Mat> outputs;
    Mat modelInput = input.clone();
    Mat showImg = modelInput.clone();
    dnn::blobFromImage(modelInput, blobInput, 1.0 / 255.0, modelShape, cv::Scalar(), true, false);
    net.setInput(blobInput);

    results["polar_body"] = vector<Rect>();

    QDateTime startTime = QDateTime::currentDateTime();
    net.forward(outputs, net.getUnconnectedOutLayersNames());
    QDateTime endTime = QDateTime::currentDateTime();
    qint64 intervalTimeMS = startTime.msecsTo(endTime);
    // qDebug() << "Neural Network runtime: " << intervalTimeMS << "ms";
    int rows = outputs[0].size[2];
    int dimensions = outputs[0].size[1];

    outputs[0] = outputs[0].reshape(1, dimensions);
    transpose(outputs[0], outputs[0]);

    float* data = (float*)outputs[0].data;
    float x_factor = 1.0 * modelInput.cols / modelShape.width;
    float y_factor = 1.0 * modelInput.rows / modelShape.height;

    vector<int> class_ids;
    vector<float> confidences;
    vector<Rect> boxes;
    for (int i = 0; i < rows; ++i)
    {
        float* classes_scores = data + 4;

        Mat scores(1, classPolar.size(), CV_32FC1, classes_scores);
        Point class_id;
        double maxClassScore;

        minMaxLoc(scores, 0, &maxClassScore, 0, &class_id);

        if (maxClassScore > modelScoreThreshold)
        {
            confidences.push_back(maxClassScore);
            class_ids.push_back(class_id.x);

            float x = data[0];
            float y = data[1];
            float w = data[2];
            float h = data[3];

            int left = int((x - 0.5 * w) * x_factor);
            int top = int((y - 0.5 * h) * y_factor);

            int width = int(w * x_factor);
            int height = int(h * y_factor);

            boxes.push_back(cv::Rect(left, top, width, height));
        }
        data += dimensions;
    }
    vector<int> nms_result;
    dnn::NMSBoxes(boxes, confidences, modelScoreThreshold, modelNMSThreshold, nms_result);

    for (unsigned long i = 0; i < nms_result.size(); ++i)
    {
        int idx = nms_result[i];
        int class_id = class_ids[idx];
        string className = classPolar[class_id];
        results[className].push_back(boxes[idx]);

        //显示检测结果
        // float confidence = confidences[idx];
        // Rect box = boxes[idx];
        // Scalar color;
        // std::string classString = className + ' ' + std::to_string(confidence).substr(0, 4);
        // cv::Size textSize = cv::getTextSize(classString, cv::FONT_HERSHEY_DUPLEX, 1, 2, 0);
        // cv::Rect textBox(box.x, box.y - 40, textSize.width + 10, textSize.height + 20);
        // color = Scalar(255, 0, 0);
        // cv::rectangle(showImg, box, color, 2);
        // cv::rectangle(showImg, textBox, color, cv::FILLED);
        // cv::putText(showImg, classString, cv::Point(box.x + 5, box.y - 10), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 0, 0), 2, 0);
        // imshow("PB detection", showImg);
    }
    waitKey(1);
    return results;
}
