#ifndef GLOBALMAP_H
#define GLOBALMAP_H
#include "public.h"
#include "Components/microscope.h"

class GlobalMap: public QObject
{
    Q_OBJECT
public:
    GlobalMap();

    cv::Mat mapImage;                       //图像
    float pixel2Um;                         //一个像素对应的实际长度(um)
    bool isValid;                           //地图有效的标志
};

#endif // GLOBALMAP_H
