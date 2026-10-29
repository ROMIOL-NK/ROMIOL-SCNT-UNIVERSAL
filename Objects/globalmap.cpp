#include "globalmap.h"

using namespace cv;
using namespace std;

GlobalMap::GlobalMap()
{
    // 初始化必需字段
    mapImage = cv::Mat(Size(WIDTH, HEIGHT), CV_8UC1);  // 默认大小为当前视野大小
    pixel2Um = Pixel2Um;                               // 使用全局像素缩放因子
    isValid = false;                                   // 初始无效
    widthPx = mapImage.cols;
    heightPx = mapImage.rows;
    widthUm = widthPx * pixel2Um;
    heightUm = heightPx * pixel2Um;
    originUm = cv::Point2f(0.0f, 0.0f);                // 默认原点
    refCenterUm = cv::Point2f(0.0f, 0.0f);             // 默认参考点

    // 初始化可选字段
    filledMask = cv::Mat::zeros(Size(widthPx, heightPx), CV_8UC1);  // 初始全未填充
    thumbnail = cv::Mat();                                           // 空缩略图
    thumbWidth = GTHUMB_WIDTH;                                       // 使用全局缩略图宽度
    thumbHeight = GTHUMB_HEIGHT;                                     // 使用全局缩略图高度
    lastUpdated = QDateTime::currentDateTime();                      // 当前时间
    filePath = FILE_PATH + "/globalMap.png";                         // 默认路径
    metaOverlapRatio = 0.2f;                                         // 默认重叠比
}

// 将像素坐标转换为微米坐标
cv::Point2f GlobalMap::pixelToUm(const cv::Point2f& pixelPoint) const
{
    return cv::Point2f(
        originUm.x + pixelPoint.x * pixel2Um,
        originUm.y + pixelPoint.y * pixel2Um
        );
}

// 将微米坐标转换为像素坐标
cv::Point2f GlobalMap::umToPixel(const cv::Point2f& umPoint) const
{
    return cv::Point2f(
        (umPoint.x - originUm.x) / pixel2Um,
        (umPoint.y - originUm.y) / pixel2Um
        );
}

// 检查像素是否已被填充
bool GlobalMap::isPixelFilled(int x, int y) const
{
    if (x < 0 || x >= filledMask.cols || y < 0 || y >= filledMask.rows)
        return false;
    return filledMask.at<uchar>(y, x) > 0;
}

// 更新缩略图
void GlobalMap::updateThumbnail()
{
    QMutexLocker locker(&mutex);  // 线程安全
    if (mapImage.empty())
        return;

    cv::resize(mapImage, thumbnail, Size(thumbWidth, thumbHeight));
    lastUpdated = QDateTime::currentDateTime();
}

// 保存地图到文件
void GlobalMap::saveToFile(const QString& path)
{
    QString savePath = path.isEmpty() ? filePath : path;
    QMutexLocker locker(&mutex);  // 线程安全
    if (!mapImage.empty())
    {
        cv::imwrite(savePath.toStdString(), mapImage);
        lastUpdated = QDateTime::currentDateTime();
    }
}

// 从文件加载地图
bool GlobalMap::loadFromFile(const QString& path)
{
    QString loadPath = path.isEmpty() ? filePath : path;
    QMutexLocker locker(&mutex);  // 线程安全
    cv::Mat loaded = cv::imread(loadPath.toStdString(), cv::IMREAD_GRAYSCALE);
    if (loaded.empty())
        return false;

    mapImage = loaded;
    widthPx = mapImage.cols;
    heightPx = mapImage.rows;
    widthUm = widthPx * pixel2Um;
    heightUm = heightPx * pixel2Um;
    filledMask = cv::Mat::ones(Size(widthPx, heightPx), CV_8UC1) * 255;  // 假设加载的地图全填充
    if(!mapImage.empty())
        cv::resize(mapImage, thumbnail, Size(thumbWidth, thumbHeight));
    isValid = true;
    lastUpdated = QDateTime::currentDateTime();
    return true;
}
