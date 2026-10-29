// 在体细胞区域构建地图，提取所有卵细胞的位置信息，排序，为后续操作做准备
#ifndef STEPBUILDMAP_H
#define STEPBUILDMAP_H

#include "workflow.h"
#include "Objects/globalmap.h"

// 检测框结构体
struct DetectionBox
{
    cv::Rect box;           // 检测框
    cv::Point2f center;     // 中心点
    float confidence;       // 置信度
    bool isNearOverlap;     // 是否位于重叠带附近
    bool isReplaced;        // 是否被二次检测替换
};

class StepBuildMap: public WorkFlow
{
    Q_OBJECT

public:
    StepBuildMap();

    bool ON;
    cv::Mat mapImg;
    cv::Mat validMask;
    // 地图构建参数
    float overlapRatio;     // 图像重叠比例，默认 0.2

    // 动态计算得到的重叠像素数
    float overlapPixelsH;   // 水平方向重叠像素数
    float overlapPixelsV;   // 垂直方向重叠像素数

    // 目标网格，来自界面输入
    int targetCols;         // 用户期望的列数
    int targetRows;         // 用户期望的行数

    // 显示相关参数，保留给缩略图或可视化使用
    int displayWidth;
    int displayHeight;
    float tileScale;
    int displayTileW;
    int displayTileH;
    int displayOverlapW;
    int displayOverlapH;

    // 实际移动步长，单位为像素
    float stepDW;
    float stepDH;

    int minBoxSize;
    float iouThreshold;

    // 检测结果存储
    std::vector<DetectionBox> allDetections;
    std::vector<cv::Point2f> cellCenters;
signals:
    void mapImageUpdated(const cv::Mat& mapImage);
public slots:
    WORKSTEP run();
    void build();

    // 设置用户期望的网格，参数顺序为列数、行数
    void setGridSize(int cols, int rows);

    // 设置显示窗口尺寸
    void setDisplaySize(int w, int h);

    // 只用于显示缩放参数计算，不再用于建图移动步长
    void computeDynamicSteps();

private:
    cv::Mat captureImage();

    std::vector<DetectionBox> detectYOLO(cv::Mat img);
    std::vector<DetectionBox> detectOverlapArea(cv::Mat img, cv::Rect overlapRect);
    std::vector<DetectionBox> convertToGlobalCoords(std::vector<DetectionBox> localBoxes, cv::Point2f offset);

    void markNearOverlap(std::vector<DetectionBox>& boxes, cv::Rect overlapRect);
    void replaceWithRedetection(std::vector<DetectionBox>& original, std::vector<DetectionBox> redetected);

    std::vector<DetectionBox> nmsByIoU(std::vector<DetectionBox> boxes, float threshold);
    void removeSmallBoxes(std::vector<DetectionBox>& boxes, int minSize);
    void saveResults(
        const QString& rootPath,
        cv::Mat mapImg,
        std::vector<DetectionBox> boxes,
        const cv::Point2f& refCenterUm);
    void saveTileDebugResult(
        const QString& rootPath,
        int row,
        int col,
        const cv::Mat& img,
        const std::vector<DetectionBox>& localDetections,
        const std::vector<DetectionBox>& globalDetections,
        const cv::Point2f& offset);
    float calculateIoU(const cv::Rect& box1, const cv::Rect& box2);

    // 动态生成全局地图坐标系下的重叠区域
    cv::Rect getGlobalOverlapRect(
        const cv::Point2f& prevOffset,
        const cv::Point2f& currOffset,
        const cv::Size& imgSize);

    // 动态生成当前图像局部坐标系下的重叠区域
    cv::Rect getLocalOverlapRect(
        const cv::Point2f& prevOffset,
        const cv::Point2f& currOffset,
        const cv::Size& imgSize);

    std::vector<cv::Rect> getLocalCoveredOverlapRects(
        const cv::Point2f& currOffset,
        const cv::Size& imgSize);

    std::vector<cv::Rect> getGlobalCoveredOverlapRects(
        const cv::Point2f& currOffset,
        const cv::Size& imgSize);

    void markNearOverlapByRects(
        std::vector<DetectionBox>& boxes,
        const std::vector<cv::Rect>& overlapRects);
    bool isDuplicateOocyteBox(const DetectionBox& a, const DetectionBox& b);

    std::vector<DetectionBox> removeDuplicateOocytesByGeometry(
        std::vector<DetectionBox> boxes);

    // 拼接图像
    void extendMap(
        const cv::Point2f& prevOffset,
        const cv::Point2f& currOffset,
        const cv::Mat& img);
};

#endif // STEPBUILDMAP_H