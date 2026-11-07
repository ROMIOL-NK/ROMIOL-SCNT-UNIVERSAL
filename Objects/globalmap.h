#ifndef GLOBALMAP_H
#define GLOBALMAP_H
#include "public.h"
#include "Components/microscope.h"

class GlobalMap: public QObject
{
    Q_OBJECT
public:
    GlobalMap();

    // 必需字段
    cv::Mat mapImage;                       // 全局拼接图像（灰度或彩色），主数据载体
    float pixel2Um;                         // 像素到微米的缩放因子，用于像素↔世界坐标转换
    bool isValid;                           // 地图是否有效/已构建完成标志
    int widthPx;                            // 地图宽度（像素），从 mapImage.cols 派生
    int heightPx;                           // 地图高度（像素），从 mapImage.rows 派生
    float widthUm;                          // 地图宽度（微米），与 pixel2Um 对应
    float heightUm;                         // 地图高度（微米），与 pixel2Um 对应
    cv::Point2f originUm;                   // 地图像素 (0,0) 对应的物理世界坐标（um），用于像素→世界坐标精确映射
    cv::Point2f refCenterUm;                // 参考点（例如第1张视野中心 / oocyteArea）的世界坐标，便于计算细胞 centerUm

    // 可选/建议字段
    cv::Mat filledMask;                     // 标记哪些像素已被填充的掩码，用于拼接融合与缩略图暗化处理
    cv::Mat thumbnail;                      // 预生成的缩略图（thumbWidth × thumbHeight），用于 UI 加速与鼠标映射
    int thumbWidth;                         // 缩略图宽度（像素）
    int thumbHeight;                        // 缩略图高度（像素）
    QDateTime lastUpdated;                  // 地图最后更新时间戳，用于缓存/重建判断
    QString filePath;                       // 默认保存/加载路径
    QMutex mutex;                           // 保护多线程读写 mapImage
    float metaOverlapRatio;                 // 拼接策略中的重叠比参数

    // 简便方法
    cv::Point2f pixelToUm(const cv::Point2f& pixelPoint) const;  // 将像素坐标转换为微米坐标
    cv::Point2f umToPixel(const cv::Point2f& umPoint) const;     // 将微米坐标转换为像素坐标
    bool isPixelFilled(int x, int y) const;                     // 检查像素是否已被填充
    void updateThumbnail();                                      // 更新缩略图
    void saveToFile(const QString& path = QString());            // 保存地图到文件
    bool loadFromFile(const QString& path = QString());          // 从文件加载地图
};

#endif // GLOBALMAP_H
