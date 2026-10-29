#include "stepbuildmap.h"
#include <QElapsedTimer>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <string>
#include <vector>
#include "utils/objectdetector.h"
using namespace std;
using namespace cv;


namespace StepBuildMapInternal
{
cv::Mat gWeightMap;

static inline float limitFloatValue(float value, float lower, float upper)
{
    if(value < lower)
        return lower;

    if(value > upper)
        return upper;

    return value;
}

static inline float smoothStep01Value(float t)
{
    t = limitFloatValue(t, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

static float tileReliabilityWeight(
    int x,
    int y,
    int width,
    int height,
    int edgeMargin)
{
    if(width <= 0 || height <= 0)
        return 0.0f;

    int leftDist = x;
    int rightDist = width - 1 - x;
    int topDist = y;
    int bottomDist = height - 1 - y;

    int edgeDist = std::min(
        std::min(leftDist, rightDist),
        std::min(topDist, bottomDist)
        );

    if(edgeMargin <= 1)
        return 1.0f;

    float t = static_cast<float>(edgeDist) /
              static_cast<float>(edgeMargin);
    t = smoothStep01Value(t);

    const float minWeight = 0.01f;
    return minWeight + (1.0f - minWeight) * t;
}

static cv::Mat buildTileWeightMap(const cv::Size& size, int edgeMargin)
{
    cv::Mat weightMap(size, CV_32FC1, cv::Scalar(0.0f));

    for(int y = 0; y < size.height; ++y)
    {
        for(int x = 0; x < size.width; ++x)
        {
            weightMap.at<float>(y, x) = tileReliabilityWeight(
                x,
                y,
                size.width,
                size.height,
                edgeMargin
                );
        }
    }

    return weightMap;
}

static float computeDetectionQuality(
    const cv::Rect& rect,
    const cv::Size& imgSize)
{
    if(rect.area() <= 0 || imgSize.width <= 0 || imgSize.height <= 0)
        return 0.0f;

    float w = static_cast<float>(rect.width);
    float h = static_cast<float>(rect.height);

    if(w <= 1.0f || h <= 1.0f)
        return 0.0f;

    float leftDist = static_cast<float>(rect.x);
    float topDist = static_cast<float>(rect.y);
    float rightDist = static_cast<float>(imgSize.width - rect.x - rect.width);
    float bottomDist = static_cast<float>(imgSize.height - rect.y - rect.height);

    float minBorderDist = std::min(
        std::min(leftDist, rightDist),
        std::min(topDist, bottomDist)
        );

    float borderMargin = std::max(
        24.0f,
        0.06f * static_cast<float>(std::min(imgSize.width, imgSize.height))
        );

    float borderQuality = limitFloatValue(
        minBorderDist / borderMargin,
        0.0f,
        1.0f
        );
    borderQuality = 0.12f + 0.88f * borderQuality;

    float aspectQuality = std::min(w / h, h / w);
    aspectQuality = limitFloatValue(aspectQuality, 0.0f, 1.0f);

    float sideQuality =
        std::min(w, h) /
        std::max(
            40.0f,
            0.08f * static_cast<float>(
                std::min(imgSize.width, imgSize.height)
                )
            );
    sideQuality = limitFloatValue(sideQuality, 0.0f, 1.0f);

    float quality = borderQuality *
                    (0.60f + 0.40f * aspectQuality) *
                    (0.70f + 0.30f * sideQuality);

    return limitFloatValue(quality, 0.01f, 1.0f);
}
} // namespace StepBuildMapInternal


StepBuildMap::StepBuildMap()
{
    overlapRatio = 0.2f;

    overlapPixelsH = WIDTH * overlapRatio;
    overlapPixelsV = HEIGHT * overlapRatio;

    minBoxSize = 30;
    iouThreshold = 0.5f;
    ON = false;

    targetCols = 0;
    targetRows = 0;

    displayWidth = GTHUMB_WIDTH;
    displayHeight = GTHUMB_HEIGHT;
    tileScale = 1.0f;
    displayTileW = WIDTH;
    displayTileH = HEIGHT;
    displayOverlapW = static_cast<int>(overlapPixelsH * tileScale);
    displayOverlapH = static_cast<int>(overlapPixelsV * tileScale);

    stepDW = WIDTH - overlapPixelsH;
    stepDH = HEIGHT - overlapPixelsV;
}

// cv::Mat StepBuildMap::captureImage()
// {
//     /*
//      * 移动完成后相机缓冲区中可能仍然保留运动过程中的旧帧。
//      * 这里连续丢弃几帧，再返回最后一帧，避免刚移动完立即取图。
//      */
//     cv::Mat img;
//     const int discardFrameCount = 3;
//     const int frameIntervalMs = 80;

//     for(int i = 0; i < discardFrameCount; ++i)
//     {
//         QThread::msleep(frameIntervalMs);
//         img = CurrentGray.clone();
//     }

//     return img;
// }




cv::Mat StepBuildMap::captureImage()
{
    /*
     * 首次打开程序时，相机线程可能尚未写入有效 CurrentGray。
     * 因此不能只固定丢弃三帧，而应等待到图像非空。
     */
    cv::Mat img;

    const int timeoutMs = 5000;
    const int sampleIntervalMs = 80;
    const int discardFrameCountAfterReady = 3;

    QElapsedTimer timer;
    timer.start();

    while(timer.elapsed() < timeoutMs)
    {
        QThread::msleep(sampleIntervalMs);

        if(!CurrentGray.empty())
        {
            img = CurrentGray.clone();
            break;
        }
    }

    if(img.empty())
    {
        qDebug() << "captureImage failed: CurrentGray is empty after timeout.";
        return img;
    }

    for(int i = 0; i < discardFrameCountAfterReady; ++i)
    {
        QThread::msleep(sampleIntervalMs);

        if(!CurrentGray.empty())
            img = CurrentGray.clone();
    }

    return img;
}

std::vector<DetectionBox> StepBuildMap::detectYOLO(cv::Mat img)
{
    std::vector<DetectionBox> detections;

    if(img.empty())
        return detections;

    map<string, vector<Rect>> results;
    if(isUsingSocket)
        results = objectDetector->getRects(img);
    else
        results = yolo->getRects(img);

    for(const auto& rect : results["Oocyte"])
    {
        DetectionBox box;
        box.box = rect;
        box.center = Point2f(rect.x + rect.width / 2.0f,
                             rect.y + rect.height / 2.0f);
        box.confidence = StepBuildMapInternal::computeDetectionQuality(box.box, img.size());
        box.isNearOverlap = false;
        box.isReplaced = false;
        detections.push_back(box);
    }

    return detections;
}

std::vector<DetectionBox> StepBuildMap::detectOverlapArea(cv::Mat img, cv::Rect overlapRect)
{
    std::vector<DetectionBox> detections;

    if(img.empty() || overlapRect.area() == 0)
        return detections;

    overlapRect = overlapRect & cv::Rect(0, 0, img.cols, img.rows);
    if(overlapRect.area() == 0)
        return detections;

    cv::Mat overlapImg = img(overlapRect).clone();

    map<string, vector<Rect>> results;
    if(isUsingSocket)
        results = objectDetector->getRects(overlapImg);
    else
        results = yolo->getRects(overlapImg);

    for(const auto& rect : results["Oocyte"])
    {
        DetectionBox box;
        box.box = cv::Rect(
            rect.x + overlapRect.x,
            rect.y + overlapRect.y,
            rect.width,
            rect.height
            );
        box.center = Point2f(box.box.x + box.box.width / 2.0f,
                             box.box.y + box.box.height / 2.0f);
        box.confidence = StepBuildMapInternal::computeDetectionQuality(box.box, img.size());
        box.isNearOverlap = true;
        box.isReplaced = false;
        detections.push_back(box);
    }

    return detections;
}

std::vector<DetectionBox> StepBuildMap::convertToGlobalCoords(
    std::vector<DetectionBox> localBoxes,
    cv::Point2f offset)
{
    int ox = static_cast<int>(std::round(offset.x));
    int oy = static_cast<int>(std::round(offset.y));

    for(auto& box : localBoxes)
    {
        box.box.x += ox;
        box.box.y += oy;
        box.center.x += offset.x;
        box.center.y += offset.y;
    }

    return localBoxes;
}

void StepBuildMap::markNearOverlap(std::vector<DetectionBox>& boxes, cv::Rect overlapRect)
{
    for(auto& box : boxes)
    {
        cv::Rect intersection = box.box & overlapRect;
        float overlapArea = static_cast<float>(intersection.area());
        float boxArea = static_cast<float>(box.box.area());

        if(boxArea > 0.0f && overlapArea / boxArea > 0.3f)
        {
            box.isNearOverlap = true;
        }
    }
}

void StepBuildMap::replaceWithRedetection(
    std::vector<DetectionBox>& original,
    std::vector<DetectionBox> redetected)
{
    for(auto& origBox : original)
    {
        if(!origBox.isNearOverlap)
            continue;

        float maxIoU = 0.0f;
        int bestMatch = -1;

        for(size_t i = 0; i < redetected.size(); i++)
        {
            float iou = calculateIoU(origBox.box, redetected[i].box);
            if(iou > maxIoU)
            {
                maxIoU = iou;
                bestMatch = static_cast<int>(i);
            }
        }

        if(maxIoU > 0.3f && bestMatch >= 0)
        {
            origBox.box = redetected[bestMatch].box;
            origBox.center = redetected[bestMatch].center;
            origBox.confidence = redetected[bestMatch].confidence;
            origBox.isReplaced = true;
        }
    }
}

float StepBuildMap::calculateIoU(const cv::Rect& box1, const cv::Rect& box2)
{
    cv::Rect intersection = box1 & box2;
    float intersectionArea = static_cast<float>(intersection.area());
    float unionArea = static_cast<float>(box1.area() + box2.area() - intersection.area());

    return unionArea > 0.0f ? intersectionArea / unionArea : 0.0f;
}

void StepBuildMap::setGridSize(int cols, int rows)
{
    targetCols = cols;
    targetRows = rows;

    if(targetCols <= 0)
        targetCols = 1;

    if(targetRows <= 0)
        targetRows = 1;
}

void StepBuildMap::setDisplaySize(int w, int h)
{
    displayWidth = w;
    displayHeight = h;
}

void StepBuildMap::computeDynamicSteps()
{
    int mapW = GMAP_WIDTH;
    int mapH = GMAP_HEIGHT;

    int dispW = displayWidth > 0 ? displayWidth : GTHUMB_WIDTH;
    int dispH = displayHeight > 0 ? displayHeight : GTHUMB_HEIGHT;

    if(mapW > 0 && mapH > 0)
    {
        tileScale = std::min(static_cast<float>(dispW) / static_cast<float>(mapW),
                             static_cast<float>(dispH) / static_cast<float>(mapH));
    }
    else
    {
        tileScale = 1.0f;
    }

    displayTileW = static_cast<int>(std::round(WIDTH * tileScale));
    displayTileH = static_cast<int>(std::round(HEIGHT * tileScale));
    displayOverlapW = static_cast<int>(std::round(overlapPixelsH * tileScale));
    displayOverlapH = static_cast<int>(std::round(overlapPixelsV * tileScale));
}

cv::Rect StepBuildMap::getGlobalOverlapRect(
    const cv::Point2f& prevOffset,
    const cv::Point2f& currOffset,
    const cv::Size& imgSize)
{
    int px = static_cast<int>(std::round(prevOffset.x));
    int py = static_cast<int>(std::round(prevOffset.y));

    int cx = static_cast<int>(std::round(currOffset.x));
    int cy = static_cast<int>(std::round(currOffset.y));

    cv::Rect prevRect(px, py, imgSize.width, imgSize.height);
    cv::Rect currRect(cx, cy, imgSize.width, imgSize.height);

    return prevRect & currRect;
}

cv::Rect StepBuildMap::getLocalOverlapRect(
    const cv::Point2f& prevOffset,
    const cv::Point2f& currOffset,
    const cv::Size& imgSize)
{
    int cx = static_cast<int>(std::round(currOffset.x));
    int cy = static_cast<int>(std::round(currOffset.y));

    cv::Rect globalOverlap = getGlobalOverlapRect(prevOffset, currOffset, imgSize);

    if(globalOverlap.area() <= 0)
        return cv::Rect();

    cv::Rect localOverlap(
        globalOverlap.x - cx,
        globalOverlap.y - cy,
        globalOverlap.width,
        globalOverlap.height
        );

    localOverlap = localOverlap & cv::Rect(0, 0, imgSize.width, imgSize.height);

    return localOverlap;
}

std::vector<cv::Rect> StepBuildMap::getLocalCoveredOverlapRects(
    const cv::Point2f& currOffset,
    const cv::Size& imgSize)
{
    std::vector<cv::Rect> localRects;

    if(validMask.empty())
        return localRects;

    int ox = static_cast<int>(std::round(currOffset.x));
    int oy = static_cast<int>(std::round(currOffset.y));

    cv::Rect currGlobalRect(ox, oy, imgSize.width, imgSize.height);
    cv::Rect mapRect(0, 0, validMask.cols, validMask.rows);

    cv::Rect clippedGlobalRect = currGlobalRect & mapRect;
    if(clippedGlobalRect.area() <= 0)
        return localRects;

    cv::Mat maskPatch = validMask(clippedGlobalRect).clone();

    cv::Mat binaryPatch;
    cv::threshold(maskPatch, binaryPatch, 0, 255, cv::THRESH_BINARY);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binaryPatch, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for(const auto& contour : contours)
    {
        cv::Rect r = cv::boundingRect(contour);

        if(r.area() < 50)
            continue;

        if(r.width < 5 || r.height < 5)
            continue;

        r.x += clippedGlobalRect.x - ox;
        r.y += clippedGlobalRect.y - oy;

        r = r & cv::Rect(0, 0, imgSize.width, imgSize.height);

        if(r.area() > 0)
            localRects.push_back(r);
    }

    return localRects;
}


std::vector<cv::Rect> StepBuildMap::getGlobalCoveredOverlapRects(
    const cv::Point2f& currOffset,
    const cv::Size& imgSize)
{
    std::vector<cv::Rect> globalRects;

    std::vector<cv::Rect> localRects =
        getLocalCoveredOverlapRects(currOffset, imgSize);

    int ox = static_cast<int>(std::round(currOffset.x));
    int oy = static_cast<int>(std::round(currOffset.y));

    for(auto r : localRects)
    {
        r.x += ox;
        r.y += oy;
        globalRects.push_back(r);
    }

    return globalRects;
}

void StepBuildMap::markNearOverlapByRects(
    std::vector<DetectionBox>& boxes,
    const std::vector<cv::Rect>& overlapRects)
{
    for(auto& box : boxes)
    {
        for(const auto& overlapRect : overlapRects)
        {
            cv::Rect intersection = box.box & overlapRect;

            float overlapArea = static_cast<float>(intersection.area());
            float boxArea = static_cast<float>(box.box.area());

            if(boxArea > 0.0f && overlapArea / boxArea > 0.3f)
            {
                box.isNearOverlap = true;
                break;
            }
        }
    }
}

std::vector<DetectionBox> StepBuildMap::nmsByIoU(
    std::vector<DetectionBox> boxes,
    float threshold)
{
    if(boxes.empty())
        return boxes;

    sort(boxes.begin(), boxes.end(),
         [](const DetectionBox& a, const DetectionBox& b)
         {
             if(std::abs(a.confidence - b.confidence) > 1e-4f)
                 return a.confidence > b.confidence;

             float areaA = static_cast<float>(a.box.area());
             float areaB = static_cast<float>(b.box.area());

             if(std::abs(areaA - areaB) > 1e-3f)
                 return areaA > areaB;

             if(a.isNearOverlap != b.isNearOverlap)
                 return !a.isNearOverlap;

             return a.isReplaced && !b.isReplaced;
         });

    std::vector<DetectionBox> result;
    std::vector<bool> suppressed(boxes.size(), false);

    for(size_t i = 0; i < boxes.size(); i++)
    {
        if(suppressed[i])
            continue;

        result.push_back(boxes[i]);

        for(size_t j = i + 1; j < boxes.size(); j++)
        {
            if(suppressed[j])
                continue;

            float iou = calculateIoU(boxes[i].box, boxes[j].box);
            if(iou > threshold)
            {
                suppressed[j] = true;
            }
        }
    }

    return result;
}


bool StepBuildMap::isDuplicateOocyteBox(
    const DetectionBox& a,
    const DetectionBox& b)
{
    float areaA = static_cast<float>(a.box.area());
    float areaB = static_cast<float>(b.box.area());

    if(areaA <= 0.0f || areaB <= 0.0f)
        return false;

    float minArea = std::min(areaA, areaB);
    float maxArea = std::max(areaA, areaB);
    float areaRatio = minArea / maxArea;

    float dx = a.center.x - b.center.x;
    float dy = a.center.y - b.center.y;
    float centerDist = std::sqrt(dx * dx + dy * dy);

    float diameterA = static_cast<float>(std::max(a.box.width, a.box.height));
    float diameterB = static_cast<float>(std::max(b.box.width, b.box.height));
    float refDiameter = std::max(diameterA, diameterB);
    refDiameter = std::max(refDiameter, 60.0f);

    cv::Rect inter = a.box & b.box;
    float interArea = static_cast<float>(inter.area());

    if(interArea > 0.0f)
    {
        float interOverMin = interArea / minArea;
        float iou = interArea / (areaA + areaB - interArea);

        if(interOverMin >= 0.45f)
            return true;

        if(iou >= 0.25f)
            return true;
    }

    cv::Point centerA(
        static_cast<int>(std::round(a.center.x)),
        static_cast<int>(std::round(a.center.y))
        );

    cv::Point centerB(
        static_cast<int>(std::round(b.center.x)),
        static_cast<int>(std::round(b.center.y))
        );

    bool centerInside = a.box.contains(centerB) || b.box.contains(centerA);
    bool overlapCandidate = a.isNearOverlap || b.isNearOverlap || a.isReplaced || b.isReplaced;

    /*
     * 原来的函数在 interArea <= 0 时直接返回 false。
     * 这会漏掉边缘半框和完整框之间存在少量错位的同一个卵母细胞。
     * 因此中心距离判据必须允许在无交集时生效，但阈值要受 overlapCandidate 约束。
     */
    if(centerDist <= 0.35f * refDiameter && areaRatio >= 0.25f)
        return true;

    if(centerInside && centerDist <= 0.55f * refDiameter && areaRatio >= 0.15f)
        return true;

    if(overlapCandidate && centerDist <= 0.65f * refDiameter && areaRatio >= 0.12f)
        return true;

    bool lowQualityCandidate = a.confidence < 0.45f || b.confidence < 0.45f;
    if(lowQualityCandidate && centerDist <= 0.50f * refDiameter && areaRatio >= 0.08f)
        return true;

    return false;
}



std::vector<DetectionBox> StepBuildMap::removeDuplicateOocytesByGeometry(
    std::vector<DetectionBox> boxes)
{
    if(boxes.empty())
        return boxes;

    /*
     * 排序逻辑：
     * 优先保留检测质量高的框。
     * 质量分数已经把边界框、长条框和过小框降权。
     * 不能再优先保留面积大的框，否则边界处的错误大框会压掉完整细胞框。
     */
    std::sort(boxes.begin(), boxes.end(),
              [](const DetectionBox& a, const DetectionBox& b)
              {
                  float scoreA = a.confidence;
                  float scoreB = b.confidence;

                  if(a.isReplaced)
                      scoreA += 0.03f;
                  if(b.isReplaced)
                      scoreB += 0.03f;

                  if(!a.isNearOverlap)
                      scoreA += 0.01f;
                  if(!b.isNearOverlap)
                      scoreB += 0.01f;

                  if(std::abs(scoreA - scoreB) > 1e-4f)
                      return scoreA > scoreB;

                  float areaA = static_cast<float>(a.box.area());
                  float areaB = static_cast<float>(b.box.area());

                  if(std::abs(areaA - areaB) > 1e-3f)
                      return areaA > areaB;

                  return false;
              });

    std::vector<DetectionBox> kept;

    for(const auto& box : boxes)
    {
        bool duplicate = false;

        for(const auto& oldBox : kept)
        {
            if(isDuplicateOocyteBox(box, oldBox))
            {
                duplicate = true;
                break;
            }
        }

        if(!duplicate)
            kept.push_back(box);
    }

    return kept;
}


void StepBuildMap::removeSmallBoxes(
    std::vector<DetectionBox>& boxes,
    int minSize)
{
    boxes.erase(
        remove_if(boxes.begin(), boxes.end(),
                  [minSize](const DetectionBox& box) {
                      return box.box.width < minSize || box.box.height < minSize;
                  }),
        boxes.end()
        );
}
void StepBuildMap::saveResults(
    const QString& rootPath,
    cv::Mat mapImg,
    std::vector<DetectionBox> boxes,
    const cv::Point2f& refCenterUm)
{
    if(!QDir(rootPath).exists())
        QDir().mkpath(rootPath);

    cv::Mat visImg;

    if(mapImg.channels() == 1)
        cvtColor(mapImg, visImg, COLOR_GRAY2BGR);
    else
        visImg = mapImg.clone();

    for(const auto& box : boxes)
    {
        cv::Scalar color = Scalar(0, 0, 255);

        cv::rectangle(visImg, box.box, color, 2);
        cv::circle(visImg, box.center, 3, color, -1);
    }

    QString globalMapPath = rootPath + "globalMap.jpg";
    QString globalVisPath = rootPath + "globalMap_visualized.jpg";
    QString centersPath = rootPath + "cell_centers.txt";
    QString centersUmPath = rootPath + "cell_centers_um.txt";

    bool ok1 = cv::imwrite(globalMapPath.toStdString(), mapImg);
    bool ok2 = cv::imwrite(globalVisPath.toStdString(), visImg);

    qDebug() << "Save global map:" << globalMapPath << ok1;
    qDebug() << "Save global vis:" << globalVisPath << ok2;

    cv::Mat displayImg = visImg;
    const int maxDisplayWidth = 1200;
    const int maxDisplayHeight = 800;
    double displayScale = std::min(
        1.0,
        std::min(
            static_cast<double>(maxDisplayWidth) / static_cast<double>(visImg.cols),
            static_cast<double>(maxDisplayHeight) / static_cast<double>(visImg.rows)
            )
        );

    if(displayScale < 1.0)
    {
        cv::resize(
            visImg,
            displayImg,
            cv::Size(),
            displayScale,
            displayScale,
            cv::INTER_AREA
            );
    }

    cv::namedWindow("Global Map Detection", cv::WINDOW_AUTOSIZE);
    cv::imshow("Global Map Detection", displayImg);
    cv::waitKey(1);

    std::ofstream centersFile(centersPath.toStdString());
    if(centersFile.is_open())
    {
        centersFile << "# id center_x_px center_y_px\n";

        for(size_t i = 0; i < boxes.size(); ++i)
        {
            const auto& box = boxes[i];

            centersFile << i << " "
                        << box.center.x << " "
                        << box.center.y << std::endl;
        }

        centersFile.close();
    }
    else
    {
        qDebug() << "Failed to open centers file:" << centersPath;
    }

    std::ofstream centersUmFile(centersUmPath.toStdString());
    if(centersUmFile.is_open())
    {
        centersUmFile << "# id center_x_px center_y_px center_x_um center_y_um\n";

        for(size_t i = 0; i < boxes.size(); ++i)
        {
            const auto& box = boxes[i];

            float centerXUm =
                refCenterUm.x +
                (box.center.x - WIDTH / 2.0f) * globalMap->pixel2Um;

            float centerYUm =
                refCenterUm.y +
                (box.center.y - HEIGHT / 2.0f) * globalMap->pixel2Um;

            centersUmFile << i << " "
                          << box.center.x << " "
                          << box.center.y << " "
                          << centerXUm << " "
                          << centerYUm << std::endl;
        }

        centersUmFile.close();
    }
    else
    {
        qDebug() << "Failed to open centers um file:" << centersUmPath;
    }
}

void StepBuildMap::saveTileDebugResult(
    const QString& rootPath,
    int row,
    int col,
    const cv::Mat& img,
    const std::vector<DetectionBox>& localDetections,
    const std::vector<DetectionBox>& globalDetections,
    const cv::Point2f& offset)
{
    if(img.empty())
        return;

    QString tileDir = rootPath + "tiles/";
    QString visDir = rootPath + "tiles_visualized/";
    QString txtDir = rootPath + "tiles_detections/";

    if(!QDir(tileDir).exists())
        QDir().mkpath(tileDir);

    if(!QDir(visDir).exists())
        QDir().mkpath(visDir);

    if(!QDir(txtDir).exists())
        QDir().mkpath(txtDir);

    QString baseName = QString("tile_r%1_c%2")
                           .arg(row, 3, 10, QChar('0'))
                           .arg(col, 3, 10, QChar('0'));

    QString rawImgPath = tileDir + baseName + ".jpg";
    QString visImgPath = visDir + baseName + "_det.jpg";
    QString localTxtPath = txtDir + baseName + "_local.txt";
    QString globalTxtPath = txtDir + baseName + "_global.txt";

    cv::imwrite(rawImgPath.toStdString(), img);

    cv::Mat visImg;
    if(img.channels() == 1)
        cv::cvtColor(img, visImg, cv::COLOR_GRAY2BGR);
    else
        visImg = img.clone();

    for(const auto& box : localDetections)
    {
        cv::Scalar color;

        if(box.isReplaced)
            color = cv::Scalar(0, 255, 0);
        else if(box.isNearOverlap)
            color = cv::Scalar(0, 255, 255);
        else
            color = cv::Scalar(0, 0, 255);

        cv::rectangle(visImg, box.box, color, 2);
        cv::circle(visImg, box.center, 3, color, -1);
    }

    cv::imwrite(visImgPath.toStdString(), visImg);

    std::ofstream localFile(localTxtPath.toStdString());
    if(localFile.is_open())
    {
        localFile << "# row col offset_x offset_y\n";
        localFile << row << " " << col << " "
                  << offset.x << " " << offset.y << "\n";

        localFile << "# id x y w h center_x center_y confidence near_overlap replaced\n";

        for(size_t i = 0; i < localDetections.size(); ++i)
        {
            const auto& box = localDetections[i];

            localFile << i << " "
                      << box.box.x << " "
                      << box.box.y << " "
                      << box.box.width << " "
                      << box.box.height << " "
                      << box.center.x << " "
                      << box.center.y << " "
                      << box.confidence << " "
                      << static_cast<int>(box.isNearOverlap) << " "
                      << static_cast<int>(box.isReplaced) << "\n";
        }

        localFile.close();
    }

    std::ofstream globalFile(globalTxtPath.toStdString());
    if(globalFile.is_open())
    {
        globalFile << "# row col offset_x offset_y\n";
        globalFile << row << " " << col << " "
                   << offset.x << " " << offset.y << "\n";

        globalFile << "# id global_x global_y global_w global_h global_center_x global_center_y confidence near_overlap replaced\n";

        for(size_t i = 0; i < globalDetections.size(); ++i)
        {
            const auto& box = globalDetections[i];

            globalFile << i << " "
                       << box.box.x << " "
                       << box.box.y << " "
                       << box.box.width << " "
                       << box.box.height << " "
                       << box.center.x << " "
                       << box.center.y << " "
                       << box.confidence << " "
                       << static_cast<int>(box.isNearOverlap) << " "
                       << static_cast<int>(box.isReplaced) << "\n";
        }

        globalFile.close();
    }
}


WORKSTEP StepBuildMap::run()
{
    isWorking = true;

    injectPipette->find(CurrentBGR);
    if(injectPipette->isDetected)
    {
        moveInjWait(0, 0, -areaMoveHeight, MANIP_VEL);
        moveInjWait(0,
                    (0.5 * HEIGHT - injectPipette->tip.y - 1.0 * injectPipette->diameter) * Pixel2Um,
                    0,
                    MANIP_VEL);
        moveInjWait((WIDTH - injectPipette->xm) * Pixel2Um, 0, 0, MANIP_VEL);
    }

    holdPipette->find(CurrentBGR);
    if(holdPipette->isDetected)
    {
        moveHoldWait(0, 0, -areaMoveHeight, MANIP_VEL);
        moveHoldWait(0,
                     (0.6 * HEIGHT - holdPipette->tip.y) * Pixel2Um,
                     0,
                     MANIP_VEL);
        moveHoldWait(-holdPipette->xM * Pixel2Um - 200, 0, 0, MANIP_VEL);
    }

    oocyteArea = Point2f(microscope->getRelX(), microscope->getRelY());
    globalMap->pixel2Um = Pixel2Um;

    build();

    emit moveStage(0, 0.8 * HEIGHT * Pixel2Um, STAGE_VEL);

    if(holdPipette->isDetected)
    {
        moveHoldWait(0.32 * WIDTH_UM + 200, 0, 0, MANIP_VEL);
        moveHoldWait(0, 0, areaMoveHeight, MANIP_VEL);
    }

    if(injectPipette->isDetected)
    {
        moveInjWait(-0.44 * WIDTH_UM, 0, 0, MANIP_VEL);
        moveInjWait(0, 0, areaMoveHeight, MANIP_VEL);
    }

    allOocytes->reset();

    for(const auto& box : allDetections)
    {
        Oocyte oocyte;
        oocyte.center = box.center;
        oocyte.isDetected = true;
        oocyte.centerUm = Point2f(
            oocyteArea.x + (box.center.x - WIDTH / 2) * globalMap->pixel2Um,
            oocyteArea.y + (box.center.y - HEIGHT / 2) * globalMap->pixel2Um
            );
        allOocytes->oocytes.push_back(oocyte);
    }

    std::sort(allOocytes->oocytes.begin(), allOocytes->oocytes.end(),
              [](const Oocyte& left, const Oocyte& right)
              {
                  return left.centerUm.y < right.centerUm.y;
              });
    allOocytes->isDetected = !allOocytes->oocytes.empty();

    if(!areAllOocytesEnucleated)
        nextStep = FETCHOOCYTE;
    else
        nextStep = FETCHOOCYTEFORSOMA;

    isWorking = false;
    emit finish();

    return nextStep;
}



void StepBuildMap::build()
{
    QDateTime dateTime(QDateTime::currentDateTime());
    QString time = dateTime.toString("yyyy-MM-dd-hh-mm-ss");
    QString path = FILE_PATH + QString("/map-%1/").arg(time);

    if(!QDir(path).exists())
        QDir().mkpath(path);

    cv::Mat img = captureImage();
    if(img.empty())
    {
        qDebug() << "Build map failed: first image is empty.";
        return;
    }

    int viewW = img.cols;
    int viewH = img.rows;

    int nw = targetCols;
    int nh = targetRows;

    if(nw <= 0)
    {
        if(GMAP_WIDTH_UM > 0.0f && Pixel2Um > 0.0f)
            nw = std::max(1, static_cast<int>(std::round((GMAP_WIDTH_UM / Pixel2Um) / viewW)));
        else
            nw = 1;
    }

    if(nh <= 0)
    {
        if(GMAP_HEIGHT_UM > 0.0f && Pixel2Um > 0.0f)
            nh = std::max(1, static_cast<int>(std::round((GMAP_HEIGHT_UM / Pixel2Um) / viewH)));
        else
            nh = 1;
    }

    if(nw <= 0)
        nw = 1;

    if(nh <= 0)
        nh = 1;

    if(overlapRatio < 0.0f)
        overlapRatio = 0.0f;

    if(overlapRatio >= 1.0f)
        overlapRatio = 0.5f;

    float stepX = static_cast<float>(viewW) * (1.0f - overlapRatio);
    float stepY = static_cast<float>(viewH) * (1.0f - overlapRatio);

    if(nw == 1)
        stepX = 0.0f;

    if(nh == 1)
        stepY = 0.0f;

    stepDW = stepX;
    stepDH = stepY;
    overlapPixelsH = static_cast<float>(viewW) - stepX;
    overlapPixelsV = static_cast<float>(viewH) - stepY;

    GMAP_WIDTH = static_cast<int>(std::ceil(static_cast<float>(viewW) + (nw - 1) * stepX));
    GMAP_HEIGHT = static_cast<int>(std::ceil(static_cast<float>(viewH) + (nh - 1) * stepY));

    GMAP_WIDTH = std::max(GMAP_WIDTH, viewW);
    GMAP_HEIGHT = std::max(GMAP_HEIGHT, viewH);

    GMAP_WIDTH_UM = GMAP_WIDTH * Pixel2Um;
    GMAP_HEIGHT_UM = GMAP_HEIGHT * Pixel2Um;

    computeDynamicSteps();

    qDebug() << "Build map parameters:";
    qDebug() << "cols:" << nw
             << "rows:" << nh
             << "viewW:" << viewW
             << "viewH:" << viewH
             << "stepX:" << stepX
             << "stepY:" << stepY
             << "overlapX:" << overlapPixelsH
             << "overlapY:" << overlapPixelsV
             << "GMAP_WIDTH:" << GMAP_WIDTH
             << "GMAP_HEIGHT:" << GMAP_HEIGHT;

    globalMap->mapImage = cv::Mat::zeros(Size(GMAP_WIDTH, GMAP_HEIGHT), CV_8UC1);
    validMask = cv::Mat::zeros(Size(GMAP_WIDTH, GMAP_HEIGHT), CV_8UC1);
    StepBuildMapInternal::gWeightMap = cv::Mat::zeros(Size(GMAP_WIDTH, GMAP_HEIGHT), CV_32FC1);
    globalMap->pixel2Um = Pixel2Um;
    globalMap->widthPx = GMAP_WIDTH;
    globalMap->heightPx = GMAP_HEIGHT;
    globalMap->widthUm = GMAP_WIDTH_UM;
    globalMap->heightUm = GMAP_HEIGHT_UM;
    globalMap->originUm = cv::Point2f(0.0f, 0.0f);
    globalMap->refCenterUm = oocyteArea;
    globalMap->isValid = false;

    allDetections.clear();
    cellCenters.clear();

    /*
     * 使用理论扫描位置控制载物台。
     * 使用显微镜实际坐标换算得到的 mapOffset 进行拼图和检测框坐标转换。
     * 若设备坐标方向与图像拼接方向相反，将对应 sign 改为 -1.0f。
     */
    const bool useStageMeasuredOffset = true;

    /*
     * false 为蛇形扫描。
     * true 为每一行从左到右扫描，可用于验证反向运动间隙是否是主要误差来源。
     */
    const bool useUnidirectionalScan = false;

    const float stageToMapSignX = 1.0f;
    const float stageToMapSignY = 1.0f;

    const float minPixel2Um = 1e-6f;
    const float maxAcceptableStepErrorPx =
        std::max(80.0f, 0.25f * std::max(std::abs(stepX), std::abs(stepY)));

    auto readStageUm = [this]() -> cv::Point2f
    {
        return cv::Point2f(
            static_cast<float>(microscope->getRelX()),
            static_cast<float>(microscope->getRelY())
            );
    };

    cv::Point2f startStageUm = readStageUm();

    cv::Point2f prevPlannedOffset(0.0f, 0.0f);
    cv::Point2f prevMapOffset(0.0f, 0.0f);

    std::vector<DetectionBox> firstLocalDetections = detectYOLO(img);

    std::vector<DetectionBox> firstDetections =
        convertToGlobalCoords(firstLocalDetections, cv::Point2f(0.0f, 0.0f));
    allDetections.insert(allDetections.end(), firstDetections.begin(), firstDetections.end());
    saveTileDebugResult(
        path,
        0,
        0,
        img,
        firstLocalDetections,
        firstDetections,
        cv::Point2f(0.0f, 0.0f)
        );

    cv::Rect firstRoi(0, 0, std::min(viewW, GMAP_WIDTH), std::min(viewH, GMAP_HEIGHT));
    img(firstRoi).copyTo(globalMap->mapImage(firstRoi));
    validMask(firstRoi).setTo(255);

    {
        int weightEdgeMargin = static_cast<int>(
            std::round(std::max(overlapPixelsH, overlapPixelsV))
            );
        weightEdgeMargin = std::max(16, weightEdgeMargin);

        cv::Mat firstTileWeight = StepBuildMapInternal::buildTileWeightMap(img.size(), weightEdgeMargin);
        firstTileWeight(firstRoi).copyTo(StepBuildMapInternal::gWeightMap(firstRoi));
    }

    emit mapImageUpdated(globalMap->mapImage.clone());

    ON = true;

    auto waitStageSettled = [this](int timeoutMs,
                                   float stableEpsUm,
                                   int stableNeedCount,
                                   int sampleIntervalMs) -> bool
    {
        QElapsedTimer timer;
        timer.start();

        cv::Point2f lastPos(
            static_cast<float>(microscope->getRelX()),
            static_cast<float>(microscope->getRelY())
            );

        int stableCount = 0;

        while(timer.elapsed() < timeoutMs)
        {
            QThread::msleep(sampleIntervalMs);

            cv::Point2f currPos(
                static_cast<float>(microscope->getRelX()),
                static_cast<float>(microscope->getRelY())
                );

            float dx = currPos.x - lastPos.x;
            float dy = currPos.y - lastPos.y;
            float dist = std::sqrt(dx * dx + dy * dy);

            if(dist <= stableEpsUm)
                stableCount++;
            else
                stableCount = 0;

            if(stableCount >= stableNeedCount)
                return true;

            lastPos = currPos;
        }

        qDebug() << "Stage settle wait timeout.";
        return false;
    };

    auto moveStageAndWait = [this, &waitStageSettled](float dxUm,
                                                      float dyUm,
                                                      float moveDistPx)
    {
        emit this->moveStage(dxUm, dyUm, STAGE_VEL);

        float pixelVel = static_cast<float>(PIXEL_VEL);
        if(pixelVel < 1.0f)
            pixelVel = 1.0f;

        int estimatedMsByPixel = static_cast<int>(
            2.0f * 1000.0f * moveDistPx / pixelVel + 500.0f
            );

        float moveDistUm = std::sqrt(dxUm * dxUm + dyUm * dyUm);
        float stageVel = static_cast<float>(STAGE_VEL);
        if(stageVel < 1.0f)
            stageVel = 1.0f;

        int estimatedMsByStage = static_cast<int>(
            1000.0f * moveDistUm / stageVel + 800.0f
            );

        int firstWaitMs = std::max(estimatedMsByPixel, estimatedMsByStage);
        firstWaitMs = std::max(firstWaitMs, 800);

        QThread::msleep(firstWaitMs);

        waitStageSettled(8000, 0.5f, 6, 50);

        /*
         * 机械坐标稳定后，液体与图像帧仍可能存在短暂滞后。
         * 这里再等待一小段时间，让画面稳定后再进入 captureImage。
         */
        QThread::msleep(300);
    };

    for(int row = 0; row < nh && ON; ++row)
    {
        for(int k = 0; k < nw && ON; ++k)
        {
            int col;

            if(useUnidirectionalScan)
                col = k;
            else if(row % 2 == 0)
                col = k;
            else
                col = nw - 1 - k;

            if(row == 0 && col == 0)
                continue;

            cv::Point2f plannedOffset(
                static_cast<float>(col) * stepX,
                static_cast<float>(row) * stepY
                );

            float moveX = plannedOffset.x - prevPlannedOffset.x;
            float moveY = plannedOffset.y - prevPlannedOffset.y;

            qDebug() << "Move to row:" << row
                     << "col:" << col
                     << "moveX(px):" << moveX
                     << "moveY(px):" << moveY
                     << "prevPlannedOffset:" << prevPlannedOffset.x << prevPlannedOffset.y
                     << "plannedOffset:" << plannedOffset.x << plannedOffset.y
                     << "prevMapOffset:" << prevMapOffset.x << prevMapOffset.y;

            cv::Point2f stageBeforeUm = readStageUm();

            float moveDist = std::max(std::abs(moveX), std::abs(moveY));
            moveStageAndWait(moveX * Pixel2Um, moveY * Pixel2Um, moveDist);

            cv::Point2f stageAfterUm = readStageUm();

            cv::Point2f actualStepPx(0.0f, 0.0f);
            cv::Point2f measuredOffset(0.0f, 0.0f);

            bool canUseStageCoordinate = Pixel2Um > minPixel2Um;

            if(canUseStageCoordinate)
            {
                actualStepPx.x =
                    stageToMapSignX * (stageAfterUm.x - stageBeforeUm.x) / Pixel2Um;
                actualStepPx.y =
                    stageToMapSignY * (stageAfterUm.y - stageBeforeUm.y) / Pixel2Um;

                measuredOffset.x =
                    stageToMapSignX * (stageAfterUm.x - startStageUm.x) / Pixel2Um;
                measuredOffset.y =
                    stageToMapSignY * (stageAfterUm.y - startStageUm.y) / Pixel2Um;
            }

            cv::Point2f stepError(
                actualStepPx.x - moveX,
                actualStepPx.y - moveY
                );

            cv::Point2f absoluteError(
                measuredOffset.x - plannedOffset.x,
                measuredOffset.y - plannedOffset.y
                );

            bool stepReliable = canUseStageCoordinate;

            if(std::abs(moveX) > 1.0f && std::abs(stepError.x) > maxAcceptableStepErrorPx)
                stepReliable = false;

            if(std::abs(moveY) > 1.0f && std::abs(stepError.y) > maxAcceptableStepErrorPx)
                stepReliable = false;

            if(!std::isfinite(measuredOffset.x) || !std::isfinite(measuredOffset.y))
                stepReliable = false;

            cv::Point2f currOffset = plannedOffset;

            if(useStageMeasuredOffset && stepReliable)
            {
                currOffset = measuredOffset;
            }
            else
            {
                currOffset = plannedOffset;
            }

            /*
             * 当前 extendMap 不处理负方向扩展。
             * 若实际坐标存在很小负偏移，直接截断到零，避免图像和检测框坐标系不一致。
             */
            if(currOffset.x < 0.0f && currOffset.x > -20.0f)
                currOffset.x = 0.0f;

            if(currOffset.y < 0.0f && currOffset.y > -20.0f)
                currOffset.y = 0.0f;

            qDebug() << "Stage move report:"
                     << "plannedStepPx" << moveX << moveY
                     << "actualStepPx" << actualStepPx.x << actualStepPx.y
                     << "stepErrorPx" << stepError.x << stepError.y
                     << "measuredOffsetPx" << measuredOffset.x << measuredOffset.y
                     << "plannedOffsetPx" << plannedOffset.x << plannedOffset.y
                     << "absoluteErrorPx" << absoluteError.x << absoluteError.y
                     << "chosenMapOffsetPx" << currOffset.x << currOffset.y
                     << "useStageOffset" << (useStageMeasuredOffset && stepReliable);

            img = captureImage();
            if(img.empty())
            {
                qDebug() << "Warning: captured image is empty at row:" << row << "col:" << col;
                prevPlannedOffset = plannedOffset;
                prevMapOffset = currOffset;
                continue;
            }

            std::vector<DetectionBox> localDetections = detectYOLO(img);

            std::vector<DetectionBox> currentDetections =
                convertToGlobalCoords(localDetections, currOffset);

            std::vector<cv::Rect> globalOverlapRects =
                getGlobalCoveredOverlapRects(currOffset, img.size());

            std::vector<cv::Rect> localOverlapRects =
                getLocalCoveredOverlapRects(currOffset, img.size());

            if(!globalOverlapRects.empty())
            {
                markNearOverlapByRects(currentDetections, globalOverlapRects);
            }

            std::vector<DetectionBox> allOverlapDetections;

            for(const auto& localOverlapRect : localOverlapRects)
            {
                if(localOverlapRect.area() <= 0)
                    continue;

                std::vector<DetectionBox> overlapDetections =
                    detectOverlapArea(img, localOverlapRect);

                overlapDetections = convertToGlobalCoords(overlapDetections, currOffset);

                allOverlapDetections.insert(
                    allOverlapDetections.end(),
                    overlapDetections.begin(),
                    overlapDetections.end()
                    );
            }

            if(!allOverlapDetections.empty())
            {
                replaceWithRedetection(currentDetections, allOverlapDetections);
            }

            allDetections.insert(allDetections.end(), currentDetections.begin(), currentDetections.end());

            extendMap(prevMapOffset, currOffset, img);
            emit mapImageUpdated(globalMap->mapImage.clone());

            saveTileDebugResult(
                path,
                row,
                col,
                img,
                localDetections,
                currentDetections,
                currOffset
                );

            prevPlannedOffset = plannedOffset;
            prevMapOffset = currOffset;
        }
    }

    removeSmallBoxes(allDetections, minBoxSize);

    /*
     * 先做几何去重，再做 NMS。
     * 原来的第一次 NMS 会在所有 confidence 相同或接近时随机保留边界误检，
     * 导致后续几何去重已经看不到正确框。
     */
    allDetections = removeDuplicateOocytesByGeometry(allDetections);

    allDetections = nmsByIoU(allDetections, iouThreshold);

    allDetections = removeDuplicateOocytesByGeometry(allDetections);

    cellCenters.clear();
    for(const auto& box : allDetections)
    {
        cellCenters.push_back(box.center);
    }

    saveResults(path, globalMap->mapImage, allDetections, oocyteArea);

    globalMap->updateThumbnail();
    globalMap->saveToFile(FILE_PATH + "/globalMap.png");

    qDebug() << "Map building completed. Total cells detected:" << allDetections.size();
}

void StepBuildMap::extendMap(
    const cv::Point2f& prevOffset,
    const cv::Point2f& currOffset,
    const cv::Mat& img)
{
    (void)prevOffset;

    if(img.empty())
        return;

    int ox = static_cast<int>(std::round(currOffset.x));
    int oy = static_cast<int>(std::round(currOffset.y));

    if(ox < 0 || oy < 0)
    {
        qDebug() << "extendMap skipped: negative offset is not supported."
                 << "offset" << currOffset.x << currOffset.y;
        return;
    }

    cv::Mat preMap = globalMap->mapImage.clone();
    cv::Mat preMask = validMask.clone();

    cv::Mat preWeight;
    if(!StepBuildMapInternal::gWeightMap.empty() &&
        StepBuildMapInternal::gWeightMap.type() == CV_32FC1 &&
        StepBuildMapInternal::gWeightMap.rows == preMap.rows &&
        StepBuildMapInternal::gWeightMap.cols == preMap.cols)
    {
        preWeight = StepBuildMapInternal::gWeightMap.clone();
    }
    else
    {
        preWeight = cv::Mat::zeros(preMap.size(), CV_32FC1);
    }

    int newRows = std::max(preMap.rows, oy + img.rows);
    int newCols = std::max(preMap.cols, ox + img.cols);

    cv::Mat tmpMap = cv::Mat::zeros(newRows, newCols, preMap.type());
    cv::Mat tmpMask = cv::Mat::zeros(newRows, newCols, CV_8UC1);
    cv::Mat tmpWeight = cv::Mat::zeros(newRows, newCols, CV_32FC1);

    preMap.copyTo(tmpMap(cv::Rect(0, 0, preMap.cols, preMap.rows)));

    if(!preMask.empty())
    {
        preMask.copyTo(tmpMask(cv::Rect(0, 0, preMask.cols, preMask.rows)));
    }

    if(!preWeight.empty())
    {
        preWeight.copyTo(tmpWeight(cv::Rect(0, 0, preWeight.cols, preWeight.rows)));
    }

    cv::Mat adjustedImg = img.clone();

    /*
     * Step 1:
     * Use the actual overlap between the current tile and the existing map to
     * estimate a global linear brightness correction.
     */
    {
        std::vector<float> oldPixels;
        std::vector<float> newPixels;

        oldPixels.reserve(img.rows * img.cols / 4);
        newPixels.reserve(img.rows * img.cols / 4);

        for(int i = 0; i < img.rows; ++i)
        {
            for(int j = 0; j < img.cols; ++j)
            {
                int globalX = ox + j;
                int globalY = oy + i;

                if(globalX < 0 || globalX >= tmpMap.cols ||
                    globalY < 0 || globalY >= tmpMap.rows)
                {
                    continue;
                }

                bool hasOldPixel = tmpMask.at<uchar>(globalY, globalX) > 0;
                if(!hasOldPixel)
                    continue;

                uchar oldVal = tmpMap.at<uchar>(globalY, globalX);
                uchar newVal = img.at<uchar>(i, j);

                if(oldVal < 10 || oldVal > 245 || newVal < 10 || newVal > 245)
                    continue;

                oldPixels.push_back(static_cast<float>(oldVal));
                newPixels.push_back(static_cast<float>(newVal));
            }
        }

        if(oldPixels.size() > 200)
        {
            double oldSum = 0.0;
            double newSum = 0.0;

            for(size_t k = 0; k < oldPixels.size(); ++k)
            {
                oldSum += oldPixels[k];
                newSum += newPixels[k];
            }

            double oldMean = oldSum / static_cast<double>(oldPixels.size());
            double newMean = newSum / static_cast<double>(newPixels.size());

            double oldVar = 0.0;
            double newVar = 0.0;

            for(size_t k = 0; k < oldPixels.size(); ++k)
            {
                oldVar += (oldPixels[k] - oldMean) * (oldPixels[k] - oldMean);
                newVar += (newPixels[k] - newMean) * (newPixels[k] - newMean);
            }

            oldVar /= static_cast<double>(oldPixels.size());
            newVar /= static_cast<double>(newPixels.size());

            double oldStd = std::sqrt(oldVar);
            double newStd = std::sqrt(newVar);

            double gain = 1.0;
            if(newStd > 1e-6)
                gain = oldStd / newStd;

            if(gain < 0.75)
                gain = 0.75;
            if(gain > 1.25)
                gain = 1.25;

            double bias = oldMean - gain * newMean;

            cv::Mat img32;
            img.convertTo(img32, CV_32F);
            img32 = img32 * gain + bias;
            img32.convertTo(adjustedImg, CV_8U);

            qDebug() << "Brightness match:"
                     << "oldMean" << oldMean
                     << "newMean" << newMean
                     << "gain" << gain
                     << "bias" << bias
                     << "samples" << static_cast<int>(oldPixels.size());
        }
        else
        {
            qDebug() << "Brightness match skipped, insufficient overlap samples:"
                     << static_cast<int>(oldPixels.size());
        }
    }

    /*
     * Step 2:
     * Weight-map fusion. Pixels near the tile boundary receive a smaller
     * reliability weight, while pixels near the tile center receive a larger
     * reliability weight. This prevents later boundary fragments from directly
     * overwriting complete cells that were already placed into the global map.
     */
    int weightEdgeMargin = static_cast<int>(
        std::round(std::max(overlapPixelsH, overlapPixelsV))
        );
    weightEdgeMargin = std::max(16, weightEdgeMargin);

    cv::Mat tileWeight = StepBuildMapInternal::buildTileWeightMap(adjustedImg.size(), weightEdgeMargin);

    const float minTotalWeight = 1e-6f;

    for(int i = 0; i < adjustedImg.rows; ++i)
    {
        for(int j = 0; j < adjustedImg.cols; ++j)
        {
            int globalX = ox + j;
            int globalY = oy + i;

            if(globalX < 0 || globalX >= tmpMap.cols ||
                globalY < 0 || globalY >= tmpMap.rows)
            {
                continue;
            }

            uchar newVal = adjustedImg.at<uchar>(i, j);
            float newWeight = tileWeight.at<float>(i, j);
            bool hasOldPixel = tmpMask.at<uchar>(globalY, globalX) > 0;

            /*
 * 在重叠区域降低新 tile 的融合权重，避免后来的图像反复平均旧图像，
 * 从而减少细胞边缘虚化。只对已有像素区域生效，非重叠区域仍正常写入。
 */
            const float overlapNewWeightScale = 0.15f;

            if(hasOldPixel)
            {
                newWeight *= overlapNewWeightScale;
            }

            if(!hasOldPixel)
            {
                tmpMap.at<uchar>(globalY, globalX) = newVal;
                tmpMask.at<uchar>(globalY, globalX) = 255;
                tmpWeight.at<float>(globalY, globalX) = newWeight;
                continue;
            }

            uchar oldVal = tmpMap.at<uchar>(globalY, globalX);
            float oldWeight = tmpWeight.at<float>(globalY, globalX);

            if(oldWeight < minTotalWeight)
                oldWeight = 1.0f;

            float totalWeight = oldWeight + newWeight;

            if(totalWeight < minTotalWeight)
            {
                tmpMap.at<uchar>(globalY, globalX) = newVal;
                tmpWeight.at<float>(globalY, globalX) = newWeight;
            }
            else
            {
                float fusedValue =
                    (oldWeight * static_cast<float>(oldVal) +
                     newWeight * static_cast<float>(newVal)) /
                    totalWeight;

                tmpMap.at<uchar>(globalY, globalX) =
                    cv::saturate_cast<uchar>(fusedValue);

                tmpWeight.at<float>(globalY, globalX) = totalWeight;
            }

            tmpMask.at<uchar>(globalY, globalX) = 255;
        }
    }

    globalMap->mapImage = tmpMap;
    validMask = tmpMask;
    StepBuildMapInternal::gWeightMap = tmpWeight;

    globalMap->widthPx = tmpMap.cols;
    globalMap->heightPx = tmpMap.rows;
    globalMap->widthUm = globalMap->widthPx * globalMap->pixel2Um;
    globalMap->heightUm = globalMap->heightPx * globalMap->pixel2Um;
}
