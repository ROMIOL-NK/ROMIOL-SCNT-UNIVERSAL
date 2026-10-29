#include "steppenetrate.h"
using namespace cv;
StepPenetrate::StepPenetrate(){}

void StepPenetrate::getxy(cv::Mat img, int& x, int& y) {
    int count = 0;
    int ahai = 0;
    int ahaj = 0;
    int gray = 0;
    for (int i = 0; i < img.rows; i++) {
        for (int j = 0; j < img.cols; j++) {
            if (img.at<uchar>(i, j) >= 125) {
                count++;
                ahai += i;
                ahaj += j;
            }
        }
    }
    int ahay, ahax;
    if (count != 0) {
        ahay = ahai / count;
        ahax = ahaj / count;
    }
    else {
        ahay = 0;
        ahax = 0;
    }

    x = round(ahax);
    y = round(ahay);
}

int StepPenetrate::get_center(cv::Mat img, int cy_x, int cy_y, int cy_r) {
    //cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
    cv::Mat img1;
    cv::Rect roi(cy_x - int(1.1 * cy_r), cy_y - int(1.5 * cy_r), int(2.2 * cy_r), int(3 * cy_r));
    roi &= Rect(0,0,img.cols, img.rows);
    img1 = img(roi);
    // cv::imshow("roi",img1);
    cv::Mat dst2;
    cv::threshold(img1, dst2, 0, 255, cv::THRESH_OTSU);
    dst2 = 255 - dst2;
    cv::Mat kernel = cv::Mat::ones(3, 3, CV_8U);
    cv::dilate(dst2, dst2, kernel, cv::Point(-1, -1), 3);
    kernel = cv::Mat::ones(4, 4, CV_8U);
    cv::erode(dst2, dst2, kernel, cv::Point(-1, -1), 3);
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(dst2, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
    if (contours.size() > 1) {
        std::vector<std::vector<cv::Point>> c_max;
        double max_area = 0;
        int max_cnt = 0;
        for (int i = 0; i < contours.size(); i++) {
            std::vector<cv::Point> cnt = contours[i];
            double area = cv::contourArea(cnt);
            if (area > max_area) {
                if (max_area != 0) {
                    std::vector<std::vector<cv::Point>> c_min;
                    c_min.push_back(contours[max_cnt]);
                    cv::drawContours(dst2, c_min, -1, cv::Scalar(0, 0, 0), cv::FILLED);
                }
                max_area = area;
                max_cnt = i;
            }
            else {
                std::vector<std::vector<cv::Point>> c_min;
                c_min.push_back(cnt);
                cv::drawContours(dst2, c_min, -1, cv::Scalar(0, 0, 0), cv::FILLED);
            }
        }
        c_max.push_back(contours[max_cnt]);

    }
    kernel = cv::Mat::ones(13, 13, CV_8U);
    cv::erode(dst2, dst2, kernel, cv::Point(-1, -1), 3);
    kernel = cv::Mat::ones(14, 14, CV_8U);
    cv::dilate(dst2, dst2, kernel, cv::Point(-1, -1), 3);
    cv::Rect roi_1(10, 0, dst2.cols - 10, dst2.rows);
    roi_1 &= Rect(0, 0, img.cols, img.rows);
    dst2 = dst2(roi_1);
    imshow("dst2", dst2); waitKey(1);
    int x, y;
    getxy(dst2, x, y);
    return x;
}

WORKSTEP StepPenetrate::run()
{
    isWorking = true;
    // emit setHoldPressureTarget(holdPenetrateP);
    //添加处理代码: 将卵细胞吸紧，刺破卵细胞，回撤一定距离
    QThread::msleep(1000);
    currentOocyte->find(CurrentGray);
    injectPipette->find(CurrentGray);
    holdPipette->find(CurrentGray);
    if(!currentOocyte->isDetected || !injectPipette->isDetected || !holdPipette->isDetected)
    {
        DEAL_FAULT("请手动检测卵母细胞|注射针|吸持针");
        currentOocyte->find(CurrentGray);
        injectPipette->find(CurrentGray);
        holdPipette->find(CurrentGray);
    }
    if(!currentOocyte->isDetected || !injectPipette->isDetected || !holdPipette->isDetected)
    {
        exceptionInfo = "去核穿刺前无法重新检测到卵母细胞、注射针或吸持针";
        nextStep = EXCEPTION;
        isWorking = false;
        emit finish();
        return nextStep;
    }
    //添加代码:获取刺入环节开始时的第一帧图像img1，用于获取ROI
    Mat src = CurrentGray.clone();
    int cy_x, cy_y, cy_r;
    // cv::Mat dst1;
    // cv::threshold(src, dst1, 0, 255, cv::THRESH_OTSU);
    // std::vector<cv::Vec3f> circles;
    // cv::HoughCircles(dst1, circles, cv::HOUGH_GRADIENT, 1, 5000, 100, 10, 10, 1000);
    // cv::Vec3f circle = circles[0];
    // cy_x = int(circle[0]);
    // cy_y = int(circle[1]);
    // cy_r = int(circle[2]);
    if(injectPipette->tip.x<currentOocyte->xM+currentOocyte->radius)//注射针针尖在卵细胞左侧，先移到右侧
    {
        moveInjWait((currentOocyte->xM-injectPipette->tip.x)*Pixel2Um+50, 0, 0, MANIP_VEL);
    }
    //添加代码:设置注射针刺入速度
    moveInjWait(0, (currentOocyte->center.y-injectPipette->tip.y)*Pixel2Um, 0, MANIP_VEL);

    // 穿刺预定位后重新检测，用当前双针位置计算吸持针针口边界。
    currentOocyte->find(CurrentGray);
    injectPipette->find(CurrentGray);
    holdPipette->find(CurrentGray);
    if(!currentOocyte->isDetected || !injectPipette->isDetected || !holdPipette->isDetected)
    {
        exceptionInfo = "去核穿刺预定位后无法检测到卵母细胞、注射针或吸持针";
        nextStep = EXCEPTION;
        isWorking = false;
        emit finish();
        return nextStep;
    }

    cy_x = currentOocyte->center.x;
    cy_y = currentOocyte->center.y;
    cy_r = currentOocyte->radius;

    const float holdSafetyMargin = 0.0f;
    double x0Inj = injector->manipulator->getAbsX();
    float holdSafetyTipX = holdPipette->xM+holdSafetyMargin/Pixel2Um;
    float availablePenetration = (injectPipette->tip.x-holdSafetyTipX)*Pixel2Um;
    double holdSafetyAbsX = x0Inj-availablePenetration;
    bool holdSafetyReached = availablePenetration<=0;
    if(!holdSafetyReached)
    {
        emit moveInjX(0, -60, true);
        int initialMoveDelay = int(1000*(availablePenetration-5.0f)/60.0f);
        if(initialMoveDelay>200) initialMoveDelay = 200;
        if(initialMoveDelay>0) QThread::msleep(initialMoveDelay);
    }

    int puncture_threshold = 5;
    int stable_threshold = 20;
    bool stable = true;
    int init = -1;
    int min_x = -1;
    ON = true;
    while (ON && !holdSafetyReached)
    {
        if(injector->manipulator->getAbsX()<=holdSafetyAbsX)
        {
            holdSafetyReached = true;
            break;
        }
        if(injector->manipulator->getAbsX()<x0Inj-(injectPipette->xm-currentOocyte->xMZP)*Pixel2Um-0.9*(currentOocyte->xMZP-currentOocyte->xmZP)*Pixel2Um)//扎透|0.9，停止
        {
            qDebug()<<"0.9 break";
            qDebug()<<"injAbsX: "<<injector->manipulator->getAbsX()<<"x0InJ: "<<x0Inj<<"inj xm: "<<injectPipette->xm<<"oocyte xm: "<<currentOocyte->xm<<"Pixel2Um: "<<Pixel2Um;
            break;
        }
        //添加代码:获取当前图片img
        int now = get_center(CurrentGray.clone(), cy_x, cy_y, cy_r);
        if(injector->manipulator->getAbsX()<=holdSafetyAbsX)
        {
            holdSafetyReached = true;
            break;
        }
        // qDebug()<<now<<", ";
        if (init == -1) {
            init = now;
            min_x = now;
        }
        if (now < min_x) { min_x = now; }
        if (stable && abs(now - init) > stable_threshold) { stable = false; }
        if (!stable && ((now - min_x) >= puncture_threshold))
            break;
    }
    emit moveInjX(0, 0, true);
    if(holdSafetyReached)
        qDebug()<<"hold pipette safety limit reached: "<<holdSafetyAbsX;
    QThread::sleep(1);

    //撤针距离重新计算
    // 原回撤量末尾减 20 um；再减 10 um，使去核穿刺后的小回撤比原来少 10 um。
    // double withdrawDis = (x0Inj-injector->manipulator->getAbsX())-(injectPipette->xm-currentOocyte->xM)*Pixel2Um-20;
    double withdrawDis = (x0Inj-injector->manipulator->getAbsX())-(injectPipette->xm-currentOocyte->xM)*Pixel2Um-30;
    // double withdrawDis = 5;
    if(withdrawDis>0)
        moveInjWait(withdrawDis, 0, 0, 250);
    //------------------------------------------------------步骤完成---------------------------------------------------
    nextStep = ENUCLEATION;
    isWorking = false;
    emit finish();
    return nextStep;
}
