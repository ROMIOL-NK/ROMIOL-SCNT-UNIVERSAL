#include "autofocus.h"
#include "WorkFlow/vision.h"

#include <QDebug>

using namespace cv;

AutoFocus::AutoFocus(QObject *parent)
    : QObject(parent)
{
    ON = true;
    running = false;
    moveSpeed = 100;
    maxIterations = 30;
    maxTravel = 100;
    requiredConfirm = 3;
    injectorFocusedStep = 2;
    injectorUnfocusStep = 5;
    holderStep = 5;
    holderDropLimit = 3;
}

void AutoFocus::debugLog(const QString &message, bool updateStatus)
{
    bool shouldPrint = message.contains(" result: iteration=")
            || message.contains(" initial:")
            || message.contains("auto focus finished")
            || message.contains("auto focus failed")
            || message.contains("focus exit:")
            || message.contains("safety travel limit exceeded")
            || message.contains("Stop request received");
    if(shouldPrint)
        qDebug().noquote() << "[AutoFocus]" << message;
    if(updateStatus)
        emit status(message);
}

QString AutoFocus::targetName(Target target) const
{
    return target == InjectorTarget ? "Injector" : "Holder";
}

void AutoFocus::runInjector()
{
    if(running)
    {
        debugLog("Auto focus is already running");
        return;
    }

    running = true;
    ON = true;
    debugLog("Injector auto focus started");
    bool ok = focusInjector();
    running = false;
    debugLog(ok ? "Injector auto focus finished" : "Injector auto focus failed");
    emit finish(ok ? "Injector auto focus finished" : "Injector auto focus failed");
}

void AutoFocus::runHolder()
{
    if(running)
    {
        debugLog("Auto focus is already running");
        return;
    }

    running = true;
    ON = true;
    debugLog("Holder auto focus started");
    bool ok = focusHolder();
    running = false;
    debugLog(ok ? "Holder auto focus finished" : "Holder auto focus failed");
    emit finish(ok ? "Holder auto focus finished" : "Holder auto focus failed");
}

void AutoFocus::stop()
{
    debugLog("Stop request received", false);
    ON = false;
}

bool AutoFocus::ensureSocketConnected()
{
    if(!vision)
    {
        debugLog("Socket check failed: vision is null");
        return false;
    }

    bool connected = false;
    QMetaObject::invokeMethod(vision, "isConnected",
                              Qt::BlockingQueuedConnection,
                              Q_RETURN_ARG(bool, connected));
    debugLog(QString("Socket connected: %1").arg(connected ? "true" : "false"), false);
    return connected;
}

bool AutoFocus::saveCurrentImage()
{
    if(CurrentBGR.empty())
    {
        debugLog("Save image failed: CurrentBGR is empty");
        return false;
    }

    QString folder = "F:/.romiol/";
    if(!QDir().exists(folder))
        QDir().mkpath(folder);

    bool ok = imwrite((folder + "img.jpg").toStdString(), CurrentBGR);
    debugLog(QString("Save image %1: %2img.jpg")
             .arg(ok ? "ok" : "failed")
             .arg(folder), false);
    return ok;
}

QString AutoFocus::requestServer(const QString &command, int timeoutMs)
{
    QString response;
    if(!vision)
    {
        debugLog(QString("Request skipped: vision is null, command=%1").arg(command));
        return response;
    }

    QMetaObject::invokeMethod(vision, "request",
                              Qt::BlockingQueuedConnection,
                              Q_RETURN_ARG(QString, response),
                              Q_ARG(QString, command),
                              Q_ARG(int, timeoutMs));
    response = response.trimmed();
    return response;
}

AutoFocus::FocusResult AutoFocus::requestFocus(const QString &command)
{
    FocusResult result;
    result.cls = "2";
    result.clarity = 0.0;
    result.ok = false;

    if(!saveCurrentImage())
        return result;

    QString response = requestServer(command);
    QStringList parts = response.split(":");
    if(parts.size() != 2)
    {
        debugLog(QString("Focus response format error: command=%1 response=%2").arg(command, response));
        return result;
    }

    bool clarityOk = false;
    result.cls = parts[0].trimmed();
    result.clarity = parts[1].trimmed().toDouble(&clarityOk);
    result.ok = clarityOk;
    if(result.ok)
    {
        debugLog(QString("Focus result: command=%1 class=%2 clarity=%3")
                 .arg(command)
                 .arg(result.cls)
                 .arg(result.clarity, 0, 'f', 6), false);
    }
    else
    {
        debugLog(QString("Focus clarity parse failed: command=%1 response=%2").arg(command, response));
    }
    return result;
}

bool AutoFocus::moveTargetZ(Target target, int distance, int *currentDistance)
{
    if(!ON)
    {
        debugLog("Auto focus stopped before moveZ");
        return false;
    }

    if(distance == 0)
    {
        debugLog(QString("Move skipped: target=%1 distance=0").arg(targetName(target)), false);
        return true;
    }

    int nextDistance = currentDistance ? *currentDistance + distance : distance;
    if(currentDistance)
    {
        if(qAbs(nextDistance) > maxTravel)
        {
            debugLog(QString("Stop: safety travel limit exceeded, target=%1 nextTotalZ=%2 maxTravel=%3")
                     .arg(targetName(target))
                     .arg(nextDistance)
                     .arg(maxTravel));
            ON = false;
            return false;
        }
        *currentDistance = nextDistance;
    }


    if(target == InjectorTarget)
        emit moveInjZ(distance, moveSpeed, false);
    else
        emit moveHoldZ(distance, moveSpeed, false);

    QThread::msleep(1200);
    if(!ON)
        debugLog("Auto focus stopped after moveZ wait");
    return ON;
}

bool AutoFocus::focusInjector()
{
    if(!ensureSocketConnected())
    {
        debugLog("Stop: socket is not connected");
        return false;
    }

    requestServer("reset_focus");

    int currentDistance = 0;
    FocusResult initial = requestFocus("focus");
    debugLog(QString("Injector initial: ok=%1 class=%2 clarity=%3")
             .arg(initial.ok ? "true" : "false")
             .arg(initial.cls)
             .arg(initial.clarity, 0, 'f', 6), false);

    if(!ON)
    {
        debugLog("Injector focus exit: stop requested");
        return false;
    }
    if(!initial.ok || initial.cls == "2")
    {
        debugLog("Injector focus exit: no initial injector detected, no move issued");
        return false;
    }

    int bestDistance = 0;
    double bestClarity = 0.0;
    QString previousClass = initial.cls;
    QString targetClass;
    int direction = 1;

    if(initial.cls == "1")
    {
        targetClass = "-1";
        direction = 1;
    }
    else if(initial.cls == "-1")
    {
        targetClass = "1";
        direction = -1;
    }
    else
    {
        int confirm = 0;
        for(int i = 0; ON && i < maxIterations; ++i)
        {
            debugLog(QString("Injector seek boundary: iteration=%1 confirm=%2").arg(i + 1).arg(confirm), false);
            if(!moveTargetZ(InjectorTarget, -injectorFocusedStep, &currentDistance))
                return false;
            FocusResult r = requestFocus("focus");
            if(!r.ok)
                return false;
            debugLog(QString("Injector seek boundary result: iteration=%1 class=%2 clarity=%3")
                     .arg(i + 1)
                     .arg(r.cls)
                     .arg(r.clarity, 0, 'f', 6), false);
            if(r.cls == "1")
                confirm++;
            else
                confirm = 0;
            if(confirm >= requiredConfirm)
                break;
        }
        targetClass = "-1";
        direction = 1;
        previousClass = "1";
    }

    debugLog(QString("Injector scan start: previousClass=%1 targetClass=%2 direction=%3")
             .arg(previousClass)
             .arg(targetClass)
             .arg(direction), false);

    int confirm = 0;
    for(int i = 0; ON && i < maxIterations; ++i)
    {
        int step = (previousClass == "0") ? injectorFocusedStep : injectorUnfocusStep;
        debugLog(QString("Injector scan move: iteration=%1 step=%2 confirm=%3")
                 .arg(i + 1)
                 .arg(direction * step)
                 .arg(confirm), false);
        if(!moveTargetZ(InjectorTarget, direction * step, &currentDistance))
            return false;

        FocusResult r = requestFocus("focus");
        if(!r.ok)
            return false;

        if(r.cls == "0" && r.clarity > bestClarity)
        {
            bestClarity = r.clarity;
            bestDistance = currentDistance;
            debugLog(QString("Injector best updated: bestDistance=%1 bestClarity=%2")
                     .arg(bestDistance)
                     .arg(bestClarity, 0, 'f', 6), false);
        }

        if(r.cls == targetClass)
            confirm++;
        else
            confirm = 0;

        debugLog(QString("Injector scan result: iteration=%1 class=%2 clarity=%3 totalZ=%4 bestZ=%5 confirm=%6")
                 .arg(i + 1)
                 .arg(r.cls)
                 .arg(r.clarity, 0, 'f', 6)
                 .arg(currentDistance)
                 .arg(bestDistance)
                 .arg(confirm), false);

        if(confirm >= requiredConfirm)
            break;

        previousClass = r.cls;
    }

    if(!ON)
    {
        debugLog("Injector focus exit: stop requested");
        return false;
    }
    if(bestClarity <= 0.0)
    {
        debugLog("Injector focus exit: no valid best clarity");
        return false;
    }

    debugLog(QString("Injector return to best: currentZ=%1 bestZ=%2 delta=%3")
             .arg(currentDistance)
             .arg(bestDistance)
             .arg(bestDistance - currentDistance), false);
    return moveTargetZ(InjectorTarget, bestDistance - currentDistance, &currentDistance);
}

bool AutoFocus::focusHolder()
{
    if(!ensureSocketConnected())
    {
        debugLog("Stop: socket is not connected");
        return false;
    }

    requestServer("reset_focus");

    int currentDistance = 0;
    FocusResult initial = requestFocus("focus_holder");
    debugLog(QString("Holder initial: ok=%1 class=%2 clarity=%3")
             .arg(initial.ok ? "true" : "false")
             .arg(initial.cls)
             .arg(initial.clarity, 0, 'f', 6), false);

    if(!ON)
    {
        debugLog("Holder focus exit: stop requested");
        return false;
    }
    if(!initial.ok || initial.clarity <= 0.0)
    {
        debugLog(QString("Holder focus exit: invalid initial result, class=%1 clarity=%2. class=2 means no holder detected; class=0 with clarity<=0 means holder detected but clarity is zero. no move issued")
                 .arg(initial.cls)
                 .arg(initial.clarity, 0, 'f', 6));
        return false;
    }

    int bestDistance = 0;
    double bestClarity = initial.clarity;
    double previousClarity = initial.clarity;
    int dropCount = 0;

    for(int i = 0; ON && i < maxIterations; ++i)
    {
        debugLog(QString("Holder scan move: iteration=%1 step=%2 dropCount=%3")
                 .arg(i + 1)
                 .arg(holderStep)
                 .arg(dropCount), false);
        if(!moveTargetZ(HolderTarget, holderStep, &currentDistance))
            return false;

        FocusResult r = requestFocus("focus_holder");
        if(!r.ok || r.clarity <= 0.0)
        {
            debugLog(QString("Holder focus exit: invalid scan result iteration=%1 ok=%2 clarity=%3")
                     .arg(i + 1)
                     .arg(r.ok ? "true" : "false")
                     .arg(r.clarity, 0, 'f', 6));
            return false;
        }

        if(r.clarity > bestClarity)
        {
            bestClarity = r.clarity;
            bestDistance = currentDistance;
            debugLog(QString("Holder best updated: bestDistance=%1 bestClarity=%2")
                     .arg(bestDistance)
                     .arg(bestClarity, 0, 'f', 6), false);
        }

        if(r.clarity < previousClarity)
            dropCount++;
        else
            dropCount = 0;

        previousClarity = r.clarity;
        debugLog(QString("Holder scan result: iteration=%1 class=%2 clarity=%3 totalZ=%4 bestZ=%5 bestClarity=%6 dropCount=%7")
                 .arg(i + 1)
                 .arg(r.cls)
                 .arg(r.clarity, 0, 'f', 6)
                 .arg(currentDistance)
                 .arg(bestDistance)
                 .arg(bestClarity, 0, 'f', 6)
                 .arg(dropCount), false);
        if(dropCount >= holderDropLimit)
            break;
    }

    if(!ON)
    {
        debugLog("Holder focus exit: stop requested");
        return false;
    }
    if(bestClarity <= 0.0)
    {
        debugLog("Holder focus exit: no valid best clarity");
        return false;
    }

    debugLog(QString("Holder return to best: currentZ=%1 bestZ=%2 delta=%3")
             .arg(currentDistance)
             .arg(bestDistance)
             .arg(bestDistance - currentDistance), false);
    return moveTargetZ(HolderTarget, bestDistance - currentDistance, &currentDistance);
}
