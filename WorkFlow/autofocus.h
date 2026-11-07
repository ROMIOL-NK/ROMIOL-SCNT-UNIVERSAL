#ifndef AUTOFOCUS_H
#define AUTOFOCUS_H

#include "public.h"

class AutoFocus : public QObject
{
    Q_OBJECT
public:
    explicit AutoFocus(QObject *parent = nullptr);

public slots:
    void runInjector();
    void runHolder();
    void stop();

signals:
    void moveInjZ(float, float, bool);
    void moveHoldZ(float, float, bool);
    void status(QString);
    void finish(QString);

private:
    enum Target {
        InjectorTarget,
        HolderTarget
    };

    struct FocusResult {
        QString cls;
        double clarity;
        bool ok;
    };

    bool ON;
    bool running;
    int moveSpeed;
    int maxIterations;
    int maxTravel;
    int requiredConfirm;
    int injectorFocusedStep;
    int injectorUnfocusStep;
    int holderStep;
    int holderDropLimit;

    void debugLog(const QString &message, bool updateStatus = true);
    QString targetName(Target target) const;
    bool ensureSocketConnected();
    bool saveCurrentImage();
    QString requestServer(const QString &command, int timeoutMs = 5000);
    FocusResult requestFocus(const QString &command);
    bool moveTargetZ(Target target, int distance, int *currentDistance = nullptr);
    bool focusInjector();
    bool focusHolder();
};

#endif // AUTOFOCUS_H
