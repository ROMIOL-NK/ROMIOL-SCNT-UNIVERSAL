QT       += core gui serialport charts
QT       += axcontainer
QT       += network
QT       += multimediawidgets
QT       += multimedia
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

QT_CONFIG -= qtquickcompiler
CONFIG += c++11
CONFIG += debug_and_release
CONFIG += resources_big

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    Components/holder.cpp \
    Components/injector.cpp \
    Components/microscope.cpp \
    Controllers/controller.cpp \
    Controllers/controller_adrc.cpp \
    Controllers/controller_cgpc.cpp \
    Controllers/controller_foasmc.cpp \
    Controllers/controller_foasmc_rbf.cpp \
    Controllers/controller_fuzzy_foasmc_rbf_pi.cpp \
    Controllers/controller_gpc.cpp \
    Controllers/controller_hinfsmcfoi.cpp \
    Controllers/controller_mrfoasmc_rbf.cpp \
    Controllers/controller_pid.cpp \
    Controllers/controller_pneumatic.cpp \
    Controllers/controller_test.cpp \
    Controllers/controller_zdt.cpp \
    Devices/cameratoup.cpp \
    Devices/camerausb.cpp \
    Devices/focusdevicenikon.cpp \
    Devices/focusdeviceprior.cpp \
    Devices/manipulatormp285.cpp \
    Devices/manipulatorscientifica.cpp \
    Devices/manipulatorscientificamotion8.cpp \
    Devices/psensormik_p300.cpp \
    Devices/psensormik_p3000.cpp \
    Devices/stageprior.cpp \
    Devices/stagetrio.cpp \
    Devices/stagezmotion.cpp \
    Devices/syringemotorgt.cpp \
    Devices/syringemotorvince.cpp \
    Objects/globalmap.cpp \
    Objects/holdpipette.cpp \
    Objects/injectpipette.cpp \
    Objects/oocyte.cpp \
    Objects/oocytes.cpp \
    Objects/soma.cpp \
    Objects/somas.cpp \
    WorkFlow/autoclone.cpp \
    WorkFlow/autofocus.cpp \
    WorkFlow/functions.cpp \
    WorkFlow/pservo.cpp \
    WorkFlow/stepbuildmap.cpp \
    WorkFlow/stepenucleation.cpp \
    WorkFlow/stepexception.cpp \
    WorkFlow/stepexpel.cpp \
    WorkFlow/stepfetchoocyte.cpp \
    WorkFlow/stepfetchoocyteforsoma.cpp \
    WorkFlow/stepfetchsoma.cpp \
    WorkFlow/stepinjectsoma.cpp \
    WorkFlow/steppenetrate.cpp \
    WorkFlow/steppenetrateforsoma.cpp \
    WorkFlow/stepreleaseoocyte.cpp \
    WorkFlow/steprotateforsoma.cpp \
    WorkFlow/steprotateinplane.cpp \
    WorkFlow/steprotateoutplane.cpp \
    WorkFlow/track.cpp \
    WorkFlow/vision.cpp \
    WorkFlow/workflow.cpp \
    childwindows.cpp \
    main.cpp \
    public.cpp \
    romiolscnt.cpp \
    utils/custom_algorithms.cpp \
    utils/neuralnetwork.cpp \
    utils/objectdetector.cpp \
    utils/triopc.cpp

HEADERS += \
    Components/components.h \
    Components/holder.h \
    Components/injector.h \
    Components/microscope.h \
    Controllers/controller.h \
    Controllers/controller_adrc.h \
    Controllers/controller_cgpc.h \
    Controllers/controller_foasmc.h \
    Controllers/controller_foasmc_rbf.h \
    Controllers/controller_fuzzy_foasmc_rbf_pi.h \
    Controllers/controller_gpc.h \
    Controllers/controller_hinfsmcfoi.h \
    Controllers/controller_mrfoasmc_rbf.h \
    Controllers/controller_pid.h \
    Controllers/controller_pneumatic.h \
    Controllers/controller_test.h \
    Controllers/controller_zdt.h \
    Controllers/controllers.h \
    Devices/Base/camera.h \
    Devices/Base/focusdevice.h \
    Devices/Base/manipulator.h \
    Devices/Base/psensor.h \
    Devices/Base/stage.h \
    Devices/Base/syringemotor.h \
    Devices/cameratoup.h \
    Devices/camerausb.h \
    Devices/focusdevicenikon.h \
    Devices/focusdeviceprior.h \
    Devices/manipulatormp285.h \
    Devices/manipulatorscientifica.h \
    Devices/manipulatorscientificamotion8.h \
    Devices/psensormik_p300.h \
    Devices/psensormik_p3000.h \
    Devices/stageprior.h \
    Devices/stagetrio.h \
    Devices/stagezmotion.h \
    Devices/syringemotorgt.h \
    Devices/syringemotorvince.h \
    Objects/globalmap.h \
    Objects/holdpipette.h \
    Objects/injectpipette.h \
    Objects/objects.h \
    Objects/oocyte.h \
    Objects/oocytes.h \
    Objects/soma.h \
    Objects/somas.h \
    WorkFlow/autoclone.h \
    WorkFlow/autofocus.h \
    WorkFlow/functions.h \
    WorkFlow/pservo.h \
    WorkFlow/stepbuildmap.h \
    WorkFlow/stepenucleation.h \
    WorkFlow/stepexception.h \
    WorkFlow/stepexpel.h \
    WorkFlow/stepfetchoocyte.h \
    WorkFlow/stepfetchoocyteforsoma.h \
    WorkFlow/stepfetchsoma.h \
    WorkFlow/stepinjectsoma.h \
    WorkFlow/steppenetrate.h \
    WorkFlow/steppenetrateforsoma.h \
    WorkFlow/stepreleaseoocyte.h \
    WorkFlow/steprotateforsoma.h \
    WorkFlow/steprotateinplane.h \
    WorkFlow/steprotateoutplane.h \
    WorkFlow/track.h \
    WorkFlow/vision.h \
    WorkFlow/workflow.h \
    childwindows.h \
    public.h \
    romiolscnt.h \
    utils/custom_algorithms.h \
    utils/gts.h \
    utils/neuralnetwork.h \
    utils/objectdetector.h \
    utils/toupcam.h \
    utils/triopc.h \
    utils/zauxdll.h \
    utils/zmotion.h

FORMS += \
    humanassistance.ui \
    romiolscnt.ui

INCLUDEPATH += $$PWD

#第三方库
#OSQP
LIBS += $$PWD/LIB/libosqp.dll.a
#Clips
INCLUDEPATH += $$PWD/3rdparty/clips
LIBS += -L$$PWD/LIB/ -lClips# Clips
#LIBS += $$PWD/LIB/Clips.dll# Clips
#设备链接库
LIBS += -L$$PWD/LIB/ -lgts      # GoogolTech电机
LIBS += -L$$PWD/LIB/ -ltoupcam  # 图谱相机
LIBS += -L$$PWD/LIB/ -lzauxdll
LIBS += -L$$PWD/LIB/ -lzmotion

# #------------------------------------------深圳配置 IPC start------------------------------------------
# DEFINES += SYSTEM_SZ_IPC
# CONFIG(debug,debug|release)\
# {#debug 模式
#     # opencv
#     INCLUDEPATH += D:\OpenCV\opencv-4.8.0\build\include
#     LIBS += -LD:\OpenCV\opencv-4.8.0\build\x64\vc16\lib/ -lopencv_world480d
# }

# CONFIG(release,debug|release)\
# {#release 模式
#     # opnecv cpu版
#     INCLUDEPATH += D:\OpenCV\opencv-4.8.0\build\include
#     LIBS += -LD:\OpenCV\opencv-4.8.0\build\x64\vc16\lib/ -lopencv_world480
# }
# #------------------------------------------深圳配置 IPC end------------------------------------------

#------------------------------------------深圳配置 Station start------------------------------------------
# DEFINES += SYSTEM_SZ_STATION
# CONFIG(debug,debug|release)\
# {#debug 模式
#     # opencv
#     INCLUDEPATH += D:\OpenCV\opencv-4.8.0\build\include
#     LIBS += -LD:\OpenCV\opencv-4.8.0\build\x64\vc16\lib/ -lopencv_world480d
# }

# CONFIG(release,debug|release)\
# {#release 模式
#     exists(C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v11.1\bin\nppial64_11.dll){# 对应深圳424拥有GPU的工控机
#     # opencv 本地编译的 gpu版
#     INCLUDEPATH += D:\OpenCV\opencv-4.7.0-gpu\build\install\include
#     LIBS += -LD:\OpenCV\opencv-4.7.0-gpu\build\lib\Release -lopencv_world470}
#     else{# 对应其他的平台
#     # opnecv cpu版
#     INCLUDEPATH += D:\OpenCV\opencv-4.8.0\build\include
#     LIBS += -LD:\OpenCV\opencv-4.8.0\build\x64\vc16\lib/ -lopencv_world480}
# }
#------------------------------------------深圳配置 Station end------------------------------------------


#------------------------------------------天津配置 start------------------------------------------
DEFINES += SYSTEM_TJ
CONFIG(debug,debug|release)\
{
    INCLUDEPATH += D:/Software/OpenCV/opencv/build/include
    LIBS += -LD:/Software/OpenCV/opencv/build/x64/vc16/lib -lopencv_world480d
}
CONFIG(release,debug|release)\
{
    INCLUDEPATH += D:/Software/OpenCV/opencv/build/include
    LIBS += -LD:/Software/OpenCV/opencv/build/x64/vc16/lib -lopencv_world480
}
#------------------------------------------天津配置 end------------------------------------------
# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

RESOURCES += \
    Resources/resources.qrc

DISTFILES +=
