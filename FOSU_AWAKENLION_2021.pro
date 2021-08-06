QT += core
QT -= gui

CONFIG += c++11

TARGET = FOSU_AWAKENLION_2021
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

#INCLUDEPATH += -I/usr/local/include \
#               -I/usr/local/include/opencv \
#               -I/usr/local/include/opencv2 \

#LIBS += -L/usr/local/lib -lopencv_ml -lopencv_dnn -lopencv_objdetect -lopencv_stitching -lopencv_superres -lopencv_videostab -lopencv_photo -lopencv_shape -lopencv_video -lopencv_calib3d -lopencv_features2d -lopencv_highgui -lopencv_videoio -lopencv_imgcodecs -lopencv_imgproc -lopencv_flann -lopencv_core

QMAKE_CFLAGS  =`pkg-config --cflags opencv`
LIBS += `pkg-config --libs opencv` -lMVSDK
INCLUDEPATH = -I./include

HEADERS += \
    ArmorDetector/TargetDetection.h \
    Settings/Settings.h \
    Serial/Serial.h \
    RuneDetector/RuneDetector.h \
    Serial/Protocol.h \
    Preprocessing/Preprocessing.h \
    AngleSolver/PnpSolver.h \
    Serial/PackData.h \
    Serial/JudgementInfo.h \
    Serial/InfantryInfo.h \
    Gui/Gui.h \
    ArmorDetector/ArmorDetector.h \
    Camera/CameraApi.h \
    Camera/MVVideoCapture.h \
    AngleSolver/GravityCompensateResolve.h \
    AngleSolver/bayesEstimateVelocity.h

SOURCES += \
    ArmorDetector/TargetDetection.cpp \
    Settings/Settings.cpp \
    Serial/Serial.cpp \
    RuneDetector/RuneDetector.cpp \
    Serial/Protocol.cpp \
    Preprocessing/Preprocessing.cpp \
    AngleSolver/PnpSolver.cpp \
    Serial/PackData.cpp \
    Serial/InfantryInfo.cpp \
    main.cpp \
    Camera/MVVideoCapture.cpp \
    Gui/Gui.cpp \
    ArmorDetector/ArmorDetector.cpp \
    AngleSolver/GravityCompensateResolve.cpp \
    AngleSolver/bayesEstimateVelocity.cpp

DISTFILES += \
    config_file/param_rune.yml \
    config_file/param_other.yml \
    config_file/param_armor.yml \
    config_file/param_throw_compensate.yml \
    calibration/Camera640.xml \
    config_file/param_pid-sentry7.yml \
    config_file/param_pid-hero1.yml \
    config_file/param_pid-fantry5.yml \
    config_file/param_pid-fantry3.yml \
    config_file/right.jpg \
    config_file/left.jpg \
    src/README.md \
    calibration/default.xml \
    config_file/param_kalman.yml \
    calibration/Camera752-hero1.xml \
    config_file/param_kalman.yml

