#ifndef MVVIDEOCAPTURE_H
#define MVVIDEOCAPTURE_H

#include "Settings/Settings.h"

#if (USE_VIDEO == 0)
#include "CameraApi.h"

#include <thread>
#include <mutex>
#include <opencv2/opencv.hpp>

class MVVideoCapture
{
public:
    static int    Init(int id = 0);
    static int    Play();
    static int    Read();
    static int    GetFrame(cv::Mat& frame);
    static int    Stop();
    static int    Uninit();
    static int    SetExposureTime(bool auto_exp, double exp_time = 10000);
    static double GetExposureTime();
    static int    SetLargeResolution(bool if_large_resolution);
    static cv::Size   GetResolution();
    static int    SetGain(double gain);
    static double GetGain();
    static int    SetWBMode(bool auto_wb = true);
    static int    GetWBMode(bool & auto_wb);
    static int    SetOnceWB();

    static int  iCameraCounts;
    static int  iStatus;
    static int  hCamera;
    static int  channel;
    static tSdkCameraCapbility     tCapability;      //设备描述信息
    static tSdkFrameHead           sFrameInfo;
    static unsigned char           *	pbyBuffer;
    static unsigned char           * g_pRgbBuffer[2];     //处理后数据缓存区
    static int                     ready_buffer_id;
    static bool                    stopped;
    static bool                    updated;
    static std::mutex              mutex1;

private:
    MVVideoCapture();
};
#endif  // USE_VIDEO

#endif // MVVIDEOCAPTURE_H
