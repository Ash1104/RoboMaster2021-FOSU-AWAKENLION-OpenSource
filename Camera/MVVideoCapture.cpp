#include "Camera/MVVideoCapture.h"

#if (USE_VIDEO==0)
#include <CameraDefine.h>
#include <zconf.h>
//#include "include/Common.h"

int                     MVVideoCapture::iCameraCounts = 4;
int                     MVVideoCapture::iStatus = 0;
int                     MVVideoCapture::hCamera = 0;
int                     MVVideoCapture::channel = 3;

//tSdkCameraDevInfo       MVVideoCapture::tCameraEnumList[4];
tSdkCameraCapbility     MVVideoCapture::tCapability;      //设备描述信息
tSdkFrameHead           MVVideoCapture::sFrameInfo;
unsigned char           *MVVideoCapture::pbyBuffer = NULL;
unsigned char           *MVVideoCapture::g_pRgbBuffer[2] = {NULL, NULL};     //处理后数据缓存区
int                     MVVideoCapture::ready_buffer_id = 0;
bool                    MVVideoCapture::stopped = false;
bool                    MVVideoCapture::updated = false;
std::mutex              MVVideoCapture::mutex1;

MVVideoCapture::MVVideoCapture()
{
}

int MVVideoCapture::Init(int id)
{
    printf("相机SDK初始化中...\n");
    CameraSdkInit(1);
    printf("相机SDK初始化完成\n");

    printf("枚举设备，并建立设备列表中...\n");
    tSdkCameraDevInfo tCameraEnumList[4];
    CameraEnumerateDevice(tCameraEnumList,&iCameraCounts);
    if(iCameraCounts==0)
    {
        perror("Error: 没有连接设备\n");
        return -1;
    } else if (iCameraCounts <= id) {
        printf("部分相机未连接，已连接相机数: %d, ID: %d.\n", iCameraCounts, id);
        return -1;
    } else {
        printf("已连接相机数: %d.\n", iCameraCounts);
    }
    printf("枚举设备，并建立设备列表完成\n");

    //相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
    iStatus = CameraInit(&(tCameraEnumList[id]),-1,-1,&hCamera);
    //初始化失败
    if (iStatus!=CAMERA_STATUS_SUCCESS)
    {
        perror("Error:相机初始化失败\n");
        return -1;
    } else {
        printf("相机初始化成功\n");
    }

    //设置色温模式
//    iStatus = CameraSetPresetClrTemp(hCamera, 1);
//    if (iStatus == CAMERA_STATUS_SUCCESS) {
//        printf("选择指定预设色温模式成功\n");
//    } else {
//        printf("选择指定预设色温模式错误码：%d\n", iStatus);
//    }

//    iStatus = CameraSetClrTempMode(hCamera, 1);
//    if (iStatus == CAMERA_STATUS_SUCCESS) {
//        printf("设置白平衡时使用的色温模式成功\n");
//    } else {
//        printf("设置白平衡时使用的色温模式错误码: %d\n", iStatus);
//    }

    //获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
    CameraGetCapability(hCamera,&tCapability);

    //设置输出为彩色
    channel = 3;
    CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_BGR8);

    //初始化缓冲区
    g_pRgbBuffer[0] = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);
    g_pRgbBuffer[1] = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);
}

int MVVideoCapture::Uninit()
{
    printf("保存相机参数...\n");
    CameraSaveParameter(hCamera, 0);

    printf("相机反初始化。释放资源...\n");
    int status = CameraUnInit(hCamera);

    printf("status: %d\n", status);

    if (status == CAMERA_STATUS_SUCCESS) {
        printf("相机反初始化成功\n");
    } else {
        printf("相机反初始化错误码: %d\n", status);
    }

    if (g_pRgbBuffer[0] != NULL) {
        free(g_pRgbBuffer[0]);
        g_pRgbBuffer[0] = NULL;
    }

    if (g_pRgbBuffer[1] != NULL) {
        free(g_pRgbBuffer[1]);
        g_pRgbBuffer[1] = NULL;
    }
}

int MVVideoCapture::SetExposureTime(bool auto_exp, double exp_time)
{
    if (auto_exp) {
        CameraSdkStatus status = CameraSetAeState(hCamera, true);
        if (status == CAMERA_STATUS_SUCCESS) {
            printf("自动曝光设置成功\n");
        } else {
            perror("自动曝光设置失败\n");
            return status;
        }
    } else {
        CameraSdkStatus status = CameraSetAeState(hCamera, false);
        if (status == CAMERA_STATUS_SUCCESS) {
            printf("手动曝光设置成功\n");
        } else {
            perror("手动曝光设置失败\n");
            return status;
        }
        CameraSdkStatus status1 = CameraSetExposureTime(hCamera, exp_time);
        if (status1 == CAMERA_STATUS_SUCCESS) {
            printf("设置曝光时间成功\n");
        } else {
            perror("设置曝光时间失败\n");
            return status;
        }
    }

    return 0;
}

double MVVideoCapture::GetExposureTime()
{
    int auto_exp;
    if (CameraGetAeState(hCamera, &auto_exp) == CAMERA_STATUS_SUCCESS) {
        if (auto_exp) {
            return 0;
        } else {
            double exp_time;
            if (CameraGetExposureTime(hCamera, &exp_time) == CAMERA_STATUS_SUCCESS) {
                return exp_time;
            } else {
                printf("获得相机的曝光时间失败\n");
                return -1;
            }
        }
    } else {
        printf("获得相机当前的曝光模式失败\n");
        return -1;
    }
}

int MVVideoCapture::SetLargeResolution(bool if_large_resolution)
{
    tSdkImageResolution resolution;
    if (if_large_resolution) {
        resolution.iIndex = 0;
        if (CameraSetImageResolution(hCamera, &resolution) == CAMERA_STATUS_SUCCESS) {
            printf("大分辨率，设置预览的分辨率成功\n");
        } else {
            printf("大分辨率，设置预览的分辨率失败\n");
            return -1;
        }
    } else {
        resolution.iIndex = 1;
        CameraSetImageResolution(hCamera, &resolution);
        if (CameraSetImageResolution(hCamera, &resolution) == CAMERA_STATUS_SUCCESS) {
            printf("小分辨率，设置预览的分辨率成功\n");
        } else {
            printf("小分辨率，设置预览的分辨率失败\n");
            return -1;
        }
    }

  return 0;
}

int MVVideoCapture::SetWBMode(bool auto_wb)
{
    int status = CameraSetWbMode(hCamera, auto_wb);
    if (CAMERA_STATUS_SUCCESS == status) {
        printf("设置相机白平衡模式：%d 成功\n", auto_wb);
    } else {
        printf("设置相机白平衡模式：%d 失败! 错误码: %d\n", auto_wb, status);
    }
}

int MVVideoCapture::GetWBMode(bool &auto_wb)
{
    int res = 0;
    if (CAMERA_STATUS_SUCCESS == CameraGetWbMode(hCamera, &res)) {
        printf("获得当前的白平衡模式： %d 成功\n", res);
    } else {
        printf("获得当前的白平衡失败\n");
    }
    auto_wb = res;
}

int MVVideoCapture::SetOnceWB()
{
  int status = CameraSetOnceWB(hCamera);
  if (CAMERA_STATUS_SUCCESS == status) {
    printf("进行一次白平衡成功\n");
  } else {
    printf("进行一次白平衡错误码: %d!\n", status);
  }
}

int MVVideoCapture::SetGain(double gain)
{
    int set_gain = int(gain*100);
    int status = CameraSetGain(hCamera, set_gain, set_gain, set_gain);
    if (CAMERA_STATUS_SUCCESS == status) {
        printf("设置图像的数字增益成功\n");
    } else {
        printf("设置图像的数字增益错误码: %d\n", status);
    }
}

double MVVideoCapture::GetGain()
{
    int r_gain, g_gain, b_gain;
    int status = CameraGetGain(hCamera, &r_gain, &g_gain, &b_gain);
    if (CAMERA_STATUS_SUCCESS == status) {
        printf("获得图像处理的数字增益成功\n");
    } else {
        printf("获得图像处理的数字增益错误码: %d\n", status);
    }

    return (r_gain + g_gain + b_gain)/300.;
}

int MVVideoCapture::Play()
{
    CameraPlay(hCamera);
    std::thread thread1(MVVideoCapture::Read);
    thread1.detach();
}

int MVVideoCapture::Read()
{
    while (!stopped) {
        if (CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 1000) == CAMERA_STATUS_SUCCESS) {
            CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer[(ready_buffer_id+1)%2], &sFrameInfo);
            ready_buffer_id = (ready_buffer_id + 1)%2;
            updated = true;
            CameraReleaseImageBuffer(hCamera, pbyBuffer);
        }
    }
}

int MVVideoCapture::GetFrame(cv::Mat &frame)
{
    if (frame.cols != sFrameInfo.iWidth || frame.rows != sFrameInfo.iHeight)
    {
//        printf("GetFrame: resize frame !\n");
        frame.create(sFrameInfo.iHeight, sFrameInfo.iWidth, CV_8UC3);
    }
//    TIME_BEGIN();
    while (!updated)
    {
        usleep(1000);
    }
//    TIME_END("Wait for camera data");
    memcpy(frame.data, g_pRgbBuffer[ready_buffer_id], frame.cols*frame.rows*3);
    updated = false;
    return 0;
}

int MVVideoCapture::Stop()
{
    stopped = true;
    usleep(30000);
    return 0;
}
#endif
