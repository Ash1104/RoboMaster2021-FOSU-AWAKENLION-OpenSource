#ifndef SETTINGS_H
#define SETTINGS_H

#include <mutex>
#include <sys/time.h>

#include "opencv2/opencv.hpp"

#define DEBUG_MODE       // 模式选择
//#define USE_CUTECOM     // 是否使用串口助手
//#define DEBUG_CAMERA    // 工业相机位置调试
#define USE_SERIAL       // 使用串口
#define USE_VIDEO 0      // 0迈德威视 1摄像头/视频
//#define USE_WIN
#define SHOW_WIDTH 752
#define SHOW_HEIGHT 480
#define SHOW_RADIO 1     //显示比例
#define DATA_BUFF_ROW_SIZE_MAX 5
#define DATA_BUFF_COL_SIZE_MAX 1200

//-----------------------------------【宏定义-调试模块】--------------------------------------------
// brief：主要用于定义一些窗口
//-----------------------------------------------------------------------------------------------
#define VIDEO_PATH "/home/rm/视频/3.mp4"                   // 测试视频路径
#define WIN_NAME_ARMORP_PREPROCESS "bin"                   // 灯条预处理窗口
#define WIN_NAME_ARMORP_TARGET "灯条调试"                  // 灯条调试窗口
#define WIN_NAME_ARMORP "小甲板调试"                       // 装甲板调试窗口
#define WIN_NAME_RUNEP_PREPROCESS "神符preprocess调试"
#define WIN_NAME_RUNEP "debug-rune-cur-param"              // 调试窗口命名
#define WIN_NAME_RUNEP_R "debug-rune-center-param"         // 调试窗口命名
#define WIN_NAME_RUNE_COMPENSATE "debug-rune-compensate"
#define WIN_NAME_RUNE_TIME "debug-rune-time"

//-----------------------------------【宏定义-键盘指令】--------------------------------------------
// brief：
// 按v  开始、结束录视频
// 按p  截图
// 按s  保存参数
//------------------------------------------------------------------------------------------------
#define KEY_SAVE_VIDEO  76  //开始、结束录视频
#define KEY_SAVE_RESULT 70  //截图
#define KEY_SAVE_PARAM  115 //保存参数

#define ARMS 1          // 当前PC所属机器人编号
#if (ARMS == 1)
#define PARAM_OTHER_PATH "/home/rm/rm2021/FOSU_AWAKENLION_2021/FOSU_AWAKENLION_2021/config_file/param_other.yml"            // 全局配置文件路径
#define PARAM_ARMOR_PATH "/home/rm/rm2021/FOSU_AWAKENLION_2021/FOSU_AWAKENLION_2021/config_file/param_armor.yml"            // 装甲板配置文件路径
#define PARAM_CALIBRATION_752 "/home/rm/rm2021/FOSU_AWAKENLION_2021/FOSU_AWAKENLION_2021/calibration/Camera752-hero1.xml"   // 相机参数
#define PARAM_RUNE_PATH "/home/rm/rm2021/FOSU_AWAKENLION_2021/FOSU_AWAKENLION_2021/config_file/param_rune.yml"              // 神符配置文件路径
#define SAVE_VIDEO_DIR   "/home/rm/rm2021/FOSU_AWAKENLION_2021/FOSU_AWAKENLION_2021/save_video/"
#define SAVE_PIC_DIR   "/home/rm/rm2021/FOSU_AWAKENLION_2021/FOSU_AWAKENLION_2021/save_pic/"

#endif  // ARMS

//-----------------------------------【模式切换枚举】------------------------------------------
// brief：用于模式切换
//------------------------------------------------------------------------------------------
enum MainMode
{
    armor_mode, small_rune_mode, big_rune_mode, static_rune_mode
};

//-----------------------------------【装甲板颜色枚举】----------------------------------------
// brief：用于敌方装甲板颜色切换
//------------------------------------------------------------------------------------------
enum EnemyColor
{
    red, blue
};

//-----------------------------------【ROI模式切换枚举】---------------------------------------
// brief：用于ROI模式下与普通模式切换
//------------------------------------------------------------------------------------------
enum TrackMode
{
    normal_mode, roi_mode
};

//-----------------------------------【能量机关模式切换】---------------------------------------
// brief：能量机关模式切换
//------------------------------------------------------------------------------------------
enum state
{
    start_state, shoot_state, shift_state, stop_state
};

//-----------------------------------【配置-串口】--------------------------------------------
// brief：串口配置参数
//------------------------------------------------------------------------------------------
struct DeviceParam
{
    const char *dev_name;                 // 设备名称
    int baud_rate;                        // 波特率
    int databits;                         // 数据位
    int stopbits;                         // 停止位
    char parity;                          // 校验位
    DeviceParam()
    {
        dev_name = "/dev/ttyUSB0";         // 设备名称
        baud_rate = 115200;                // 波特率
        databits = 8;                      // 数据位
        stopbits = 1;                      // 停止位
        parity = 'n';                      // 校验位
    }
};

//-----------------------------------【能量机关补偿角度】--------------------------------------
// brief：
//------------------------------------------------------------------------------------------
struct RuneCompensate
{
    int is_ptz_raise;                      // 是否向上
    int pitch_offset;                      // 俯仰角补偿
    int is_ptz_right;                      // 是否向右
    int yaw_offset;                        // 偏航角补偿
    RuneCompensate()
    {
        is_ptz_raise = 0;                  // 是否向上
        pitch_offset = 23;                 // 俯仰角补偿
        is_ptz_right = 0;                  // 是否向右
        yaw_offset = 4;                    // 偏航角补偿
    }
};

//-----------------------------------【能量机关各种时间】--------------------------------------
// brief：
//------------------------------------------------------------------------------------------
struct RuneTimeParam
{
    int TRIG2SHOOT2HIT_DELAY;              // 下位机相应时间 + 发弹延时 + 空中时间
    int PTZ_SHIFT_DELAY;                   // 云台移动时间
    int SHOOT2HIT_DELAY;                   // 打击亮起时间，可以压缩很小
    RuneTimeParam()
    {
        TRIG2SHOOT2HIT_DELAY = 83;         // 下位机相应时间 + 发弹延时 + 空中时间
        PTZ_SHIFT_DELAY = 42;              // 云台移动时间
        SHOOT2HIT_DELAY = 52;              // 打击亮起时间，可以压缩很小
    }
};

//-----------------------------------【调式参数】---------------------------------------------
// brief：用于调试参数，创建滑动条
//------------------------------------------------------------------------------------------
struct Debug
{
    int b_show_src;                        // 原图
    int b_show_bin;                        // 二值图
    int b_show_target;                     // 目标-灯条
    int b_show_armor;                      // 装甲板
    int b_show_result;                     // 结果
    int b_show_fps;                        // 帧率
    int b_save_result;                     // 识别结果图
    int n_save_result;                     // 存储序号
    char n_key_order;                      // 键盘键位
    int b_save_pic;                        // 存储图像
    int f_save_pic_inter;                  // 存储图像参数
    int expore_time;                       // 曝光时间
    int b_show_dc;                         // 波形图
    int b_show_ex_armor;                   // 识别最终图像
    int b_show_param_rune;                 // 能量机关参数调试界面
    int b_show_bin_rune;                   // 能量机关二值化图像及调试界面
    int b_show_td_rune;                    // 能量机关目标装甲板调试界面
    int b_show_cp_rune;                    // 能量机关父轮廓筛选界面
    int b_show_center_rune;                // 能量机关中心筛选界面
    int b_show_cp_rune_result;             // 能量机关父轮廓筛选结果
    int b_show_center_rune_result;         // 能量机关中心点筛选结果
    int b_show_rune_center;                // 能量机关中心
    int b_show_compensate_rune;            // 能量机关补偿角度
    int b_show_rune_time_delay;            // 能量机关时间延迟
    int b_show_throw_compensate;           // 英雄吊射补偿角度
    int b_show_shoot;                      // 由算法控制是否发射

    Debug()
    {
        b_show_src = false;                // 原图
        b_show_bin = false;                // 二值图
        b_show_target = false;             // 目标-灯条
        b_show_armor = false;              // 装甲板
        b_show_result = false;             // 结果
        b_show_fps = false;                // 帧率
        b_save_result = false;             // 识别结果图
        n_save_result = 0;                 // 存储序号
        n_key_order = -1;                  // 键盘键位
        b_save_pic = false;                // 存储图像
        f_save_pic_inter = 1000;           // 存储图像参数
        expore_time = 7000;                // 曝光时间
        b_show_dc = false;                 // 波形图
        b_show_ex_armor = false;            // 识别最终图像
        b_show_param_rune = false;         // 能量机关参数调试界面
        b_show_bin_rune = false;           // 能量机关二值化图像及调试界面
        b_show_td_rune = false;            // 能量机关目标装甲板调试界面
        b_show_cp_rune = false;            // 能量机关父轮廓筛选界面
        b_show_center_rune = false;        // 能量机关中心筛选界面
        b_show_cp_rune_result = false;     // 能量机关父轮廓筛选结果
        b_show_center_rune_result = false; // 能量机关中心点筛选结果
        b_show_rune_center = false;        // 能量机关中心
        b_show_compensate_rune = false;    // 能量机关补偿角度
        b_show_rune_time_delay = false;    // 能量机关时间延迟
        b_show_throw_compensate = false;   // 英雄吊射补偿角度
        b_show_shoot = false;              // 由算法控制是否发射
    }
};

//-----------------------------------【摄像头参数】-------------------------------------------
// brief：摄像头参数初始化，便于角度解算
//------------------------------------------------------------------------------------------
struct CameraParam
{
    double x;                              // 工业相机关于云台中心x轴偏移量
    double y;                              // 工业相机关于云台中心y轴偏移量
    double z;                              // 工业相机关于云台中心z轴偏移量
    double y_offset;                       // y轴偏移量
    double z_offset_ptz2bul;               // z轴偏移量
    double overlap_dist;                   // 距离
    double bullet_speed;                   // 射速（能量机关用）
    CameraParam()
    {
        x = 0.;                            // 工业相机关于云台中心x轴偏移量
        y = 5.;                            // 工业相机关于云台中心y轴偏移量
        z = 10.;                           // 工业相机关于云台中心z轴偏移量
        y_offset = 0.;                     // y轴偏移量
        z_offset_ptz2bul = 10.0;           // z轴偏移量
        overlap_dist = 100000.;            // 距离
        bullet_speed = 26.;                // 射速（能量机关用）
    }
};

//-----------------------------------【参数-装甲板】--------------------------------------------
// brief：装甲板参数设置
//--------------------------------------------------------------------------------------------
struct ArmorParam {
    int delta_height_max;                  // 高度差
    int delta_angle_max;                   // 角度差
    int delta_w_ratio_min;                 // 宽比例min
    int delta_w_ratio_max;                 // 宽比例max
    int delta_h_max;                       // 高度差
    int armor_hw_ratio_min;                // 长宽比
    int armor_hw_ratio_max;                // 长宽比

    ArmorParam(){
        delta_height_max = 10;             // 高度差
        delta_angle_max = 45;              // 角度差
        delta_w_ratio_min = 10;            // 宽比例min
        delta_w_ratio_max = 90;            // 宽比例max
        delta_h_max = 20;                  // 高度差
        armor_hw_ratio_min = 15;           // 长宽比
        armor_hw_ratio_max = 30;           // 长宽比
    }
};

//-----------------------------------【参数-预处理】--------------------------------------------
// brief：
//-------------------------------------------------------------------------------------------
struct PreprocessParam
{
    int gray_thre_min;                     // 灰度图二值化最小阈值
    int gray_thre_max;                     // 灰度图二值化最大阈值
    int gray_max_w;                        // 自适应
    int gray_avg_w;                        // 自适应

    PreprocessParam()
    {
        gray_thre_min = 35;                // 灰度图二值化最小阈值
        gray_thre_max = 70;                // 灰度图二值化最大阈值
        gray_max_w = 6;                    // 自适应
        gray_avg_w = 4;                    // 自适应
    }
};

//--------------------------------------【目标尺寸参数】--------------------------------------
// brief：
//------------------------------------------------------------------------------------------
struct TargetSize
{
    int len;                               // 轮廓长度
    float ratio;                           // 长宽比
    int area;                              // 面积
    float area_ratio;                      // 面积比
    float area_len_ratio;                  // 面积比长度
    int corners;                           // 多边形逼近
    int slope_offset;                      // 角度偏移
};

//-----------------------------------【矩形四个点坐标】----------------------------------------
// brief：
//------------------------------------------------------------------------------------------
struct QuadrilateralPos
{
    cv::Point2f p[4];                      // 矩形四个角点坐标
};

//-----------------------------------【信息-灯条筛选】-------------------------------------------
// brief：
//---------------------------------------------------------------------------------------------
struct LightInfo
{
    int idx;                               // 索引
    std::vector<cv::Point>contours;        // 轮廓
    struct TargetSize size;                // 轮廓尺寸信息
    cv::RotatedRect rrect;                 // 灯条矩形
    cv::Rect rect;                         // 灯条矩形
};

//-----------------------------------【信息-装甲板检测信息】-------------------------------------
// brief：
//-------------------------------------------------------------------------------------------
struct ArmorInfo
{
    struct QuadrilateralPos pos;           // 装甲板四个顶点在屏幕投影坐标
    cv::RotatedRect rrect;                 // 装甲板矩形
    bool is_small;                         // 是否小装甲板
    std::vector<float> s_list;             // 筛选条件集合
};


//-----------------------------------【参数-目标尺寸】--------------------------------------------
// brief：
//---------------------------------------------------------------------------------------------
struct TargetSizeParam
{
    int len_min;                           // 轮廓长度最小值
    int len_max;                           // 轮廓长度最大值
    int ratio_min;                         // 长宽比最小值
    int ratio_max;                         // 长宽比最大值
    int area_min;                          // 最小面积
    int area_max;                          // 最大面积
    int area_ratio_min;                    // 矩形度最小值
    int area_ratio_max;                    // 矩形度最大值
    int area_len_ratio_min;                // 面积与长度比最小值
    int area_len_ratio_max;                // 面积与长度比最大值
    int corners_size_min;                  // 角点数目最小值
    int slope_offset;                      // 倾斜角差
    int color_th;                          // 识别颜色所在区域比例
    TargetSizeParam()
    {
        len_min = 50;                      // 轮廓长度最小值
        len_max = 10000;                   // 轮廓长度最大值
        ratio_min = 1;                     // 长宽比最小值
        ratio_max = 2;                     // 长宽比最大值
        area_min = 5000;                   // 最小面积
        area_max = 300000;                 // 最大面积
        area_ratio_min = 20;               // 矩形度最小值
        area_ratio_max = 100;              // 矩形度最大值
        area_len_ratio_min = 5;            // 面积与长度比最小值
        area_len_ratio_max = 10;           // 面积与长度比最大值
        corners_size_min = 7;              // 角点数目最小值
        slope_offset = 15;                 // 倾斜角差
        color_th = 60;                     // 识别颜色所在区域比例
    }
};

//--------------------------------------【预处理信息】----------------------------------------
// brief：
//------------------------------------------------------------------------------------------
struct PreprocessInfo
{
    int thre;                              // 阈值
};

//-----------------------------------【参数-神符】--------------------------------------------
// brief：
//------------------------------------------------------------------------------------------
struct TargetSizeParamRune
{
    int len_min;                           // 轮廓长度最小值
    int len_max;                           // 轮廓长度最大值
    int hw_ratio_min;                      // 长宽比最小值
    int hw_ratio_max;                      // 长宽比最大值
    int area_min;                          // 最小面积
    int area_max;                          // 最大面积
    int area_ratio_min;                    // 矩形度最小值
    int area_ratio_max;                    // 矩形度最大值
    int parent_area_min;                   // 父轮廓面积最小值
    int parent_area_max;                   // 父轮廓面积最大值
    int parent_hw_ratio_min;               // 父轮廓长宽比最小值
    int parent_hw_ratio_max;               // 父轮廓长宽比最大值
    int center_distance_min;               // 装甲板中心到能量机关中心最小值
    int center_distance_max;               // 装甲板中心到能量机关中心最大值

    TargetSizeParamRune()
    {
        len_min = 50;                      // 轮廓长度最小值
        len_max = 100;                     // 轮廓长度最大值
        hw_ratio_min = 100;                // 长宽比最小值
        hw_ratio_max = 200;                // 长宽比最大值
        area_min = 500;                    // 最小面积
        area_max = 3000;                   // 最大面积
        area_ratio_min = 50;               // 矩形度最小值
        area_ratio_max = 100;              // 矩形度最大值
        parent_area_min = 310;             // 父轮廓面积最小值
        parent_area_max = 720;             // 父轮廓面积最大值
        parent_hw_ratio_min = 180;         // 父轮廓长宽比最小值
        parent_hw_ratio_max = 230;         // 父轮廓长宽比最大值
        center_distance_min = 60;          // 装甲板中心到能量机关中心最小值
        center_distance_max = 70;          // 装甲板中心到能量机关中心最大值
    }
};


//-----------------------------------【参数-神符】--------------------------------------------
// brief：
//------------------------------------------------------------------------------------------
struct RuneParam
{
    struct RuneTimeParam rune_time_delay;          // 能量机关延迟时间
    struct RuneCompensate rune_compensate;         // 能量机关补偿
    struct TargetSizeParamRune tgt_size_param_c;   // 中心
    struct TargetSizeParamRune tgt_size_param_u;   // 未激活
};

//-----------------------------------【重定义数据结构类型】-------------------------------------
// brief：便于与电控进行协议沟通
//------------------------------------------------------------------------------------------
typedef signed int	       s32;
typedef unsigned int	   u32;
typedef signed short	   s16;
typedef unsigned short	   u16;
typedef unsigned char	   u8;

typedef signed int	       int32_t;
typedef unsigned int	   uint32_t;
typedef signed short	   int16_t;
typedef unsigned short	   uint16_t;
typedef unsigned char	   uint8_t;

//-----------------------------------【类-设置】-------------------------------------------
// brief：
//---------------------------------------------------------------------------------------
class MainSettings
{
public:
    MainSettings(const char *param);

    /**
     * @brief  读取其他参数
     * @param  param_path
     * @return void
     * @author 梁尧森
     * @date   2019.3.7
     */
    void readOtherParam(const char *param_path);

    /**
     * @brief  设置主要参数
     * @param  param_path
     * @return void
     * @author 梁尧森
     * @date   2019.3.7
     */
    void setMainParam(const char *win_name);

    /**
     * @brief  保存配置
     * @param  第一个参数
     * @return 返回值
     * @author 梁尧森
     * @date   2019.3.7
     */
    void writeOtherParam(const char *param_path);

    /**
     * @brief  设置工业相机位置
     * @param  窗口名称
     * @author 参与开发人员
     * @date   2019-
     */
    void setCameraParam(const char *win_name);

    /**
     * @brief  获取照片
     * @author 参与开发人员
     */
    bool grabImg(cv::Mat &img, char order, long interval);

public:
    int main_mode = armor_mode;            // 主要模式
    int main_mode_flag = 0;                // 模式切换标志位
    int enemy_color = red;                 // 敌方装甲板颜色
    int track_mode = normal_mode;          // ROI模式
    Debug debug;                           // 调试参数
    CameraParam camera_param;              // 工业相机初始化参数
    DeviceParam port_param;                // 串口通信参数
};

//-----------------------------------【类-设置】--------------------------------------------
// brief：
//----------------------------------------------------------------------------------------
class Settings
{
public:
    Settings();

    /**
     * @brief  设置目标预处理参数
     * @param  窗口名称
     * @return bool
     * @author 梁尧森
     * @date   2019.3.16
     */
    bool setPreprocessParam(const char *win_name);

    /**
     * @brief  设置目标预处理参数
     * @param  窗口名称
     * @return bool
     * @author 梁尧森
     * @date   2019.3.16
     */
    bool setPreprocessParam(const char *win_name, MainSettings *main_setting);

    /**
     * @brief  设置能量机关参数
     * @param  窗口名称
     * @author 参与开发人员
     * @date   2019-
     */
    void setTgtSizeParamRune(const char *win_name, struct TargetSizeParamRune &tgt_size_param);

    /**
     * @brief  设置目标检测参数
     * @param  窗口名称
     * @param  主要设置
     * @return bool
     * @author 梁尧森
     * @date   2019.3.16
     */
    bool setTgtSizeParam(const char *win_name, MainSettings *main_setting);

public:
    struct PreprocessParam preprocess_param;  // 预处理参数
    struct TargetSizeParam tgt_size_param;    // 目标参数
};

//-----------------------------------【类-装甲板设置】---------------------------------------
// brief：
//----------------------------------------------------------------------------------------
class ArmorSettings:public Settings
{
public:
    ArmorSettings(const char *param);

    /**
     * @brief  设置装甲参数线程
     * @param  主要设置
     * @return 返回值
     * @author 梁尧森
     * @date   2019.3.7
     */
    bool setArmorParam(MainSettings *main_setting);

    /**
     * @brief  获取装甲板配置
     * @param  param_path
     * @return void
     * @author 梁尧森
     * @date   2019.3.7
     */
    void readArmorParam(const char *param_path);

    /**
     * @brief  保存装甲配置
     * @param  参数文件路径
     * @return 返回值
     * @author 梁尧森
     * @date   2019.3.7
     */
    void writeArmorParam(const char *param_path);

public:
    double armor_height;               // 装甲板高度
    double armor_width;                // 装甲板宽度
    double small_armor_height;         // 小装甲板高度
    double small_armor_width;          // 小装甲板宽度

    ArmorParam armor_param;            // 装甲板参数
    std::vector<cv::Point> armor_contours;  // 装甲板轮廓
};

//-----------------------------------【类-神符设置】--------------------------------------------
// brief：
//------------------------------------------------------------------------------------------------
class RuneSettings:public Settings
{
public:
    RuneSettings(const char *param);

    /**
     * @brief  设置神符参数线程
     * @return 返回值
     * @author 张泾锋
     * @date   2019.3.15
     */
    void setRuneParam(MainSettings *main_setting);

    /**
     * @brief  读取神符配置
     * @param  参数文件路径
     * @return 返回值
     * @author 张泾锋
     * @date   2019.3.15
     */
    void readRuneParam(const char *param_path);

    /**
     * @brief  setRuneCompensate
     * @param  win_name
     * @param  rune_compensate
     * @author 包先宏
     * @date   2021.4.24
     */
    void setRuneCompensate(const char *win_name, struct RuneCompensate &rune_compensate);

    /**
     * @brief setRuneTimeParam
     * @param win_name
     * @param rune_time
     * @author 包先宏
     * @date 2021.4.24
     */
    void setRuneTimeParam(const char *win_name, struct RuneTimeParam &rune_time);

    /**
     * @brief  保存神符配置
     * @param  参数文件路径
     * @return 返回值
     * @author 张泾锋
     * @date   2019.3.15
     */
    void writeParamRune(const char *param_path);
public:
    RuneParam rune_param;
};

#endif // SETTINGS_H
