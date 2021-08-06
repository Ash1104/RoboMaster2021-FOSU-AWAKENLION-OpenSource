#ifndef RUNEDETECTOR_H
#define RUNEDETECTOR_H

#include "Settings/Settings.h"
#include "Serial/Serial.h"
#include "Serial/PackData.h"
#include "AngleSolver/PnpSolver.h"
#include "ArmorDetector/TargetDetection.h"
#include "Preprocessing/Preprocessing.h"

//#define TRIG2SHOOT2HIT_DELAY 0.6 //需要超前转动的时间：下位机响应时间 + 发弹延时 + 子弹发出到击中的时间（S）：0.1（假设） + 0.145（大概） + 0.25（接近）//0.482
//#define PTZ_SHIFT_DELAY 0.3 // 允许云台移动的时间（云台一定要响应完成并到达）：下位机响应时间 + 云台移动延时 + 到达后的缓冲时间（S）：0.1（假设） + 0.47（大概） + 0.038 //0.65
//#define SHOOT2HIT_DELAY 0.82 // 等待子弹发射到装甲点亮的时间：子弹发出到击中的时间 + 到达后的缓冲时间（注：包括打中装甲到点亮过程的延迟，赛场跟实验室的延迟不一样）（S）：0.25（接近） + 0.04
#define SHOOT_NUM 6 // (8 + 1) 9times
#define PTZ2TARGET_RELATIVE_DIS 685 // 原点到目标水平相对距离，即z轴上的距离，子弹发射初始位置再到目标距离：643.0 + 42 = 685
#define FRAME_FILTER_NUM 20 // 大能量机关模式开始时先读的FRAME_FILTER_NUM + 1帧，过滤筛选合适的需要的数据
#define RUNE_ARMOR_WD_RADIO 1.7857 // 1.923077 //1.80212
#define OFFSET_Z_PTZ2BULLET 7.5 // 云台中心与子弹发射初始位置的偏移量

class DetectRuneCenter
{
public:
    DetectRuneCenter(RuneSettings *rune_settings, MainSettings *main_settings);

    /**
     * @brief 设置能量机关中心
     * @param 原图
     * @param 装甲板旋转矩形
     * @author 包先宏
     */
    bool setRuneCenter(cv::Mat src, cv::RotatedRect rune_armor_corner);

    /**
     * @brief 获取装甲板中心点
     * @return 装甲板中心点坐标
     * @author 包先宏
     */
    cv::Point2f getRuneCenter(void);

    /**
     * @brief 红蓝预处理
     * @param 原图
     * @param 处理后的图
     * @param 阈值
     */
    void redOrBlueTre(cv::Mat src, cv::Mat &dst, AdaptiveThreshold &threshold);

public:
    RuneSettings *rune_settings;
    MainSettings *main_settings;
    TargetDetection center_detection;
    AdaptiveThreshold center_adaptive_threshold;

protected:
    cv::Point2f rune_center;
};

class RuneDetector:public DetectRuneCenter
{
public:
    RuneDetector(RuneSettings *rune_settings, MainSettings *main_settings, bool is_clockwise = true);

    /**
     * @brief 获取CP上检索的矩形
     * @return 目标装甲板集合
     * @author 包先宏
     */
    std::vector<cv::RotatedRect> getPrepTargetRotRect();

    /**
     * @brief 获取当前未激活风叶的位置
     * @param 原图
     * @param 装甲板旋转矩形
     * @param 装甲板四个角点坐标
     */
    bool getCurPos(cv::Mat src, cv::RotatedRect &rot_rect, QuadrilateralPos &cur_pos);

private:
    TargetDetection prep_activate_detection;
    AdaptiveThreshold prep_activate_adaptive_threshold;

protected:
    bool is_clockwise;//神符是否为顺时针方向旋转
};


class RuneHitLogic:public RuneDetector
{
public:
    RuneHitLogic(RuneSettings *rune_settings,
                 MainSettings *main_settings, bool is_clockwise = true);

    /**
     * @brief   小能量机关模式
     * @authors 张泾锋、包先宏
     * @date    2020-
     */
    void smallRuneHitProc(cv::Mat src, AngleSolver &angle_solver,
                          PackData *pack_data, UnpackData *unpack_data, Serial *serial,
                          long t1_param, GravityCompensateResolve &gc);

    /**
     * @brief   大能量机关模式
     * @authors 张泾锋、包先宏
     * @date    2020-
     */
    void bigRuneHitProc(cv::Mat src, AngleSolver &angle_solver,
                        PackData *pack_data, UnpackData *unpack_data, Serial *serial,
                        long t1_param, GravityCompensateResolve &gc);

    /**
     * @brief   静止能量机关模式
     * @authors 张泾锋,包先宏
     * @date    2020-
     */
    void staticRuneHitProc(cv::Mat src, AngleSolver &angle_solver,
                          PackData *pack_data, UnpackData *unpack_data, Serial *serial,
                          long t1_param, GravityCompensateResolve &gc);

    /**
     * @brief 判断是否继续下一次打击
     * @param 原图
     * @param angle_solver
     * @param pack_data
     * @param serial
     * @param t1_param
     * @param gc
     */
    void judgeHitOrShift(cv::Mat src, AngleSolver &angle_solver,
                         PackData *pack_data, UnpackData *unpack_data, Serial *serial, long t1_param,
                         GravityCompensateResolve &gc);

    /**
     * @brief 待激活装甲板关于能量机关中心点角度
     * @param 待激活装甲板中心坐标
     * @author 包先宏
     */
    void setAngleRecent(cv::Point2f pos);

    /**
     * @brief 获取角度差这段的时间差
     * @return 时间差
     * @author 包先宏
     */
    double getAngleDiffForBigRuneInit();

    /**
     * @brief 计算t1时刻所在正弦函数周期所在的t1
     * @param 角度差
     * @param 时间差
     * @return 周期时间t1
     */
    double calStartTimeInPeriod(float angle_diff_of_interval, double time_interval);

    /**
     * @brief 计算周期时间内转过的角度处理
     * @return 角度
     */
    double predictAngleForSin(void);

    /**
     * @brief 计算在T时间内转过的角度
     * @param 周期时间t1
     * @param 时间差
     * @return 角度
     */
    double calPredictAngleByPeriod(double time_begin, double time_interval);

    /**
     * @brief 预测转动角度
     * @param 目标装甲板矩形
     * @param angle_solver
     * @param pack_data
     * @param serial
     * @param t1_param
     * @param 旋转角度
     * @param gc
     */
    void predictShiftProcWithCompansate(cv::RotatedRect &rot_rect, AngleSolver &angle_solver,
                                        PackData *pack_data, UnpackData *unpack_data, Serial *serial, long t1_param,
                                        float rotated_angle, GravityCompensateResolve &gc);

    /**
     * @brief 待移动角度
     * @param 目标矩形
     * @param angle_solver
     * @param pack_data
     * @param serial
     * @param t1_param
     * @param gc
     */
    void normalShiftProcWithCompansate(cv::RotatedRect &rot_rect, AngleSolver &angle_solver,
                                       PackData *pack_data, UnpackData *unpack_data, Serial *serial, long t1_param,
                                       GravityCompensateResolve &gc);

    /**
     * @brief 预测绝对角度
     * @param 目标pit绝对角
     * @param 目标yaw绝对角
     * @param T时间后旋转的角度
     */
    void predictAbsAngle(float &pit_abs_angle, float &yaw_abs_angle, float rotated_angle);

    /**
     * @brief 装甲板中点预测
     * @param 当前装甲板位置
     * @param 旋转角
     */
    void getPrePosPoint(cv::Point3f cur_pos_point, float rotated_angle);

    /**
     * @brief 解算绝对角
     * @param 世界坐标系三维点
     * @param pit_abs_angle
     * @param yaw_abs_angle
     */
    void calCoorinateAbsAngle(cv::Point3f curPos_3d_ori, float &pit_abs_angle, float &yaw_abs_angle);


    /**
     * @brief 角度补偿(赛前和训练跟电控对好补偿)
     * @param pit_ref_send
     * @param yaw_ref_send
     * @author 包先宏
     */
    void CompensateAngle(UnpackData *unpack_data, float &pit_ref_send, float &yaw_ref_send);

    /**
     * @brief 装甲板世界坐标系坐标
     * @param pitch绝对角
     * @param yaw绝对角
     * @return 装甲板3d点
     */
    cv::Point3f calCurPosCoordinate(float pit_abs_angle, float yaw_abs_angle);

    /**
     * @brief 发送打击命令
     * @param pack_data
     * @param serial
     * @param t1_param
     */
    void sendHitMsg2Stm(PackData *pack_data, Serial *serial, long t1_param);

    /**
     * @brief 发送移动命令
     * @param pack_data
     * @param serial
     * @param t1_param
     */
    void sendShiftMsg2Stm(PackData *pack_data, Serial *serial, long t1_param);

    /**
     * @brief 发送停止命令
     * @param pack_data
     * @param serial
     * @param t1_param
     */
    void sendStopMsg2Stm(PackData *pack_data, Serial *serial, long t1_param);

    /**
     * @brief 获取当前移动角度
     * @param 目标矩形
     * @param angle_solver
     * @param 长宽比
     */
    void getNormalShiftAngle(cv::RotatedRect &rot_rect, AngleSolver &angle_solver, float ratio);

    /**
     * @brief 重力补偿
     * @param gc
     */
    void setPitAbsAngleWithGavityCompansate(GravityCompensateResolve &gc);

    /**
     * @brief  方向稳定性检测
     * @author 张泾锋
     * @date   2020-
     */
    void setRuneRotDirection(void);

    /**
     * @brief 选取能量机关三维坐标
     * @return 能量机关中心点三维坐标
     */
    cv::Point3f selectRuneCenter3d(void);

    /**
     * @brief 选取能量机关方向
     */
    bool selectRuneRotation(void);

    /**
     * @brief 更新旋转方向
     * @param is_clockwise
     */
    void updateRuneRotDirection(bool is_clockwise);

    /**
     * @brief 获取pit绝对角
     * @return pit绝对角
     */
    float getPitAbsAngle(void);

    /**
     * @brief 获取yaw绝对角
     * @return yaw绝对角
     */
    float getYawAbsAngle(void);

    /**
     * @brief 获取旋转方向
     */
    bool getRotation(void);

private:
    int frame_count;  // 帧数
    int shoot_count;  // 发送射击命令次数

    uint16_t state_change;  // 模式

    struct timeval t1_difftime;  // 计时
    struct timeval t2_difftime;  // 计时
    struct timeval t1_time;
    struct timeval t2_time;
    struct timeval t1_time_to_stop;
    struct timeval t2_time_to_stop;

    float T; //预测位置专用时间
    float angle_recent;  // 现在的角度
    float angle_recent_2PI;  // 现在的角度 0-2pi
    float angle_old;  // 上一帧点角度
    float angle_old_2PI; // 上一帧的角度 0-2pi
    float angle_diff;  // 角度差

    float pit_abs_angle;  // 绝对角
    float yaw_abs_angle;  // 绝对角
    float pit_gavity_compansate_angle;  // 重力补偿角度

    cv::Point2f center_2d;  // 中心点2d点
    cv::Point3f center_point_3d;  // 中心点3d点
    cv::Point3f cur_pos_point_3d;  // 装甲板3d点
    cv::Point3f pre_pos_point_3d;  // 预测点

    std::vector<bool> rotation_capacity;  // 旋转方向
    std::vector<cv::Point3f> center_capacity;  // 能量机关中心点

    cv::RotatedRect rot_rect;  // 装甲板矩形
    QuadrilateralPos cur_pos;  // 装甲板角点

    float pit_compansate_angle;  // pit补偿角
    float yaw_compansate_angle;  // yaw补偿角

    //（大小机关不同之处）
    float amplitude; //正弦函数的振幅
    float angular_frequency; //正弦函数的角频率
    float initial_phase; //正弦函数的初相
    float const_number; //正弦函数的常量系数
    double time_begin_in_period; //初始时间对应周期里的时间
    float angle_old_for_bigRune; //初始角度（周期变速运动的大能量机关用）

    int active_rune_num = 0;

    struct timeval t1_difftime_for_period;  // 周期时间t1
    struct timeval t2_difftime_for_period;  // 周期时间t2

    double rotated_angle;  // 旋转角度
};


#endif // RUNEDETECTOR_H
