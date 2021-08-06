#ifndef PNPSOLVER_H
#define PNPSOLVER_H

#include "Settings/Settings.h"
#include "GravityCompensateResolve.h"
#include <math.h>
#include <QTextStream>
#include <QFile>



class AngleSolver
{
public:
    AngleSolver(const char *camera_param_file_name, double z_scale = 1.0, double min_dist = 10.0, double max_dist = 1000.0);

    /**
     * @brief  设置相机安装参数
     * @param  第一个参数含义
     * @author 梁尧森
     * @date   2018.9.21
     */
    void setRelationPoseCameraPTZ(const double ptz_camera_x, const double ptz_camera_y,
                                  const double ptz_camera_z, double y_offset_barrel_ptz);

    /**
     * @brief  设置目标尺寸
     * @param  目标矩形真实宽度
     * @param  目标矩形真实高度
     * @return 返回值含义
     * @author 梁尧森
     * @date   2018.9.21
     */
    void setTargetSize(double width, double height);

    /**
     * @brief  解析角度
     * @author 梁尧森
     * @date   2018.9.21
     */
    bool getAngle(std::vector<cv::Point2f> target2d, double bullet_speed,
                  double current_ptz_angle=0, const cv::Point2f &offset=cv::Point2f());

    /**
     * @brief  solvePnP
     * @author 参与开发人员
     * @date   2018-
     */
    void solvePnP4Points(const std::vector<cv::Point2f> &points2d,
                         cv::Mat &rot, cv::Mat &trans);

    /**
     * @brief  相机坐标转换到PTZ坐标
     * @param  目标在相机坐标下的位置
     * @param  目标在PTZ坐标下的位置
     * @author 梁尧森
     * @date   2018.9.21
     */
    void tranformationCamera2PTZ(const cv::Mat &pos, cv::Mat &transed_pos);

    /**
     * @brief  根据目标在PTZ坐标中的位置，计算偏移角度，使枪管瞄准目标
     * @param  第一个参数含义
     * @author 梁尧森
     * @date   2018.9.21
     */
    void adjustPTZ2Barrel(const cv::Mat &pos_in_ptz,double &angle_x, double &angle_y,
                          double bullet_speed, double current_ptz_angle);

    /**
     * @brief  rectPnpSolver
     * @param  camera_param_file_name
     * @author 参与开发人员
     * @date   2018-
     */
    void rectPnpSolver(const char *camera_param_file_name);

    /**
     * @brief  获取光轴中心
     * @return 光轴中心
     * @author 梁尧森
     * @date   2018.9.21
     */
    cv::Point2f getImageCenter();

    /**
     * @brief 获取相对角度
     * @param 目标矩形
     * @param 正式长宽比
     * @param 补偿
     */
    bool getAngleWithRectify(const cv::RotatedRect &rect, double wh_ratio,
                             const cv::Point2f &offset = cv::Point2f());

    /**
     * @brief  获取矩形四个角点的坐标
     * @author 梁尧森
     * @date   2018.9.21
     */
    void getTarget2dPoinstion(const cv::RotatedRect &rect, std::vector<cv::Point2f> &target2d,
                              const cv::Point2f &offset);

    /**
     * @brief  无重力相对角解算
     * @param  目标坐标点
     * @author 参与开发人员
     * @date   2018-
     */
    bool getAngleWithoutGavity(std::vector<cv::Point2f> target2d);

    /**
     * @brief  能量机关专用PnP解算
     * @author 参与开发人员
     * @date   2020-
     */
    void solvePnP4Points_rune(const std::vector<cv::Point2f> &points2d, cv::Mat &rot, cv::Mat &trans);

    /**
     * @brief 无重力补偿
     * @author 参与开发人员
     * @date   2020-
     */
    void adjustPTZ2BarrelWithoutGavity(const cv::Mat &pos_in_ptz,
                                       double &angle_x, double &angle_y);

    /**
     * @brief
     * @param  第一个参数含义
     * @return 返回角度、距离、姿态角等信息
     * @author 梁尧森
     * @date   2018.9.21
     */
    double getPitRef();

    /**
     * @brief
     * @param  第一个参数含义
     * @return 返回角度、距离、姿态角等信息
     * @author 梁尧森
     * @date   2018.9.21
     */
    double getYawRef();

    /**
     * @brief  单目测距
     * @param  armor_rect
     * @param  dist
     * @author 陈强
     * @date   2021-
     */
    void getDistanceDanmu(std::vector<cv::Point2f>armor_rect, double &dist);

public:
    cv::Mat cam_matrix;                 // 内参矩阵
    cv::Mat distortion_coeff;           // 畸变系数
    double width_target;                // 目标矩形真实宽度
    double height_target;               // 目标矩形真实高度
    std::vector<cv::Point3f> point3d;   // 目标三维坐标
    cv::Mat rot;                        // 旋转矩阵
    cv::Mat position_in_camera;         // 偏移向量
    cv::Mat position_in_ptz;            // 目标在PTZ坐标中的位置
    double pit_ref;                     // 俯仰角
    double yaw_ref;                     // 偏航角
private:
    cv::Mat rot_camera2ptz;             // 旋转矩阵（相机坐标转换到PTZ坐标）
    cv::Mat trans_camera2ptz;           // 偏移向量（相机坐标转换到PTZ坐标）
    double offset_y_barrel_ptz;         // 云台中心与枪管的偏移量
    double min_distance;                // 最小检测距离
    double max_distance;                // 最大检测距离
    double scale_z;                     // 放缩比例
};

#endif
