#ifndef TARGETDETECTION_H
#define TARGETDETECTION_H

#include "Settings/Settings.h"

class TargetDetection
{
public:
    TargetDetection(TargetSizeParam &target_param_size, Debug &debug, std::vector<cv::Point> contours_target = std::vector<cv::Point>());
    TargetDetection(TargetSizeParamRune &target_param_size, Debug &debug);

    /**
     * @brief  检测出符合面积大小，长宽比，hu特征的轮廓。返回新的列表
     * @param  源图-二值图
     * @param  目标轮廓列表引用
     * @return 是否有目标
     * @author 梁尧森
     * @date   2019.3.29
     */
    bool targetDetectionProc(cv::Mat src,
                              std::vector<std::vector<cv::Point>> &target_contours,
                              std::vector<cv::RotatedRect> &rot_rect_list,
                              std::vector<cv::Rect> &rect_list,
                              std::vector<struct TargetSize> &target_size_list);

    /**
     * @brief  检测出符合面积大小，长宽比。能量机关专用
     * @param  源图-二值图
     * @param  目标轮廓列表引用
     * @return 是否有目标
     * @author 梁尧森
     * @date   2019.3.29
     */
    bool targetDetectionProc(cv::Mat src,
                             std::vector<std::vector<cv::Point>> &target_contours,
                             std::vector<struct TargetSize> &target_size_list); //能量机关专用


    /**
     * @brief  返回新的列表
     * @param  轮廓
     * @param  筛选轮廓
     * @param  外接最大旋转矩形
     * @param  外接最大矩形
     * @param  轮廓属性
     * @return 返回值
     * @author 梁尧森
     * @date   2019.3.27
     */
    void filterByAttrubute(std::vector<std::vector<cv::Point>> contours,
                           std::vector<std::vector<cv::Point>> &target_contours,
                           std::vector<cv::RotatedRect> &rot_rect_list,
                           std::vector<cv::Rect> &rect_list,
                           std::vector<struct TargetSize> &target_size_list);

    /**
     * @brief  切换参数对象
     * @param  tsize_rune
     * @author 包先宏
     * @date   2021-
     */
    void setRuneTargetSize(TargetSizeParamRune &tsize_rune);

    /**
     * @brief  简介
     * @param  第一个参数
     * @return 返回值
     * @author 梁尧森
     * @date   2019.3.27
     */
    cv::RotatedRect adjustRect(const cv::RotatedRect &rect);

    /**
     * @brief  获取目标装甲板集合
     * @return 目标装甲板集合
     * @author 参与开发人员
     * @date   2019-
     */
    std::vector<cv::RotatedRect> getTargetRotRect();

    /**
     * @brief  获取目标装甲板拓扑信息
     * @return 目标装甲板拓扑信息
     * @author 参与开发人员
     * @date   2019-
     */
    std::vector<cv::Vec4i> getTargetHierarchy();

    /**
     * @brief  获取目标轮廓
     * @return 轮廓
     * @author 参与开发人员
     * @date   2019-
     */
    std::vector<std::vector<cv::Point2i>> getContours();

#ifdef DEBUG_MODE
public:
    cv::Mat ret;                              // src.clone()用于调试
    cv::Mat ret_2;                            // src.clone()用于调试
    cv::Mat ret_3;                            // src.clone()用于调试
    cv::Mat ret_4;
#endif // DEBUG_MODE

protected:
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i>              hierarchy;
    std::vector<cv::RotatedRect>        rot_rect_list;

private:
    TargetSizeParam *tsize;                   // 筛选使用的参数
    TargetSizeParamRune *rune_cur_size_param; // 能量机关参数
    std::vector<cv::Point> contours_target;   // 目标轮廓
    struct Debug *debug;                      // 主要用于显示调试窗口
};

#endif  // TARGETDETECTION_H
