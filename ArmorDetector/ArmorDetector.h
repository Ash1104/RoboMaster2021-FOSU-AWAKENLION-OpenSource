#ifndef ARMORDETECTOR_H
#define ARMORDETECTOR_H

#include "Settings/Settings.h"
#include "Preprocessing/Preprocessing.h"
#include "TargetDetection.h"
#include "AngleSolver/PnpSolver.h"
#include "Serial/PackData.h"
#include "AngleSolver/GravityCompensateResolve.h"
#define POINT_DIST(p1, p2) std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y-p2.y))
#define GET_DIST(a,b) std::abs(a-b)/(a>b?a:b)


class ArmorDetector: public AdaptiveThreshold, public TargetDetection
{
public:

    ArmorDetector(ArmorSettings *armor_setting, MainSettings *main_setting, std::vector<cv::Point> armor_contours);

    /**
     * @brief  自瞄模式
     * @param  原图
     * @return void
     * @author 醒狮战队所有参与自瞄开发的算法组成员
     * @date   2021.6.29
     */
    void armorDetecProc(cv::Mat src, AngleSolver &angle_slover,
                        PackData *pack_data, UnpackData *unpack_data,
                        Serial *serial, long t1_param, GravityCompensateResolve &gc,
                        ArmorDetector &armor,int &flag, int &lost_flag);

    /**
     * @brief  检测出所有装甲板
     * @param  原图
     * @return 装甲板四个点的列表
     * @author 梁尧森
     * @date   2019.3.31
     */
    bool detectorProc(cv::Mat src, QuadrilateralPos &pos,
                      cv::RotatedRect &rot_rect, bool &is_small);

    /**
     * @brief  进一步筛选灯条
     * @param  轮廓列表
     * @param  旋转矩形列表
     * @param  矩形列表
     * @param  目标尺寸信息列表
     * @param  灯条信息列表的引用
     * @author 梁尧森
     * @date   2019.4.20
     */
    void filterEnemyLight(cv::Mat src,
                          std::vector<std::vector<cv::Point>> light_contours,
                          std::vector<cv::RotatedRect> rot_rect_list,
                          std::vector<cv::Rect> rect_list,
                          std::vector<struct TargetSize> target_size_list,
                          std::vector<LightInfo> &light_info);

    /**
     * @brief  根据两个灯条匹配出装甲板
     * @param  灯条信息列表
     * @param  装甲板信息列表的引用
     * @author 梁尧森
     * @date   2019.4.20
     */
    void findArmorFrom2Light(std::vector<LightInfo> &light_list,
                             std::vector<ArmorInfo> &armor_list);

    /**
     * @brief  根据条件筛选装甲板
     * @param  装甲板信息列表
     * @param  装甲板信息列表的引用
     * @author 梁尧森
     * @date   2019.4.20
     */
    void filterAllArmor(std::vector<ArmorInfo> armor_list,
                        std::vector<ArmorInfo> &armor_all_list);

    /**
     * @brief 避免越界
     * @param rect
     * @param size
     */
    bool makeRectSafe(cv::Rect &rect, cv::Size size);

    /**
     * @brief  权重累加
     * @return 总和
     * @author 参与开发人员
     * @date   2019-
     */
    float addByWeight(std::vector<float> s, std::vector<float> w);

    /**
     * @brief  boundingRRect
     * @return 旋转矩形
     * @author 参与开发人员
     * @date   2019-
     */
    cv::RotatedRect boundingRRect(const cv::RotatedRect &left,
                                  const cv::RotatedRect &right);
public:
    double pit_final, yaw_final;      // 最终要发送的pitch、yaw轴点角度
    int armor_num;                    // 识别到的装甲板数目
    int fps;                          // 帧率

private:
    ArmorSettings *armor_setting;     // 装甲板滑动条参数
    MainSettings *main_setting;       // 主要设置（用于窗口的显示）
#ifdef DEBUG_MODE
    cv::Mat src_cl;                   // 克隆原图便于调试使用
#endif // DEBUG_MODE
};

#endif // ARMORDETECTOR_H
