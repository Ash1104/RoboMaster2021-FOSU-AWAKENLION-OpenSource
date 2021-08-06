//-----------------------------------【title】--------------------------------------------
// brief：
//------------------------------------------------------------------------------------------------

#ifndef PREPROCESSING_H
#define PREPROCESSING_H

#include "Settings/Settings.h"

#define VAL_LIMIT(val, min, max) \
do {\
if((val) <= (min))\
{\
  (val) = (min);\
}\
else if((val) >= (max))\
{\
  (val) = (max);\
}\
} while(0)\

class Preprocessing
{
public:
    Preprocessing();
    ~Preprocessing();

public:
    cv::Mat element;
    cv::Mat element1;
    cv::Mat element2;
    int element_size;
};

class AdaptiveThreshold:Preprocessing
{
public:
    AdaptiveThreshold(struct PreprocessParam &preprocess_param);

    /**
     * @brief  局部二值化
     * @param  源图
     * @author 梁尧森
     * @date   2018.11.7
     */
    void adaptiveThresholdProc(cv::Mat src, cv::Mat &dst, int method = -1, int morphology = -1, int iteration = 1, int filter = -1);

    /**
     * @brief  正片叠底+迭代法测试（红色）
     * @param  第一个参数
     * @return 返回值
     * @author 梁尧森
     * @date   2018-
     */
    void mulAndIterRedTre(cv::Mat src, cv:: Mat &bin, struct PreprocessInfo &preprocess_info);

    /**
     * @brief  正片叠底+迭代法测试（蓝色）
     * @param  第一个参数
     * @return 返回值
     * @author 梁尧森
     * @date   2018-
     */
    void mulAndIterBlueTre(cv::Mat src, cv:: Mat &bin, struct PreprocessInfo &preprocess_info);

    /**
     * @brief  平均灰度法
     * @param  第一个参数
     * @return 返回值
     * @author 梁尧森
     * @date   2018-
     */
    int avgGrayThreshold(cv::Mat src);

    /**
     * @brief  腐蚀膨胀
     * @param  二值图
     * @author 参与开发人员
     * @date   2018-
     */
    void filter(cv::Mat &dst);

private:
    struct PreprocessParam *preprocess_param;  // 预处理调试参数
    struct PreprocessInfo preprocess_info_rune;
};

#endif
