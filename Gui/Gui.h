#ifndef Gui_H
#define Gui_H

#include "Settings/Settings.h"

typedef struct ProOffset
{
    float proportion_v;   // 值比例
    float proportion_t;   // 时间比例
    int offset_v;         // 值偏移
    int offset_t;         // 时间偏移
    ProOffset(){}
}ProOffset_t;

class Gui{
public:
    Gui(cv::Point2f center, double digits_size, int status = 0, cv::Mat img = cv::Mat());

private:
    cv::Mat src;
    cv::Mat ret_img;
    cv::Point2f center;
    double digits_size;
    int status;
};

class DataCollect
{
public:
    DataCollect(int canvas_w, int canvas_h, int ndataSize = -1, int ndataRange = -1);

    /**
     * @brief  调用私有方法dataCollect，通过多次调用，传入多个数据，在一个窗口绘制该组数据的波形
     * @param  显示的数据向量
     * @param  对应的颜色向量
     * @param  窗口名
     * @author 张泾锋
     * @date   2018-11-14
     */
    void dataCollectProc(std::vector<float> data, std::vector<cv::Scalar> color, std::string winname);

    /**
     * @brief  设置画布
     * @author 梁尧森
     * @date   2018-11-14
     */
    void setCanvas();

    /**
     * @brief  gui
     * @param  显示的数据
     * @param  对应的颜色
     * @author 梁尧森
     * @date   2018-11-14
     */
    void gui(const std::string winname);

    /**
     * @brief  私有方法，实现核心的数据波形绘制操作
     * @param  显示的数据
     * @param  对应的颜色
     * @param  窗口名
     * @author 梁尧森
     * @date   2018-11-14
     */
    void dataCollect(float data,cv::Scalar color,int data_q_index,const std::string winname);


private:
    float data_q[DATA_BUFF_ROW_SIZE_MAX][DATA_BUFF_COL_SIZE_MAX]; // 数据队列
    int data_q_h;
    int canvas_w;    // 画布宽度
    int ndataSize;   // 显示数据个数
    int ndataRange;  // 显示数据范围
    float interval;  // 区间=画布宽度/显示数据个数
    int canvas_h;    // 画布高度
    cv::Mat coll;    // 画布
    ProOffset pro_offset;
};

#endif /* Gui_hpp */
