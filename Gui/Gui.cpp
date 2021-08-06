#include "Gui.h"

int is_changeOffset;
int offset_v_tmp;
int offset_t_tmp;
int xold;
int yold;

Gui::Gui(cv::Point2f center, double digits_size, int status, cv::Mat img)
{
    this->ret_img = img.clone();
    this->center = center;
    this->digits_size = digits_size;
    this->status = status;

    cv::circle(this->ret_img, cv::Point(center), 3, cv::Scalar(0, 255, 0), 1);
    cv::line(this->ret_img, cv::Point(0, center.y), cv::Point(img.cols, center.y), cv::Scalar(0, 127, 0), 1);
    cv::line(this->ret_img, cv::Point(center.x, 0), cv::Point(center.x, img.rows), cv::Scalar(0, 127, 0), 1);
}

DataCollect::DataCollect(int canvas_w, int canvas_h, int ndataSize, int ndataRange)
{
    for(int i = 0; i < DATA_BUFF_ROW_SIZE_MAX; i++)
        for(int j = 0; j < DATA_BUFF_COL_SIZE_MAX; j++)
            data_q[i][j] = 0;

    data_q_h = 0;
    this->canvas_w = canvas_w;              // 画布宽度

    if(ndataSize == -1)
        this->ndataSize = canvas_w;         // 显示数据个数 = 画布宽度
    else
        this->ndataSize = ndataSize;        // 显示数据个数

    if(ndataRange == -1)
        this->ndataRange = canvas_h;        // 显示数据范围 = 画布高度
    else
        this->ndataRange = ndataRange;      // 显示数据范围

    interval = float(canvas_w) / canvas_w;  // 区间 = 画布宽度 / 显示数据个数
    this->canvas_h = canvas_h;

    pro_offset.proportion_v = 1;  // 值比例
    pro_offset.proportion_t = 1;  // 时间比例
    pro_offset.offset_v = -90;    // 值偏移
    pro_offset.offset_t = 0;      // 时间偏移
}

void DataCollect::dataCollectProc(std::vector<float> data, std::vector<cv::Scalar> color, std::string winname)
{
    //黑色的背景
    coll = cv::Mat::zeros(canvas_h, canvas_w, CV_8UC3);

    setCanvas();
    gui(winname);

    for (int i = 0; i < data.size(); i++)
    {
        if (i < DATA_BUFF_ROW_SIZE_MAX)
            dataCollect(data[i], color[i], i, winname);
        else
        {
            std::cout << "Error: The number of data vectors exceeding the maximum!" << std::endl;
            return;
        }
    }
    data_q_h++;
    cv::imshow(winname, coll);
}

void on_MouseHandle(int event, int x, int y, int flags, void* param)
{
    ProOffset_t *offset = (struct ProOffset *)param;

    switch(event)
    {
     //鼠标移动消息
    case cv::EVENT_MOUSEMOVE:
    {
        if(is_changeOffset)//如果是否进行绘制的标识符为真，则记录下长和宽到RECT型变量中
        {
            //设置偏移
            offset->offset_v = offset_v_tmp + y-yold;
            offset->offset_t = offset_t_tmp + x-xold;
        }
    }
    break;

    //左键按下消息
    case cv::EVENT_LBUTTONDOWN:
    {
        is_changeOffset = true;
        xold = x;
        yold = y;
    }
    break;

    //左键抬起消息
    case cv::EVENT_LBUTTONUP:
    {
        is_changeOffset = false;//置标识符为false
        offset_v_tmp = offset->offset_v;
        offset_t_tmp = offset->offset_t;
    }
    break;
    }
}


void DataCollect::gui(const std::string winname)
{
    cv::setMouseCallback(winname, on_MouseHandle, (void*)&pro_offset);
}

void DataCollect::setCanvas()
{
    int color_gray = 100;

    for(int y = 0; y < canvas_h; y += 64)
    {
        //横线
        line(coll, cv::Point(pro_offset.offset_t, y + pro_offset.offset_v),
             cv::Point(canvas_w + pro_offset.offset_t, y + pro_offset.offset_v),
             cv::Scalar(color_gray, color_gray, color_gray), 1, 8, 0);
        //横坐标
        char row_coor[10];
        sprintf(row_coor, "%d", y);
        putText(coll, row_coor,
                cv::Point(canvas_w - 50 + pro_offset.offset_t, canvas_h - y + pro_offset.offset_v),
                1, 1, cv::Scalar(color_gray,color_gray, color_gray), 1, 8);
    }

    for(int x = 0; x < canvas_w; x += 50)
    {
        //纵线
        line(coll, cv::Point(x + pro_offset.offset_t, pro_offset.offset_v),
             cv::Point(x + pro_offset.offset_t, canvas_h + pro_offset.offset_v),
             cv::Scalar(color_gray, color_gray, color_gray), 1, 8, 0);
        //纵坐标
        char col_coor[10];
        sprintf(col_coor, "%d", x);
        putText(coll, col_coor,
                cv::Point(canvas_w - x + pro_offset.offset_t, canvas_h - 50 + pro_offset.offset_v),
                1, 1, cv::Scalar(color_gray, color_gray, color_gray), 1, 8);
    }
}

void DataCollect::dataCollect(float data,cv::Scalar color,int data_q_index,const std::string winname)
{
    //data_q_h 是当前数据写入的索引（数组下标）
    //data_q_h 超出范围后归0
    //data_q   就是一个用数组实现的循环队列
    data_q_h = data_q_h % ndataSize;
    data_q[data_q_index][data_q_h] = data;
    //缩放到合适的范围
    data_q[data_q_index][data_q_h] *= (canvas_h / ndataRange);

    char fdata[10] = "";
    std::sprintf(fdata, "%.f", data);
    cv::putText(coll, fdata, cv::Point(canvas_w -35 + pro_offset.offset_t, canvas_h - data + pro_offset.offset_v), 1, 1, color, 1, 8);

    //将 data_q 循环队列中的所有数据（点）连接起来，画出
    int p_data;
    for(int j = 0; j < ndataSize - 1; j++)
    {
        p_data = data_q_h - j + canvas_w;
        //画线
        //第2个参数：前一个点（后一个点的数据位置）Point(x,y) x是时间轴，从前各往后 y是数据的大小
        //第3个参数：当前点（当前的数据位置）
        line(coll, cv::Point(interval * (ndataSize - j - 1) + pro_offset.offset_t,
                         canvas_h - data_q[data_q_index][(p_data - 1) % canvas_w] * pro_offset.proportion_v + pro_offset.offset_v),
             cv::Point(interval * (ndataSize - j) + pro_offset.offset_t,
                   canvas_h - data_q[data_q_index][p_data % canvas_w] * pro_offset.proportion_v + pro_offset.offset_v),
             color, 1, 8, 0);
    }
}
