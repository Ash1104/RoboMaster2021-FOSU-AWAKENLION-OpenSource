#include "Preprocessing.h"

AdaptiveThreshold::AdaptiveThreshold(struct PreprocessParam &preprocess_param)
{
    this->preprocess_param = &preprocess_param;
}

Preprocessing::Preprocessing()
{
    element_size = 5;
}

Preprocessing::~Preprocessing()
{
    element1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 5));
    element2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 7));
}

void AdaptiveThreshold::adaptiveThresholdProc(cv::Mat src, cv::Mat &dst, int method, int morphology, int iteration, int filter)
{
    switch (method)
    {
    case 0:  // red
    {
        std::vector<cv::Mat> rgbChannels(3);
        cv::split(src, rgbChannels);
        cv::Mat gray = rgbChannels[2] - rgbChannels[0];
        cv::threshold(gray, dst, preprocess_param->gray_thre_min, 255, cv::THRESH_BINARY);
        break;
    }
    case 1:  // blue
    {
        std::vector<cv::Mat> rgbChannels(3);
        cv::split(src, rgbChannels);
        cv::Mat gray = rgbChannels[0] - rgbChannels[0];
        cv::threshold(gray, dst, preprocess_param->gray_thre_min, 255, cv::THRESH_BINARY);
        break;
    }
    case 2:  // red过曝光
    {
        mulAndIterRedTre(src, dst, preprocess_info_rune);
    }
    case 3:  // blue过曝光
    {
        mulAndIterBlueTre(src, dst, preprocess_info_rune);
    }
    default:
        break;
    }
}

void AdaptiveThreshold::mulAndIterRedTre(cv::Mat src, cv::Mat &bin, PreprocessInfo &preprocess_info)
{
    cv::Mat gray_g = cv::Mat::zeros(src.size(), CV_8UC1);
    cv::Mat gray_r = cv::Mat::zeros(src.size(), CV_8UC1);

    // 因为常量会比变量快，所以当图像通道确定时，所以通道用常量
    int srcRows = src.cols * 3;
    for(int i = 0; i < src.rows; i++)
    {
        uchar *p_src = src.data+srcRows * i;
        uchar *p_gray_g = gray_g.data + gray_g.cols * i;
        uchar *p_gray_r = gray_r.data + gray_r.cols * i;
        int jj = 0;
        for(int j = 0; j < srcRows; j += 3)
        {
            *(p_gray_g + jj) = *(p_src + j + 1);
            *(p_gray_r + jj) = *(p_src + j) * 0.1 + *(p_src + j + 1) * 0.2 + *(p_src + j + 2) * 0.7;
            jj++;
        }
    }
    // 最大灰度
    uchar gray_max = gray_r.data[0];
    for(int i = 0; i < gray_r.rows; i++)
    {
        uchar *p = gray_r.data + i * gray_r.cols;
        for(int j = 0; j < gray_r.cols; j++)
        {
            if(*(p + j) > gray_max)
                gray_max = *(p + j);
        }
    }

    int &thre = preprocess_info.thre;
    thre = gray_max * preprocess_param->gray_max_w / 10. + avgGrayThreshold(gray_r) * preprocess_param->gray_avg_w / 1.;


    VAL_LIMIT(thre, preprocess_param->gray_thre_min, preprocess_param->gray_thre_max);

    cv::threshold(gray_r, bin, thre, 255, 0);
}

void AdaptiveThreshold::mulAndIterBlueTre(cv::Mat src, cv::Mat &bin, PreprocessInfo &preprocess_info)
{
    cv::Mat gray_b = cv::Mat::zeros(src.size(), CV_8UC1);
    cv::Mat gray_g = cv::Mat::zeros(src.size(), CV_8UC1);

    int srcRows = src.cols * 3;
    for(int i = 0; i < src.rows; i++)
    {
        uchar *p_src = src.data + srcRows * i;
        uchar *p_gray_b = gray_b.data + gray_b.cols * i;
        uchar *p_gray_g = gray_g.data + gray_g.cols * i;

        int jj = 0;
        for(int j = 0; j < srcRows; j += 3)
        {
            *(p_gray_b + jj) = *(p_src + j) * 0.7 + *(p_src + j + 1) * 0.2 + *(p_src + j + 2) * 0.1;
            *(p_gray_g + jj) = *(p_src + j + 1);
            jj++;
        }
    }
    //最大灰度
    uchar gray_max = gray_b.data[0];
    for(int i = 0; i < gray_b.rows; i++)
    {
        uchar *p = gray_b.data + i * gray_b.cols;
        for(int j = 0; j < gray_b.cols; j++)
        {
            if(*(p + j) > gray_max)
                gray_max = *(p + j);
        }
    }

    int &thre = preprocess_info.thre;
    thre = gray_max * preprocess_param->gray_max_w / 10. + avgGrayThreshold(gray_b) * preprocess_param->gray_avg_w / 1.;

    VAL_LIMIT(thre, preprocess_param->gray_thre_min, preprocess_param->gray_thre_max);

    cv::threshold(gray_b, bin, thre, 255, 0);
}

int AdaptiveThreshold::avgGrayThreshold(cv::Mat src)
{
    long sum_gray = 0;
    for(int j = 0; j < src.rows; j++)
    {
        uchar *data = src.ptr<uchar>(j);
        for(int i = 0; i < src.cols; i++)
        {
            sum_gray += data[i];
        }
    }
    return sum_gray * 1.0 / (src.cols * src.rows);
}

void AdaptiveThreshold::filter(cv::Mat &dst)
{
    //cv::erode(dst, dst, element1);
    cv::dilate(dst, dst, element2);

//    setStructElement();
//    cv::morphologyEx(dst, dst, cv::MORPH_OPEN, element);
//    cv::morphologyEx(dst, dst,cv::MORPH_CLOSE,element);
    cv::medianBlur(dst, dst, 3);
}
