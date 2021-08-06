#include "ArmorDetector.h"

ArmorDetector::ArmorDetector(ArmorSettings *armor_setting, MainSettings *main_setting, std::vector<cv::Point> armor_contours)
    :AdaptiveThreshold(armor_setting->preprocess_param), TargetDetection(armor_setting->tgt_size_param, main_setting->debug, armor_contours)
{
    this->armor_setting = armor_setting;
    this->main_setting = main_setting;
}

void ArmorDetector::armorDetecProc(cv::Mat src, AngleSolver &angle_slover,
                                   PackData *pack_data, UnpackData *unpack_data,
                                   Serial *serial, long t1_param, GravityCompensateResolve &gc,
                                   ArmorDetector &armor,int &flag, int &lost_flag)
{
    bool is_small;               // 是否小装甲板
    bool is_find_armor = false;  // 是否寻找到目标装甲板
    QuadrilateralPos armor_pos;  // 装甲板四个点坐标
    cv::RotatedRect rot_rect;    // 装甲板旋转矩形
    static double pit_last, yaw_last;
    // 1.识别灯条; 2.获取所有装甲板四个点位置; 3.选择最优装甲板
    is_find_armor = armor.detectorProc(src, armor_pos, rot_rect, is_small);

    std::cout << src.rows << "   " << src.cols << std::endl;
    // 4.解算角度
    if(is_find_armor)
    {
        flag++;
        lost_flag = 0;
        std::vector<cv::Point2f> armor_rect;
        float ex_armor_x, ex_armor_y;
        ex_armor_x = std::abs(armor_pos.p[0].x - armor_pos.p[2].x);
        ex_armor_y = armor_pos.p[0].y - armor_pos.p[3].y;

        // 装甲板扩展预测 不用考虑越界问题
        if(unpack_data->getStm2PcMesg()->stm32_info_data.run_left == 1)
        {
            for(int i = 0; i < 4; i++)
            {
                armor_pos.p[i].x += ex_armor_x;
                armor_pos.p[i].y -= ex_armor_y;
            }
        }
        else if(unpack_data->getStm2PcMesg()->stm32_info_data.run_left == 2)
        {
            for(int i = 0; i < 4; i++)
            {
                armor_pos.p[i].x -= ex_armor_x;
                armor_pos.p[i].y += ex_armor_y;
            }
        }

        if(unpack_data->getStm2PcMesg()->stm32_info_data.is_left == 1)
        {
            for(int i = 0; i < 4; i++)
            {
                armor_pos.p[i].x -= ex_armor_x;
                armor_pos.p[i].y += ex_armor_y;
            }
        }
        else if(unpack_data->getStm2PcMesg()->stm32_info_data.is_left == 2)
        {
            for(int i = 0; i < 4; i++)
            {
                armor_pos.p[i].x += ex_armor_x;
                armor_pos.p[i].y -= ex_armor_y;
            }
        }

        for(int i = 0; i < 4; i++)
            armor_rect.push_back(armor_pos.p[i]);

#ifdef DEBUG_MODE
        cv::Mat src_clone = src.clone();
        for(int i = 0; i < 4; i++)
            cv::line(src_clone, armor_pos.p[i], armor_pos.p[(i + 1) % 4], cv::Scalar(0, 255, 0), 2, 8);
        if(main_setting->debug.b_show_fps)
        {
            std::ostringstream s_fps;
            s_fps << armor.fps;
            cv::putText(src_clone, "FPS: " + s_fps.str(), cv::Point(20, 30), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 0), 2, 8);
        }
        if(main_setting->debug.b_show_ex_armor)
        {
            cv::namedWindow("ex_result");
            cv::imshow("ex_result", src_clone);
        }
        else
        {
            if(-1 != cv::getWindowProperty("ex_result", 1))
                cv::destroyWindow("ex_result");
        }
#endif // DEBUG_MODE

        if(is_small)
            angle_slover.setTargetSize(12.4, 5.4);
        else
            angle_slover.setTargetSize(21.6, 5.4);

        angle_slover.getAngle(armor_rect, main_setting->camera_param.bullet_speed);

        if(!std::isnan(unpack_data->getStm2PcMesg()->stm32_info_data.bullet_spd) && unpack_data->getStm2PcMesg()->stm32_info_data.bullet_spd != 0)
        {
            double dist;
            angle_slover.getDistanceDanmu(armor_rect, dist);
            pit_final = -gc.solveAngleWithGravity(-angle_slover.getPitRef(), dist / 100, unpack_data->getStm2PcMesg()->stm32_info_data.bullet_spd);
#ifdef DEBUG_MODE
            std::cout << "distance:  " << dist << std::endl;
            std::cout << "speed:     " << unpack_data->getStm2PcMesg()->stm32_info_data.bullet_spd << std::endl;
#endif // DEBUG_MODE
        }
        else
        {
            pit_final = angle_slover.getPitRef();
#ifdef DEBUG_MODE
            std::cout << "can not get the bullet spd!!!" << std::endl;
#endif // DEBUG_MODE
        }

        yaw_final = -angle_slover.getYawRef();

#ifdef DEBUG_MODE
        std::cout << "pit_final: " << pit_final << std::endl;
        std::cout << "yaw_final: " << yaw_final << std::endl;
        std::cout << "===============================" << std::endl;
#endif // DEBUG_MODE

        pack_data->setPc2StmMesg()->gimbal_control_data.pit_ref = pit_final;
        pack_data->setPc2StmMesg()->gimbal_control_data.yaw_ref = yaw_final;
        pack_data->setPc2StmMesg()->gimbal_control_data.visual_valid = 1;
        pack_data->setPc2StmMesg()->gimbal_control_data.time = t1_param;
        pack_data->setPc2StmMesg()->gimbal_control_data.buff_shoot = 0;

        pit_last = pit_final;
        yaw_last = yaw_final;
    }
    else
    {
        flag = 0;

        pack_data->setPc2StmMesg()->gimbal_control_data.pit_ref = pit_last;
        pack_data->setPc2StmMesg()->gimbal_control_data.yaw_ref = yaw_last;
        pack_data->setPc2StmMesg()->gimbal_control_data.time = t1_param;
        pack_data->setPc2StmMesg()->gimbal_control_data.buff_shoot = 0;

        if(lost_flag < 4)
            pack_data->setPc2StmMesg()->gimbal_control_data.visual_valid = 1;
        else
        {
            pack_data->setPc2StmMesg()->gimbal_control_data.visual_valid = 0;
            pit_final = 0;
            yaw_final = 0;
        }

        lost_flag++;
    }

    // 6.串口发送
    pack_data->process(serial);
}

bool ArmorDetector::detectorProc(cv::Mat src, QuadrilateralPos &pos, cv::RotatedRect &rot_rect, bool &is_small)
{
    cv::Mat bin;
#ifdef DEBUG_MODE
    ret = src.clone();
    ret_2 = src.clone();
    ret_3 = src.clone();
#endif // DEBUG_MODE
    struct PreprocessInfo preprocess_info;
    if(this->main_setting->enemy_color == red)
        mulAndIterRedTre(src, bin, preprocess_info);
    else
        mulAndIterBlueTre(src, bin, preprocess_info);

#ifdef DEBUG_MODE
    src_cl = src.clone();
    if(this->main_setting->debug.b_show_bin)
    {
        cv::namedWindow("armor_bin");
        cv::imshow("armor_bin", bin);
    }
    else
    {
        if(-1 != cv::getWindowProperty("armor_bin", 1))
            cv::destroyWindow("armor_bin");
    }
#endif // DEBUG_MODE

    std::vector<std::vector<cv::Point>> light_contours;  // 灯条轮廓
    std::vector<cv::RotatedRect> rot_rect_list;          // 灯条矩形集合
    std::vector<cv::Rect> rect_list;                     // 灯条矩形集合
    std::vector<struct TargetSize> target_size_list;     // 目标尺寸集合(一一对应)
    std::vector<LightInfo> light_list;                   // 灯条集合

    // 1.识别敌方灯条
    if(!targetDetectionProc(bin, light_contours, rot_rect_list, rect_list, target_size_list))
        return false;
    filterEnemyLight(src, light_contours, rot_rect_list, rect_list, target_size_list, light_list);

#ifdef DEBUG_MODE
    if(main_setting->debug.b_show_target)
    {
        cv::namedWindow("enemy_det");
        cv::imshow("enemy_det",ret_2);
    }
    else
    {
        if(-1 != cv::getWindowProperty("enemy_det",1))
            cv::destroyWindow("enemy_det");
    }
#endif // DEBUG_MODE

    std::vector<ArmorInfo> armor_pre_list;
    std::vector<ArmorInfo> armor_all_list;
    static ArmorInfo armor_all_last;
    static int filter_num;

    // 2.识别敌方装甲板
    findArmorFrom2Light(light_list, armor_pre_list);
    filterAllArmor(armor_pre_list, armor_all_list);


#ifdef DEBUG_MODE
    if(main_setting->debug.b_show_armor)
    {
        cv::namedWindow("armor_det");;
        cv::imshow("armor_det", ret_3);
    }
    else
    {
        if(-1 != cv::getWindowProperty("armor_det", 1))
            cv::destroyWindow("armor_det");
    }
#endif // DEBUG_MODE

    // 3.筛选最优目标
    int s_max_idx=-1;

    if(filter_num > 0)
    {
        //当前帧所有候选目标与前一帧目标比较
        for(int i = 0; i < armor_all_list.size(); i++)
        {

            float &cur_x = armor_all_list[i].rrect.center.x;
            float &cur_y = armor_all_list[i].rrect.center.y;

            float &last_x = armor_all_last.rrect.center.x;
            float &last_y = armor_all_last.rrect.center.y;

            float dist = std::sqrt(std::pow(cur_x-last_x, 2) + std::pow(cur_y-last_y, 2));
            armor_all_list[i].s_list.push_back(-dist / 892. + 1);
        }
    }
    filter_num++;

    float s_max = -9999.f;

    armor_num = armor_all_list.size();
    std::vector<float>w_list = {3,5,10,3,1,9999};  // 高度，平行度, 长宽比，面积，跟上一帧距离（量纲不统一）
    for(int i = 0; i < armor_all_list.size(); i++)
    {
        float s = ArmorDetector::addByWeight(armor_all_list[i].s_list, w_list);
        if(s > s_max)
        {
            s_max_idx = i;
            s_max = s;
        }
    }

#ifdef DEBUG_MODE
    cv::Mat src_clone = src.clone();

    for(int i = 0; i < armor_all_list.size(); i++)
    {
        cv::Point2f vertex[4];
        armor_all_list[i].rrect.points(vertex);
        for(int j = 0; j < 4; j++)
            cv::line(src_clone, vertex[j], vertex[(j + 1) % 4], (i == s_max_idx)?cv::Scalar(0, 255, 0):cv::Scalar(0,0,255), 1);
    }
    if(main_setting->debug.b_show_result)
    {
        cv::namedWindow("result");
        cv::imshow("result", src_clone);
    }
    else
    {
        if(-1 != cv::getWindowProperty("result", 1))
            cv::destroyWindow("result");
    }
#endif // DEBUG_MODE

    if(armor_all_list.size() == 0)
        return false;

    armor_all_last = armor_all_list[s_max_idx];
    pos = armor_all_last.pos;
    rot_rect = armor_all_last.rrect;
    is_small = armor_all_list[s_max_idx].is_small;
    return true;
}


void ArmorDetector::filterAllArmor(std::vector<ArmorInfo> armor_list,
                                   std::vector<ArmorInfo> &armor_all_list)
{
    const double small_armor_wh_threshold = 2.45;

    #define SafeRect(rect, max_size) {if (makeRectSafe(rect, max_size) == false) continue;}
    //筛选所有待定的目标
    for(int i = 0; i < armor_list.size(); i++)
    {
        const cv::RotatedRect &rect = armor_list[i].rrect;

        //1.长宽比
        double w = std::max(rect.size.width, rect.size.height);
        double h = std::min(rect.size.width, rect.size.height);
        double wh_ratio = w / h;
        if (wh_ratio > armor_setting->armor_param.armor_hw_ratio_max * 0.1 || wh_ratio < armor_setting->armor_param.armor_hw_ratio_min * 0.1)
        {
#ifdef DEBUG_MODE
            std::stringstream s;
            s << wh_ratio;
            cv::putText(ret_3, "armor_wh_ratio: " + s.str(), cv::Point(rect.center) + cv::Point(0, 50), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 0, 255));
#endif
            continue;
        }
#ifdef DEBUG_MODE
        else
        {
            std::stringstream s;
            s << wh_ratio;
            cv::putText(ret_3, "armor_wh_ratio: " + s.str(), cv::Point(rect.center) + cv::Point(0, 50), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 255, 0));
        }
#endif
        bool &is_small = armor_list[i].is_small;

        if (wh_ratio < small_armor_wh_threshold)
            is_small = true;
        else
            is_small = false;

        armor_list[i].s_list.push_back(1.4 / wh_ratio);
        armor_list[i].s_list.push_back(armor_list[i].rrect.size.area() / 25000.);
        armor_all_list.push_back(armor_list[i]);
    }
}


void ArmorDetector::findArmorFrom2Light(std::vector<LightInfo> &light_list,
                                        std::vector<ArmorInfo> &armor_list)
{
    for(int i = 0; i < light_list.size(); i++)
    {
        const cv::RotatedRect &rect_i = light_list[i].rrect;
        const cv::Point &center_i = rect_i.center;
        float xi = center_i.x;
        float yi = center_i.y;

        for(int j = i + 1; j < light_list.size(); j++)
        {
            const cv::RotatedRect &rect_j = light_list[j].rrect;
            const cv::Point &center_j = rect_j.center;
            float xj = center_j.x;
            float yj = center_j.y;
            cv::RotatedRect rot_rect = boundingRRect(rect_i, rect_j);

#ifdef DEBUG_MODE
            QuadrilateralPos temp;
            rot_rect.points(temp.p);
            for(int k = 0; k < 4; k++)
                cv::line(ret_3, temp.p[k], temp.p[(k + 1) % 4], cv::Scalar(0, 0, 255), 1, 8);
#endif // DEBUG_MODE

            // 1.高度差
            float delta_height = GET_DIST(rect_i.size.height, rect_j.size.height);
            if(delta_height > armor_setting->armor_param.delta_height_max * 0.001)
            {
#ifdef DEBUG_MODE
                std::ostringstream s_delta_height;
                s_delta_height << delta_height;
                cv::putText(ret_3, "delta_height: " + s_delta_height.str(), cv::Point(rot_rect.center) + cv::Point(0, 10), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 0, 255), 1);
#endif // DEBUG_MODE
                continue;
            }
#ifdef DEBUG_MODE
            else{
                std::ostringstream s_delta_height;
                s_delta_height << delta_height;
                cv::putText(ret_3, "delta_height: " + s_delta_height.str(), cv::Point(rot_rect.center) + cv::Point(0, 10), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 255, 0), 1);
            }
#endif // DEBUG_MODE

            // 2.角度
            float delta_angle = abs(rect_j.angle - rect_i.angle);
            if(delta_angle == 180)
                delta_angle = 0;
            if(delta_angle > armor_setting->armor_param.delta_angle_max * 0.1)
            {
#ifdef DEBUG_MODE
                std::ostringstream s_j_angle, s_i_angle, s_delta_angle;
                s_j_angle << rect_j.angle;
                s_i_angle << rect_i.angle;
                s_delta_angle << delta_angle;
                cv::putText(ret_3, "rect_j_angle: " + s_j_angle.str(), cv::Point(rect_j.center) + cv::Point(0, 20), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 0, 255) ,1);
                cv::putText(ret_3, "rect_i_angle: " + s_i_angle.str(), cv::Point(rect_i.center) + cv::Point(0, 20), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 0, 255) ,1);
                cv::putText(ret_3, "delta_angle: " + s_delta_angle.str(), cv::Point(rot_rect.center) + cv::Point(0, 20), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 0, 255) ,1);
#endif // DEBUG_MODE
                continue;
            }
#ifdef DEBUG_MODE
            else
            {
                std::ostringstream s_j_angle, s_i_angle, s_delta_angle;
                s_j_angle << rect_j.angle;
                s_i_angle << rect_i.angle;
                s_delta_angle << delta_angle;
                cv::putText(ret_3, "rect_j_angle: " + s_j_angle.str(), cv::Point(rect_j.center) + cv::Point(0, 20), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 255, 0) ,1);
                cv::putText(ret_3, "rect_i_angle: " + s_i_angle.str(), cv::Point(rect_i.center) + cv::Point(0, 20), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 255, 0) ,1);
                cv::putText(ret_3, "delta_angle: " + s_delta_angle.str(), cv::Point(rot_rect.center) + cv::Point(0, 20), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 255, 0) ,1);
            }
#endif // DEBUG_MODE

            // 3.宽比高
            float delta_w = abs(xj - xi);
            float delta_w_ratio = delta_w * 1. / rect_j.size.height;
            if (delta_w_ratio < armor_setting->armor_param.delta_w_ratio_min * 0.1 || delta_w_ratio > armor_setting->armor_param.delta_w_ratio_max * 0.1)
            {
#ifdef DEBUG_MODE
                std::ostringstream s_delta_w_ratio;
                s_delta_w_ratio << delta_w_ratio;
                cv::putText(ret_3, "delta_width: " + s_delta_w_ratio.str(), cv::Point(rot_rect.center) + cv::Point(0, 30), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 0, 255), 1);
#endif // DEBUG_MODE
                continue;
            }
#ifdef DEBUG_MODE
            else
            {
                std::ostringstream s_delta_w_ratio;
                s_delta_w_ratio << delta_w_ratio;
                cv::putText(ret_3, "delta_width: " + s_delta_w_ratio.str(), cv::Point(rot_rect.center) + cv::Point(0, 30), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 255, 0), 1);
            }
#endif // DEBUG_MODE

            // 4.高度差比宽
            float delta_h = std::abs(yi - yj) / delta_w;
            if (delta_h > armor_setting->armor_param.delta_h_max * 0.001)
            {
#ifdef DEBUG_MODE
                std::ostringstream s_delta_h;
                s_delta_h << delta_h;
                cv::putText(ret_3, "delta_h: " + s_delta_h.str(), cv::Point(rot_rect.center) + cv::Point(0, 40), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 0, 255), 1);
#endif // DEBUG_MODE
                continue;
            }
#ifdef DEBUG_MODE
            else
            {
                std::ostringstream s_delta_h;
                s_delta_h << delta_h;
                cv::putText(ret_3, "delta_h: " + s_delta_h.str(), cv::Point(rot_rect.center) + cv::Point(0, 40), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 255, 0), 1);
            }
#endif // DEBUG_MODE
            cv::Point2f p_i[4], p_j[4];
            rect_i.points(p_i);
            rect_j.points(p_j);

            ArmorInfo armor;

            cv::Point2f *p = armor.pos.p;
            std::vector<cv::Point2f> p_list = {
                (p_i[1] + p_i[2]) / 2,
                (p_i[3] + p_i[0]) / 2,
                (p_j[1] + p_j[2]) / 2,
                (p_j[3] + p_j[0]) / 2
            };

            //装甲板矩形角点排序
            std::sort(p_list.begin(), p_list.end(), [](cv::Point2f &p1, cv::Point2f &p2)->bool {return p1.x < p2.x; });

            if(p_list[0].y > p_list[1].y)
            {
                cv::Point2f point_temp = p_list[0];
                p_list[0] = p_list[1];
                p_list[1] = point_temp;
            }
            if(p_list[2].y < p_list[3].y)
            {
                cv::Point2f point_temp = p_list[2];
                p_list[2] = p_list[3];
                p_list[3] = point_temp;
            }

            for(int i = 0; i < 4; i++)
                p[i] = p_list[i];

            armor.rrect = rot_rect;
            armor.s_list={1 - delta_height, (armor_setting->armor_param.delta_angle_max - delta_angle) / armor_setting->armor_param.delta_angle_max, -delta_h / 1.5 + 1};
            armor_list.push_back(armor);

#ifdef DEBUG_MODE
            for(int k = 0; k < 4; k++)
                cv::line(ret_3, armor.pos.p[k], armor.pos.p[(k+1) % 4], cv::Scalar(0, 255, 0), 3, 8);
#endif // DEBUG_MODE
        }
    }
}

cv::RotatedRect ArmorDetector::boundingRRect(const cv::RotatedRect &left, const cv::RotatedRect &right)
{
    const cv::Point &pl = left.center, &pr = right.center;
    cv::Point2f center = (pl + pr) / 2.0;
    cv::Size2f wh_l = left.size;
    cv::Size2f wh_r = right.size;
    float width = POINT_DIST(pl, pr) - (wh_l.width + wh_r.width) / 2.0;
    float height = std::max(wh_l.height, wh_r.height);
    float angle = std::atan2(right.center.y - left.center.y, right.center.x - left.center.x);
    return cv::RotatedRect(center, cv::Size2f(width, height), angle * 180 / CV_PI);
}

void ArmorDetector::filterEnemyLight(cv::Mat src,
                                     std::vector<std::vector<cv::Point>> light_contours,
                                     std::vector<cv::RotatedRect> rot_rect_list,
                                     std::vector<cv::Rect> rect_list,
                                     std::vector<struct TargetSize> target_size_list,
                                     std::vector<LightInfo> &light_info)
{
    for(int i = 0; i < light_contours.size(); i++)
    {
        LightInfo light_info_i;
        cv::RotatedRect rot_rect = adjustRect(rot_rect_list[i]);

        // 1.角度
        double angle = rot_rect.angle;
        angle = 90 - angle;
        angle = angle < 0.0 ? angle + 180 : angle;
        float delta_angle = abs(angle - 90);
        if(delta_angle > armor_setting->tgt_size_param.slope_offset)
        {
#ifdef DEBUG_MODE
            std::ostringstream s_slope_offset;
            s_slope_offset << delta_angle;
            cv::putText(ret_2, "slope_offset: " + s_slope_offset.str(), cv::Point(rot_rect.center) + cv::Point(0, 10), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 0, 255), 1);
#endif // DEBUG_MODE
            continue;
        }
#ifdef DEBUG_MODE
        else
        {
            std::ostringstream s_slope_offset;
            s_slope_offset << delta_angle;
            cv::putText(ret_2, "slope_offset: " + s_slope_offset.str(), cv::Point(rot_rect.center) + cv::Point(0, 10), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 255, 0), 1);
        }
#endif // DEBUG_MODE

        // 2.颜色
        cv::Rect rect = rect_list[i];
        static cv::Point ex_ratio_point = 0.5 * cv::Point(rect.width, rect.height);
        cv::Rect rect_ex(rect.tl() - ex_ratio_point, rect.br() + ex_ratio_point);
        makeRectSafe(rect_ex, cv::Size(SHOW_WIDTH / SHOW_RADIO, SHOW_HEIGHT / SHOW_RADIO));
        cv::Mat roi_not_continue = src(rect_ex);
        cv::Mat ch[3];
        cv::Mat roi = roi_not_continue.clone();
        cv::split(roi, ch);
        int diff = 0;
        for(int i = 0; i < roi.rows; i++)
        {
            uchar *p_b = ch[0].ptr<uchar>(i);
            uchar *p_r = ch[2].ptr<uchar>(i);
            for(int j = 0; j < roi.cols; j++)
            {
                diff += *(p_b + j) - *(p_r + j);
            }
        }

        float avg_diff = diff * 1. / target_size_list[i].area;
        if(std::abs(avg_diff) < armor_setting->tgt_size_param.color_th || ((this->main_setting->enemy_color == red && avg_diff > 0) || (this->main_setting->enemy_color == blue && avg_diff < 0)))
        {
#ifdef DEBUG_MODE
            std::ostringstream s;
            s << avg_diff;
            cv::putText(ret_2, "color_th: " + s.str(), cv::Point(rot_rect.center)+cv::Point(0, 20), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 0, 255), 1);
            cv::rectangle(ret_2, rect_ex, cv::Scalar(0,0,255), 1);
#endif
            continue;
        }
#ifdef DEBUG_MODE
        else
        {
            std::ostringstream s;
            s << avg_diff;
            cv::putText(ret_2, "color_th: " + s.str(), cv::Point(rot_rect.center)+cv::Point(0, 20), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 255, 0), 1);
            cv::rectangle(ret_2, rect_ex, cv::Scalar(0,255,0), 1);
        }
#endif
        light_info_i.idx = i;
        light_info_i.contours.assign(light_contours[i].begin(), light_contours[i].end());
        light_info_i.size = target_size_list[i];
        light_info_i.rrect = rot_rect;
        light_info_i.rect = rect_list[i];
        light_info.push_back(light_info_i);
    }
}

bool ArmorDetector::makeRectSafe(cv::Rect & rect, cv::Size size)
{
    if (rect.x < 0)
        rect.x = 0;
    if (rect.x + rect.width > size.width)
        rect.width = size.width - rect.x;
    if (rect.y < 0)
        rect.y = 0;
    if (rect.y + rect.height > size.height)
        rect.height = size.height - rect.y;
    if (rect.width <= 0 || rect.height <= 0)
        return false;
    return true;
}

float ArmorDetector::addByWeight(std::vector<float> s, std::vector<float> w)
{
    float sum = 0;
    for(int i = 0; i < s.size(); i++)
    {
        sum += s[i] * w[i];
    }
    return sum;
}

