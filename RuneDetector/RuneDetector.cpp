#include "RuneDetector/RuneDetector.h"

RuneHitLogic::RuneHitLogic(RuneSettings *rune_settings, MainSettings *main_settings, bool is_clockwise):
                           RuneDetector(rune_settings, main_settings, is_clockwise)
{
    T = (rune_settings->rune_param.rune_time_delay.TRIG2SHOOT2HIT_DELAY + rune_settings->rune_param.rune_time_delay.PTZ_SHIFT_DELAY) * 0.01;
    frame_count = 0;
    shoot_count = 0;
    state_change = start_state;

    amplitude = 0.785;
    angular_frequency = 1.884;
    initial_phase = 0;
    const_number = 1.305;
}

RuneDetector::RuneDetector(RuneSettings *rune_settings, MainSettings *main_settings, bool is_clockwise):
                           DetectRuneCenter(rune_settings, main_settings),
                           prep_activate_detection(rune_settings->rune_param.tgt_size_param_u, main_settings->debug),
                           prep_activate_adaptive_threshold(rune_settings->preprocess_param)
{
    this->is_clockwise = is_clockwise;
}

DetectRuneCenter::DetectRuneCenter(RuneSettings *rune_settings, MainSettings *main_settings):
                                   center_detection(rune_settings->rune_param.tgt_size_param_c, main_settings->debug),
                                   center_adaptive_threshold(rune_settings->preprocess_param)
{
    this->rune_settings = rune_settings;
    this->main_settings = main_settings;
}

cv::Point2f DetectRuneCenter::getRuneCenter(void)
{
    return this->rune_center;
}

float RuneHitLogic::getPitAbsAngle()
{
    return pit_abs_angle;
}

float RuneHitLogic::getYawAbsAngle()
{
    return yaw_abs_angle;
}

bool RuneHitLogic::getRotation()
{
    return is_clockwise;
}

bool DetectRuneCenter::setRuneCenter(cv::Mat src, cv::RotatedRect rune_armor_corner)
{
    cv::Mat dst;
    std::vector<std::vector<cv::Point>>contours;
    std::vector<TargetSize> target_size;

    redOrBlueTre(src, dst, center_adaptive_threshold);
    center_adaptive_threshold.filter(dst);

#ifdef DEBUG_MODE
    cv::Mat src_clone = src.clone();
    center_detection.ret_4 = src.clone();
#endif // DEBUG_MODE

    center_detection.setRuneTargetSize(rune_settings->rune_param.tgt_size_param_c);
    center_detection.targetDetectionProc(dst, contours, target_size);
    center_detection.setRuneTargetSize(rune_settings->rune_param.tgt_size_param_u);

#ifdef DEBUG_MODE
    if(main_settings->debug.b_show_center_rune)
    {
        cv::namedWindow("debug_center_rune");
        cv::imshow("debug_center_rune", center_detection.ret_4);
    }
    else
    {
        if(-1 != cv::getWindowProperty("debug_center_rune", 1))
            cv::destroyWindow("debug_center_rune");
    }
#endif // DEBUG_MODE

    for(int i = 0; i < center_detection.getTargetRotRect().size(); i++)
    {
#ifdef DEBUG_MODE
        cv::circle(src_clone, cv::Point(center_detection.getTargetRotRect()[i].center), 1, cv::Scalar(0, 255, 0), 2);
        cv::circle(src_clone, cv::Point(rune_armor_corner.center), 1, cv::Scalar(0, 255, 0), 2);
#endif // DEBUG_MODE
        int delta_x = rune_armor_corner.center.x - center_detection.getTargetRotRect()[i].center.x;
        int delta_y = rune_armor_corner.center.y - center_detection.getTargetRotRect()[i].center.y;
#ifdef DEBUG_MODE
        int add_x = rune_armor_corner.center.x + center_detection.getTargetRotRect()[i].center.x;
        int add_y = rune_armor_corner.center.y + center_detection.getTargetRotRect()[i].center.y;
#endif // DEBUG_MODE
        int center_distance = abs(sqrt(delta_x * delta_x + delta_y * delta_y));

        if(center_distance < rune_settings->rune_param.tgt_size_param_c.center_distance_min || center_distance > rune_settings->rune_param.tgt_size_param_c.center_distance_max)
        {
#ifdef DEBUG_MODE
        std::ostringstream s_center_distance;
        s_center_distance << center_distance;
        cv::putText(src_clone, "distance: " + s_center_distance.str(), cv::Point(add_x/2, add_y/2) + cv::Point(0, 10), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 0, 255));
        cv::line(src_clone, rune_armor_corner.center, center_detection.getTargetRotRect()[i].center, cv::Scalar(0, 0, 255));
#endif // DEBUG_MODE
        continue;
        }
#ifdef DEBUG_MODE
        std::ostringstream s_center_distance;
        s_center_distance << center_distance;
        cv::putText(src_clone, "distance: " + s_center_distance.str(), cv::Point(add_x/2, add_y/2) + cv::Point(0, 10), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 255, 0));
        cv::line(src_clone, rune_armor_corner.center, center_detection.getTargetRotRect()[i].center, cv::Scalar(0, 255, 0));
#endif // DEBUG_MODE
        rune_center = center_detection.getTargetRotRect()[i].center;
    }
#ifdef DEBUG_MODE
    if(main_settings->debug.b_show_rune_center)
    {
        cv::namedWindow("debug_rune_center_distance");
        cv::imshow("debug_rune_center_distance", src_clone);
    }
    else
    {
        if(-1!=cv::getWindowProperty("debug_rune_center_distance", 1))
            cv::destroyWindow("debug_rune_center_distance");
    }
#endif // DEBUG_MODE
    return rune_center.x;  // ???????????????????????????false
}


void DetectRuneCenter::redOrBlueTre(cv::Mat src, cv::Mat &dst, AdaptiveThreshold &threshold)
{
    if(this->main_settings->enemy_color == red)
        threshold.adaptiveThresholdProc(src, dst, 0); //0 red, 1 bule, 2 red?????????, 3 blue?????????
    else
        threshold.adaptiveThresholdProc(src, dst, 1);
}

std::vector<cv::RotatedRect> RuneDetector::getPrepTargetRotRect()
{
    return this->prep_activate_detection.getTargetRotRect();
}

bool RuneDetector::getCurPos(cv::Mat src, cv::RotatedRect &rot_rect, QuadrilateralPos &cur_pos)
{
    bool is_find_curpos = false;
    bool is_find_curpos_too_more = false;
    cv::Mat dst;
    cv::RotatedRect rot_rect_temp;
    QuadrilateralPos cur_pos_temp;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<TargetSize> target_size;
    redOrBlueTre(src, dst, prep_activate_adaptive_threshold);
    prep_activate_adaptive_threshold.filter(dst);

#ifdef DEBUG_MODE
    cv::Mat src_clone = src.clone();
    cv::Mat rune_cp_result = src.clone();
    if(main_settings->debug.b_show_bin_rune)
    {
        cv::namedWindow("bin");
        cv::imshow("bin", dst);
    }
    else
    {
        if(-1 != cv::getWindowProperty("bin",1))
            cv::destroyWindow("bin");
    }
#endif // DEBUG_MODE

    prep_activate_detection.targetDetectionProc(dst, contours, target_size);

    if(prep_activate_detection.getTargetRotRect().size() == 1)
    {
        if(prep_activate_detection.getTargetHierarchy()[0][3] != -1)
        {
            // 1.???????????????
            int parent_area = cv::contourArea(prep_activate_detection.getContours()[prep_activate_detection.getTargetHierarchy()[0][3]]);
            if(parent_area > rune_settings->rune_param.tgt_size_param_u.parent_area_min && parent_area < rune_settings->rune_param.tgt_size_param_u.parent_area_max)
            {
#ifdef DEBUG_MODE
                std::ostringstream s_parent_area;
                s_parent_area << parent_area;
                cv::putText(src_clone, "parent_area: " + s_parent_area.str(), cv::Point(prep_activate_detection.getTargetRotRect()[0].center) + cv::Point(0, 10), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 255, 0), 1, 8);
#endif // DEBUG_MODE

                // 2.??????????????????
                std::vector<cv::Point> parent_contour_poly;
                cv::approxPolyDP(prep_activate_detection.getContours()[prep_activate_detection.getTargetHierarchy()[0][3]], parent_contour_poly, 4, true);
                int parent_poly_size = parent_contour_poly.size();
                if(parent_poly_size == 7 || parent_poly_size == 8 || parent_poly_size == 9 || parent_poly_size == 10)
                {
#ifdef DEBUG_MODE
                    for(int i = 0; i < parent_poly_size; i++)
                        cv::line(src_clone, parent_contour_poly[i], parent_contour_poly[(i + 1) % parent_poly_size], cv::Scalar(0, 255, 0), 1, 8);
#endif // DEBUG_MODE

                    // 3,??????????????????????????????
                    cv::RotatedRect parent_rot_rect = prep_activate_detection.adjustRect(cv::fitEllipse(prep_activate_detection.getContours()[prep_activate_detection.getTargetHierarchy()[0][3]]));
                    float parent_hw_ratio = parent_rot_rect.size.height * 1.0 / parent_rot_rect.size.width;
                    if(parent_hw_ratio > rune_settings->rune_param.tgt_size_param_u.parent_hw_ratio_min * 0.01 && parent_hw_ratio < rune_settings->rune_param.tgt_size_param_u.parent_hw_ratio_max * 0.01)
                    {
#ifdef DEBUG_MODE
                        QuadrilateralPos parent_rot_rect_points;
                        parent_rot_rect.points(parent_rot_rect_points.p);
                        for(int i = 0; i < 4; i++)
                            cv::line(src_clone, parent_rot_rect_points.p[i], parent_rot_rect_points.p[(i + 1) % 4], cv::Scalar(0, 255, 0), 1, 8);
                        std::ostringstream s_parent_hw_ratio;
                        s_parent_hw_ratio << parent_hw_ratio;
                        cv::putText(src_clone, "hw_ratio: " + s_parent_hw_ratio.str(), cv::Point(prep_activate_detection.getTargetRotRect()[0].center) + cv::Point(0, 20), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 255, 0), 1, 8);
#endif // DEBUG_MODE
                        std::vector<cv::Point> cur_pos_contours_poly;
                        cv::approxPolyDP(contours[0], cur_pos_contours_poly, 4, true);
                        prep_activate_detection.getTargetRotRect()[0].points(cur_pos_temp.p);
                        rot_rect_temp = prep_activate_detection.getTargetRotRect()[0];
                        is_find_curpos = true;
#ifdef DEBUG_MODE
                        std::cout << "pre rune find success!" << std::endl;
                        for(int i = 0; i < 4; i++)
                            cv::line(rune_cp_result, cur_pos_temp.p[i], cur_pos_temp.p[(i + 1) % 4], cv::Scalar(0, 255, 0), 2, 8);
#endif // DEBUG_MODE
                    }
#ifdef DEBUG_MODE
                    else // parent_hw_ratio
                    {
                        std::cout << "parent_hw_ratio is wrong!" << std::endl;
                        std::ostringstream s_parent_hw_ratio;
                        s_parent_hw_ratio << parent_hw_ratio;
                        cv::putText(src_clone, "hw_ratio: " + s_parent_hw_ratio.str(), cv::Point(prep_activate_detection.getTargetRotRect()[0].center) + cv::Point(0, 20), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 0, 255), 1, 8);
                    }
#endif // DEBUG_MODE
                }
#ifdef DEBUG_MODE
                else // parent_poly_size
                {
                    std::cout << "parent_poly_size is wrong!" << std::endl;
                    for(int i = 0; i < parent_poly_size; i++)
                        cv::line(src_clone, parent_contour_poly[i], parent_contour_poly[(i + 1) % parent_poly_size], cv::Scalar(0, 0, 255), 2, 8);
                }
#endif // DEBUG_MODE
            }
#ifdef DEBUG_MODE
            else // parent_area
            {
                std::cout << "parent_area is wrong!" << std::endl;
                std::ostringstream s_parent_area;
                s_parent_area << parent_area;
                cv::putText(src_clone, "parent_area: " + s_parent_area.str(), cv::Point(prep_activate_detection.getTargetRotRect()[0].center) + cv::Point(0, 10), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 255, 0), 1, 8);
            }
#endif // DEBUG_MODE
        }
#ifdef DEBUG_MODE
        else // hierarchy[0][3]
        {
            std::cout << "No parent profile" << std::endl;
        }
#endif // DEBUG_MODE
    }
    else if(prep_activate_detection.getTargetRotRect().size() > 1) // targetRotRect.size()
    {
        int prep_activate_count = 0;
        std::vector<std::vector<cv::Point>> parent_contours_poly(prep_activate_detection.getTargetRotRect().size());
        for(int i = 0; i < prep_activate_detection.getTargetRotRect().size(); i++)
        {
            if(prep_activate_detection.getTargetHierarchy()[i][3] != -1)
            {
                // 1.???????????????
                int parent_area = cv::contourArea(prep_activate_detection.getContours()[prep_activate_detection.getTargetHierarchy()[i][3]]);
                if(parent_area > rune_settings->rune_param.tgt_size_param_u.parent_area_min && parent_area < rune_settings->rune_param.tgt_size_param_u.parent_area_max)
                {
#ifdef DEBUG_MODE
                    std::ostringstream s_parent_area;
                    s_parent_area << parent_area;
                    cv::putText(src_clone, "parent_area: " + s_parent_area.str(), cv::Point(prep_activate_detection.getTargetRotRect()[i].center) + cv::Point(0, 10), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 255, 0), 1, 8);
#endif // DEBUG_MODE

                    // 2.??????????????????
                    cv::approxPolyDP(prep_activate_detection.getContours()[prep_activate_detection.getTargetHierarchy()[i][3]], parent_contours_poly[i], 4, true);
                    int parent_poly_size = parent_contours_poly[i].size();
                    if(parent_poly_size == 7 || parent_poly_size == 8 || parent_poly_size == 9 || parent_poly_size == 10)
                    {
#ifdef DEBUG_MODE
                        for(int j = 0; j < parent_poly_size; j++)
                            cv::line(src_clone, parent_contours_poly[i][j], parent_contours_poly[i][(j + 1) % parent_poly_size], cv::Scalar(0, 255, 0), 1, 8);
#endif // DEBUG_MODE
                        // 3.??????????????????????????????
                        cv::RotatedRect parent_rot_rect = prep_activate_detection.adjustRect(cv::fitEllipse(prep_activate_detection.getContours()[prep_activate_detection.getTargetHierarchy()[i][3]]));
                        float parent_hw_ratio = parent_rot_rect.size.height * 1.0 / parent_rot_rect.size.width;
                        if(parent_hw_ratio > rune_settings->rune_param.tgt_size_param_u.parent_hw_ratio_min * 0.01 && parent_hw_ratio < rune_settings->rune_param.tgt_size_param_u.parent_hw_ratio_max * 0.01)
                        {
#ifdef DEBUG_MODE
                            QuadrilateralPos parent_rot_rect_points;
                            parent_rot_rect.points(parent_rot_rect_points.p);
                            for(int j = 0; j < 4; j++)
                                cv::line(src_clone, parent_rot_rect_points.p[j], parent_rot_rect_points.p[(j + 1) % 4], cv::Scalar(0, 255, 0), 1, 8);
                            std::ostringstream s_parent_hw_ratio;
                            s_parent_hw_ratio << parent_hw_ratio;
                            cv::putText(src_clone, "hw_ratio: " + s_parent_hw_ratio.str(), cv::Point(prep_activate_detection.getTargetRotRect()[i].center) + cv::Point(0, 20), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 255, 0), 1, 8);
#endif // DEBUG_MODE
                            std::vector<cv::Point> cur_pos_contours_poly;
                            cv::approxPolyDP(contours[i], cur_pos_contours_poly, 4, true);
                            prep_activate_detection.getTargetRotRect()[i].points(cur_pos_temp.p);
                            rot_rect_temp = prep_activate_detection.getTargetRotRect()[i];
                            prep_activate_count++;
                            is_find_curpos = true;
#ifdef DEBUG_MODE
                            std::cout << "find success!" << std::endl;
                            for(int j = 0; j < 4; j++)
                                cv::line(rune_cp_result, cur_pos_temp.p[j], cur_pos_temp.p[(j + 1) % 4], cv::Scalar(0, 255, 0), 2, 8);
#endif // DEBUG_MODE
                        }
#ifdef DEBUG_MODE
                        else // parent_hw_ratio
                        {
                            std::cout << "parent_hw_ratio is wrong!" << std::endl;
                            std::ostringstream s_parent_hw_ratio;
                            s_parent_hw_ratio << parent_hw_ratio;
                            cv::putText(src_clone, "hw_ratio: " + s_parent_hw_ratio.str(), cv::Point(prep_activate_detection.getTargetRotRect()[i].center) + cv::Point(0, 20), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 0, 255), 1, 8);
                        }
#endif // DEBUG_MODE
                    }
#ifdef DEBUG_MODE
                    else // parent_poly_size
                    {
                        for(int j = 0; j < parent_poly_size; j++)
                            cv::line(src_clone, parent_contours_poly[i][j], parent_contours_poly[i][(j + 1) % parent_poly_size], cv::Scalar(0, 0, 255), 2, 8);
                        std::cout << "parent_poly_size is wrong!" << std::endl;
                    }
#endif // DEBUG_MODE
                }
#ifdef DEBUG_MODE
                else // parent_area
                {
                    std::cout << "parent_area is wrong!" << std::endl;
                    std::ostringstream s_parent_area;
                    s_parent_area << parent_area;
                    cv::putText(src_clone, "parent_area: " + s_parent_area.str(), cv::Point(prep_activate_detection.getTargetRotRect()[i].center) + cv::Point(0, 10), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(0, 0, 255), 1, 8);
                }
#endif // DEBUG_MODE
            }
#ifdef DEBUG_MODE
            else // parent_contours
            {
                std::cout << "No parent profile" << std::endl;
            }
#endif // DEBUG_MODE
        }
        if (prep_activate_count > 1)
            is_find_curpos_too_more = 1;
    }

#ifdef DEBUG_MODE
    if(main_settings->debug.b_show_cp_rune)
    {
        cv::namedWindow("debug_cp_rune");
        cv::imshow("debug_cp_rune", src_clone);
    }
    else
    {
        if(-1 != cv::getWindowProperty("debug_cp_rune", 1))
            cv::destroyWindow("debug_cp_rune");
    }
    if(main_settings->debug.b_show_cp_rune_result)
    {
        cv::namedWindow("cp_rune_result");
        cv::imshow("cp_rune_result", rune_cp_result);
    }
    else
    {
        if(-1 != cv::getWindowProperty("cp_rune_result", 1))
            cv::destroyWindow("cp_rune_result");
    }
#endif // DEBUG_MODE

    if (!is_find_curpos || is_find_curpos_too_more)
        return false;
    if (is_find_curpos)
    {
        rot_rect = rot_rect_temp;
        cur_pos = cur_pos_temp;
    }
    return is_find_curpos;
}

void RuneHitLogic::setAngleRecent(cv::Point2f pos)
{
    angle_recent = atan2((pos.y - getRuneCenter().y), (pos.x - getRuneCenter().x));
    // ???x??????????????????????????????????????????????????????angle_recent_2PI
    if(angle_recent > 0 && angle_recent < PI)
        angle_recent_2PI = 2 * PI - angle_recent;
    else
        angle_recent_2PI = -angle_recent;
}

double RuneHitLogic::getAngleDiffForBigRuneInit()
{
    return std::abs(angle_recent_2PI - angle_old_for_bigRune);
}

//???????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
double RuneHitLogic::calStartTimeInPeriod(float angle_diff_of_interval, double time_interval)
{
    double temp_formulation_1 = angular_frequency * (angle_diff_of_interval - const_number * time_interval) / (2 * amplitude);
    double temp_formulation_2 = sin(angular_frequency * time_interval / 2);
    double time_begin = (asin(temp_formulation_1 / temp_formulation_2) - initial_phase) / angular_frequency - time_interval / 2;
    return time_begin;
}

//???????????????t2_difftime_for_period????????????????????????????????????????????????????????????????????????
double RuneHitLogic::predictAngleForSin(void)
{
    gettimeofday(&t2_difftime_for_period, NULL); //????????????
    double diff_time = (long)(1000000 * (t2_difftime_for_period.tv_sec - t1_difftime_for_period.tv_sec) + t2_difftime_for_period.tv_usec - t1_difftime_for_period.tv_usec) / 1000000.0;
    double time_begin_new = time_begin_in_period + diff_time; //time_begin_in_period????????????????????????????????????????????????????????????????????????t2_difftime_for_period?????????????????????

    double rotate_angle = calPredictAngleByPeriod(time_begin_new, T); //???????????????????????????T??????????????????????????????
    return rotate_angle;
}

//??????????????????????????????????????????????????????????????????????????????????????????
double RuneHitLogic::calPredictAngleByPeriod(double time_begin, double time_interval)
{
    float angular_frequency_half = angular_frequency / 2;
    //??????????????????????????????
    double rotate_angle = amplitude / angular_frequency_half * sin( angular_frequency_half * (2 * time_begin + time_interval) + initial_phase )
                          * sin( angular_frequency_half * time_interval ) + const_number * time_interval;
    return rotate_angle;
}

void RuneHitLogic::predictShiftProcWithCompansate(cv::RotatedRect &rot_rect, AngleSolver &angle_solver,
                                                  PackData *pack_data, UnpackData *unpack_data, Serial *serial, long t1_param,
                                                  float rotated_angle, GravityCompensateResolve &gc)
{
    angle_solver.getAngleWithRectify(rot_rect, RUNE_ARMOR_WD_RADIO);
    pit_abs_angle -= angle_solver.getPitRef();
    yaw_abs_angle += angle_solver.getYawRef();

#ifdef DEBUG_MODE
    std::cout << "pit_abs_angle_before_grav: " << pit_abs_angle << std::endl;
#endif // DEBUG_MODE
    //??????????????????????????????????????????
    predictAbsAngle(pit_abs_angle, yaw_abs_angle, rotated_angle);

    CompensateAngle(unpack_data, pit_abs_angle, yaw_abs_angle);
    setPitAbsAngleWithGavityCompansate(gc);

#ifdef DEBUG_MODE
    std::cout << "pit_abs_angle_after_grav: " << pit_abs_angle << std::endl;
#endif // DEBUG_MODE

#ifdef USE_SERIAL
    //6.????????????
    sendShiftMsg2Stm(pack_data, serial, t1_param);
#endif // USE_SERIAL
}

void RuneHitLogic::normalShiftProcWithCompansate(cv::RotatedRect &rot_rect, AngleSolver &angle_solver,
                                                 PackData *pack_data, UnpackData *unpack_data, Serial *serial, long t1_param,
                                                 GravityCompensateResolve &gc)
{
    angle_solver.getAngleWithRectify(rot_rect, RUNE_ARMOR_WD_RADIO);

    pit_abs_angle -= angle_solver.getPitRef();
    yaw_abs_angle += angle_solver.getYawRef();

#ifdef DEBUG_MODE
    std::cout << "pit_abs_angle_before_grav: " << pit_abs_angle << std::endl;
#endif // DEBUG_MODE

   CompensateAngle(unpack_data, pit_abs_angle, yaw_abs_angle);
   setPitAbsAngleWithGavityCompansate(gc);

#ifdef DEBUG_MODE
    std::cout << "pit_abs_angle_after_grav: " << pit_abs_angle << std::endl;
#endif // DEBUG_MODE

#ifdef USE_SERIAL
    //6.????????????
    sendShiftMsg2Stm(pack_data, serial, t1_param);
#endif // USE_SERIAL
}

//??????????????????????????????????????????????????????????????????????????????rotated_angle????????????
void RuneHitLogic::predictAbsAngle(float &pit_abs_angle, float &yaw_abs_angle, float rotated_angle)
{
    cur_pos_point_3d = calCurPosCoordinate(pit_abs_angle, yaw_abs_angle);

#ifdef DEBUG_MODE
    std::cout << "????????????pitch?????????: " << pit_abs_angle << std::endl;
    std::cout << "????????????yaw?????????: " << yaw_abs_angle << std::endl;
#endif // DEBUG_MODE

    getPrePosPoint(cur_pos_point_3d, rotated_angle);

    calCoorinateAbsAngle(pre_pos_point_3d, pit_abs_angle, yaw_abs_angle);

#ifdef DEBUG_MODE
    std::cout << "?????????pitch?????????: " << pit_abs_angle << std::endl;
    std::cout << "?????????yaw?????????: " << yaw_abs_angle << std::endl;
#endif // DEBUG_MODE
}

void RuneHitLogic::judgeHitOrShift(cv::Mat src, AngleSolver &angle_solver, PackData *pack_data, UnpackData *unpack_data,
                                   Serial *serial, long t1_param, GravityCompensateResolve &gc)
{
    //?????????????????????????????????
    bool is_find_rune = getCurPos(src, rot_rect, cur_pos);
    if (is_find_rune) //?????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
    {
        angle_solver.getAngleWithRectify(rot_rect, RUNE_ARMOR_WD_RADIO);

        float pit_ref_send = angle_solver.getPitRef();
        float yaw_ref_send = angle_solver.getYawRef();

        if (std::abs(pit_ref_send - pit_gavity_compansate_angle - pit_compansate_angle) < 0.18f
                && std::abs(yaw_ref_send - yaw_compansate_angle) < 0.15f) //?????????????????????????????????????????????????????????????????????
        {
            sendHitMsg2Stm(pack_data, serial, t1_param);
            state_change = shoot_state;
#ifdef DEBUG_MODE
            std::cout << "entry angle 0 to hit and print state_change: " << state_change << std::endl;
#endif // DEBUG_MODE
        }
        else
        {
            pit_abs_angle -= pit_ref_send;
            yaw_abs_angle += yaw_ref_send;

#ifdef DEBUG_MODE
            std::cout << "pitch_angle before gavity: " << pit_abs_angle << std::endl;
#endif // DEBUG_MODE
            setPitAbsAngleWithGavityCompansate(gc);
            CompensateAngle(unpack_data, pit_abs_angle, yaw_abs_angle);

#ifdef DEBUG_MODE
            std::cout << "pitch_angle after gavity: " << pit_abs_angle << std::endl;
            std::cout << "entry angle change to hit and print state_change: " << state_change << std::endl;
            std::cout << "pit_offset_angle_with_compansate: " << pit_ref_send << std::endl;
            std::cout << "yaw_offset_angle_with_compansate: " << yaw_ref_send << std::endl;
#endif // DEBUG_MODE

#ifdef USE_SERIAL
            //6.????????????
            sendShiftMsg2Stm(pack_data, serial, t1_param);
#endif // USE_SERIAL

            state_change = shift_state;
            gettimeofday(&t1_difftime, NULL); //??????????????????
        }
    }
    else
    {
        sendStopMsg2Stm(pack_data, serial, t1_param);
#ifdef DEBUG_MODE
        std::cout << "not find rune!" << std::endl;
#endif // DEBUG_MODE
    }
}


//rotated_angle????????????
void RuneHitLogic::getPrePosPoint(cv::Point3f cur_pos_point, float rotated_angle)
{
    float x, y;

    //?????????????????????
    x = cur_pos_point.x - center_point_3d.x;
    y = cur_pos_point.y - center_point_3d.y;

    if (is_clockwise)
    {
        //?????????
        pre_pos_point_3d.x = x * cos(rotated_angle) + y * sin(rotated_angle);
        pre_pos_point_3d.y = y * cos(rotated_angle) - x * sin(rotated_angle);
    }
    else
    {
        //?????????
        pre_pos_point_3d.x = x * cos(rotated_angle) - y * sin(rotated_angle);
        pre_pos_point_3d.y = y * cos(rotated_angle) + x * sin(rotated_angle);
    }

    pre_pos_point_3d.x = pre_pos_point_3d.x + center_point_3d.x;
    pre_pos_point_3d.y = pre_pos_point_3d.y + center_point_3d.y;
    pre_pos_point_3d.z = cur_pos_point.z;

//#ifdef DEBUG_MODE
//    cv::Mat draw = cv::Mat::zeros(cv::Size(760, 480), CV_8UC3);
//    cv::circle(draw, Point(center_point_3d.x, center_point_3d.y) + Point(200, 200), 1, cv::Scalar(0, 255, 0), 2);
//    cv::circle(draw, Point(cur_pos_point.x, cur_pos_point.y) + Point(200, 200), 1, cv::Scalar(0, 255, 0), 2);
//    cv::line(draw, Point(center_point_3d.x + 200, center_point_3d.y + 200), Point(cur_pos_point.x + 200, cur_pos_point.y + 200), cv::Scalar(0, 255, 0), 1);
//    cv::circle(draw, Point(pre_pos_point_3d.x, pre_pos_point_3d.y) + Point(200, 200), 1, cv::Scalar(0, 0, 255), 2);
//    cv::imshow("aaaa", draw);
//    std::cout << "3d???: " << cur_pos_point << std::endl;
//    std::cout << "center: " << center_point_3d << std::endl;
//    std::cout << "??????3d???: " << pre_pos_point_3d << std::endl;
//#endif
}

//???????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
cv::Point3f RuneHitLogic::calCurPosCoordinate(float pit_abs_angle, float yaw_abs_angle)
{
    cv::Point3f curPos_3d_ori;

    pit_abs_angle = pit_abs_angle * PI / 180;
    yaw_abs_angle = yaw_abs_angle * PI / 180;

    //????????????????????????????????????????????????360???
    curPos_3d_ori.z = PTZ2TARGET_RELATIVE_DIS;
    curPos_3d_ori.x = PTZ2TARGET_RELATIVE_DIS * tan(yaw_abs_angle);
    curPos_3d_ori.y = PTZ2TARGET_RELATIVE_DIS * tan(pit_abs_angle);

    return curPos_3d_ori;
}

//??????????????????3????????????????????????????????????????????????????????????????????????????????????
void RuneHitLogic::calCoorinateAbsAngle(cv::Point3f curPos_3d_ori,
                                        float &pit_abs_angle, float &yaw_abs_angle)
{
    yaw_abs_angle = atan2(curPos_3d_ori.x, curPos_3d_ori.z);
    pit_abs_angle = atan2(curPos_3d_ori.y, curPos_3d_ori.z); //not PI/2

    yaw_abs_angle = yaw_abs_angle * 180 / PI;
    pit_abs_angle = pit_abs_angle * 180 / PI;
}

void RuneHitLogic::CompensateAngle(UnpackData *unpack_data, float &pit_ref_send, float &yaw_ref_send) //offset_angle use
{
    float pitch_offset_angle;
    float yaw_offset_angle;
    int is_ptz_raise = 0;
    int is_ptz_right = 0;

    if(!std::isnan(unpack_data->getStm2PcMesg()->stm32_info_data.pitch_offset) && !std::isnan(unpack_data->getStm2PcMesg()->stm32_info_data.yaw_offset)
       && unpack_data->getStm2PcMesg()->stm32_info_data.pitch_offset > -1000 && unpack_data->getStm2PcMesg()->stm32_info_data.yaw_offset > -1000)
    {
        pitch_offset_angle = unpack_data->getStm2PcMesg()->stm32_info_data.pitch_offset; // ?????????
        yaw_offset_angle = unpack_data->getStm2PcMesg()->stm32_info_data.yaw_offset;  // ?????????
#ifdef DEBUG_MODE
        std::cout << "pit_off:   " << pitch_offset_angle << std::endl;
        std::cout << "yaw_off:   " << yaw_offset_angle << std::endl;
#endif // DEBUG_MODE

        if(pitch_offset_angle > 0)
            is_ptz_raise = 1;
        if(yaw_offset_angle > 0)
            is_ptz_right = 1;

        pitch_offset_angle = std::abs(pitch_offset_angle);
        yaw_offset_angle = std::abs(yaw_offset_angle);
    }
    else
    {
#ifdef DEBUG_MODE
        std::cout << "not get gimbol offset!!!" << std::endl;
#endif // DEBUG_MODE
        pitch_offset_angle = rune_settings->rune_param.rune_compensate.pitch_offset / 10.0;
        yaw_offset_angle = rune_settings->rune_param.rune_compensate.yaw_offset / 10.0;
        is_ptz_raise = rune_settings->rune_param.rune_compensate.is_ptz_raise;
        is_ptz_right = rune_settings->rune_param.rune_compensate.is_ptz_right;
    }

    if (is_ptz_raise)
    {
        pit_ref_send = pit_ref_send + pitch_offset_angle; //?????????
        pit_compansate_angle = pitch_offset_angle;
    }
    else
    {
        pit_ref_send = pit_ref_send - pitch_offset_angle;
        pit_compansate_angle = -pitch_offset_angle;
    }

    if (is_ptz_right)
    {
        yaw_ref_send = yaw_ref_send + yaw_offset_angle; //?????????
        yaw_compansate_angle = yaw_offset_angle;
    }
    else
    {
        yaw_ref_send = yaw_ref_send - yaw_offset_angle;
        yaw_compansate_angle = -yaw_offset_angle;
    }
}


void RuneHitLogic::sendHitMsg2Stm(PackData *pack_data, Serial *serial, long t1_param)
{
#ifdef USE_SERIAL

#ifdef DEBUG_MODE
    std::cout << "================================================================================" << std::endl;
    std::cout << "pit_send: " << pit_abs_angle << std::endl;
    std::cout << "yaw_send: " << yaw_abs_angle << std::endl;
#endif // DEBUG_MODE

    pack_data->setPc2StmMesg()->gimbal_control_data.time = t1_param;
    pack_data->setPc2StmMesg()->gimbal_control_data.pit_ref = -pit_abs_angle;   // ???????????????
    pack_data->setPc2StmMesg()->gimbal_control_data.yaw_ref = -yaw_abs_angle;   // ???????????????
    pack_data->setPc2StmMesg()->gimbal_control_data.visual_valid = 0;
    //pack_data->setPc2StmMesg()->gimbal_control_data.buff_shoot = 1;

    static int shoot = 0;

    if(main_settings->debug.b_show_shoot)
    {
        cv::namedWindow("buff_shoot");
        cv::createTrackbar("buff_shoot","buff_shoot",&shoot ,1);
    }
    else
    {
        if(-1!=cv::getWindowProperty("buff_shoot",1))
            cv::destroyWindow("buff_shoot");
    }

    pack_data->setPc2StmMesg()->gimbal_control_data.buff_shoot = shoot;

    //6.????????????
    pack_data->process(serial);
#endif // USE_SERIAL
}

void RuneHitLogic::sendShiftMsg2Stm(PackData *pack_data, Serial *serial, long t1_param)
{
#ifdef USE_SERIAL

#ifdef DEBUG_MODE
    std::cout << "================================================================================" << std::endl;
    std::cout << "pit_send: " << pit_abs_angle << std::endl;
    std::cout << "yaw_send: " << yaw_abs_angle << std::endl;
#endif // DEBUG_MODE

    pack_data->setPc2StmMesg()->gimbal_control_data.time = t1_param;
    pack_data->setPc2StmMesg()->gimbal_control_data.pit_ref = -pit_abs_angle;   // ???????????????
    pack_data->setPc2StmMesg()->gimbal_control_data.yaw_ref = -yaw_abs_angle;   // ???????????????
    pack_data->setPc2StmMesg()->gimbal_control_data.visual_valid = 1;
    pack_data->setPc2StmMesg()->gimbal_control_data.buff_shoot = 0;
    //????????????
    pack_data->process(serial);
#endif // USE_SERIAL
}

void RuneHitLogic::sendStopMsg2Stm(PackData *pack_data, Serial *serial, long t1_param)
{
#ifdef USE_SERIAL

#ifdef DEBUG_MODE
    std::cout << "================================================================================" << std::endl;
    std::cout << "pit_send: " << pit_abs_angle << std::endl;
    std::cout << "yaw_send: " << yaw_abs_angle << std::endl;
#endif // DEBUG_MODE

    pack_data->setPc2StmMesg()->gimbal_control_data.time = t1_param;
    pack_data->setPc2StmMesg()->gimbal_control_data.pit_ref = -pit_abs_angle;   // ???????????????
    pack_data->setPc2StmMesg()->gimbal_control_data.yaw_ref = -yaw_abs_angle;   // ???????????????
    pack_data->setPc2StmMesg()->gimbal_control_data.visual_valid = 0;
    pack_data->setPc2StmMesg()->gimbal_control_data.buff_shoot = 0;
    //6.????????????
    pack_data->process(serial);
#endif // USE_SERIAL
}

void RuneHitLogic::getNormalShiftAngle(cv::RotatedRect &rot_rect, AngleSolver &angle_solver, float ratio)
{
    angle_solver.getAngleWithRectify(rot_rect, ratio);

    pit_abs_angle -= angle_solver.getPitRef();  // ?????????+?????????
    yaw_abs_angle += angle_solver.getYawRef();

#ifdef DEBUG_MODE
    std::cout << "pit_ref_angle: " << angle_solver.getPitRef() << std::endl;
    std::cout << "yaw_ref_angle: " << angle_solver.getYawRef() << std::endl;
    std::cout << "pit_abs_angle: " << pit_abs_angle << std::endl;
    std::cout << "yaw_abs_angle: " << yaw_abs_angle << std::endl;
#endif // DEBUG_MODE
}

void RuneHitLogic::setPitAbsAngleWithGavityCompansate(GravityCompensateResolve& gc)
{
    cv::Point3f target2ptz_Point = calCurPosCoordinate(pit_abs_angle, yaw_abs_angle);
    float target2ptz_dis = sqrt(powf(target2ptz_Point.x, 2) + powf(target2ptz_Point.y, 2) + powf(target2ptz_Point.z, 2)) / 100.0; //????????????

    float target2bul_dis = (target2ptz_dis * 100.0 - OFFSET_Z_PTZ2BULLET) / 100.0;

    float pit_send_angle = gc.solveAngleWithGravity(pit_abs_angle, target2bul_dis);

    pit_gavity_compansate_angle = pit_send_angle - pit_abs_angle;
    pit_abs_angle = pit_send_angle;
}

void RuneHitLogic::updateRuneRotDirection(bool is_clockwise)
{
    this->is_clockwise = is_clockwise;
}

bool RuneHitLogic::selectRuneRotation(void)
{
    int clockwise_num = 0;
    int anticlockwise_num = 0;
    int size = rotation_capacity.size();

    for(int i = 0; i < size; i++)
    {
        if(rotation_capacity[i])
            clockwise_num++;
        else
            anticlockwise_num++;
    }
    return (clockwise_num > anticlockwise_num); //?????????????????????????????????size?????????
}

cv::Point3f RuneHitLogic::selectRuneCenter3d(void)
{
    cv::Point3f center_3d;
    std::vector<cv::Point3f> center_select_capacity;
    float diff = 9;
    int size = center_capacity.size();

    if(size > 2)
    {
        int i, end = center_capacity.size() - 2;
        for(i = 0; i < end; i++)
        {
            if(std::abs(center_capacity[i].x - center_capacity[i + 1].x) < diff) //??????????????????
            {
                center_select_capacity.push_back(center_capacity[i]);
                center_3d.x += center_capacity[i].x;
                center_3d.y += center_capacity[i].y;
            }
            else //????????????
            {
                i++;
                if(std::abs(center_capacity[i].x - center_capacity[i + 1].x) < diff) //??????????????????????????????
                {
                    center_select_capacity.push_back(center_capacity[i]);
                    center_3d.x += center_capacity[i].x;
                    center_3d.y += center_capacity[i].y;
                }
                else //????????????
                {
                    if(std::abs(center_capacity[i - 1].x - center_capacity[i + 1].x) < diff) //?????????????????????????????????????????????????????????
                    {
                        center_select_capacity.push_back(center_capacity[i - 1]);
                        center_3d.x += center_capacity[i - 1].x;
                        center_3d.y += center_capacity[i - 1].y;
                    } //else none???????????????????????????????????????i????????????????????????????????????
                }
            }
        }
        if(i == end)
        {
            // 1????????????????????????if???????????????????????????????????????????????????
            // 2????????????????????????if???????????????????????????????????????????????????
            // 3????????????????????????else?????????????????????????????????????????????????????????
            if(std::abs(center_capacity[i - 1].x - center_capacity[i + 1].x) < diff)
            {
                center_select_capacity.push_back(center_capacity[i + 1]);
                center_3d.x += center_capacity[i + 1].x;
                center_3d.y += center_capacity[i + 1].y;
            }
            center_select_capacity.push_back(center_capacity[i]);
            center_3d.x += center_capacity[i].x;
            center_3d.y += center_capacity[i].y;
        }
        else //????????????????????????
        {
            if(center_select_capacity.size() == 0)
            {
                center_select_capacity.push_back(center_capacity[i]); //????????????????????????????????????????????????
                center_3d.x += center_capacity[i].x;
                center_3d.y += center_capacity[i].y;
            }
            else
            {
                if(std::abs(center_capacity[i].x - center_select_capacity[0].x) < diff) //???????????????????????????????????????????????????
                {
                    center_select_capacity.push_back(center_capacity[i]);
                    center_3d.x += center_capacity[i].x;
                    center_3d.y += center_capacity[i].y;
                }
            }
        }
        center_3d.x /= center_select_capacity.size(); //???????????????
        center_3d.y /= center_select_capacity.size();
        center_3d.z = center_select_capacity[0].z;
    }
    else if(center_capacity.size() == 2)
    {
        if(std::abs(center_capacity[0].x - center_capacity[1].x) < diff) //???????????????
        {
            //center_select_capacity.push_back(center_capacity[0]);
            center_3d.x = (center_capacity[0].x + center_capacity[1].x) / 2;
            center_3d.y = (center_capacity[0].y + center_capacity[1].y) / 2;
            center_3d.z = center_capacity[0].z;
        }
        else //????????????,???????????????
        {
            //center_select_capacity.push_back(center_capacity[0]);
            center_3d.x = center_capacity[0].x;
            center_3d.y = center_capacity[0].y;
            center_3d.z = center_capacity[0].z;
        }
    }
    else if(center_capacity.size() == 1)
    {
        //center_select_capacity.push_back(center_capacity[0]);
        center_3d.x = center_capacity[0].x;
        center_3d.y = center_capacity[0].y;
        center_3d.z = center_capacity[0].z;
    }

    return center_3d;
}

void RuneHitLogic::setRuneRotDirection(void)  // ??????????????????????????????????????????????????????????????????
{
    angle_diff = angle_diff * 180 / PI;

    if (angle_diff < -180)  // ???????????????????????????????????????????????????????????????
        updateRuneRotDirection(true);  // ?????????
    else if (angle_diff < 0)
        updateRuneRotDirection(false);  // ?????????
    else if (angle_diff > 180)  // ???????????????????????????????????????????????????????????????
        updateRuneRotDirection(false);  // ?????????
    else if (angle_diff != 0)  // ???0???????????????
        updateRuneRotDirection(true);  // ?????????
}

void RuneHitLogic::smallRuneHitProc(cv::Mat src, AngleSolver &angle_solver,
                                  PackData *pack_data, UnpackData *unpack_data,
                                  Serial *serial, long t1_param, GravityCompensateResolve &gc)
{
    if(main_settings->main_mode_flag != 1)
    {
        state_change = start_state;
        frame_count = 0;
    }

    static int shoot_count;

    switch(state_change)
    {
    case start_state:
    {
        bool is_find_rune = getCurPos(src, rot_rect, cur_pos);
        if(is_find_rune)
        {
            bool is_find_rune_center = false;
            if(getPrepTargetRotRect().size() == 1)
                is_find_rune_center = setRuneCenter(src, rot_rect);
            if(is_find_rune_center)
            {
                center_2d = getRuneCenter();
                angle_old = angle_recent;
                setAngleRecent(rot_rect.center);
#ifdef DEBUG_MODE
                std::cout << "angle_recent: " << angle_recent << std::endl;
#endif // DEBUG_MODE
                pit_abs_angle = -unpack_data->getStm2PcMesg()->stm32_info_data.pitch;
                yaw_abs_angle = -unpack_data->getStm2PcMesg()->stm32_info_data.yaw;

                if(std::isnan(pit_abs_angle) || std::isnan(yaw_abs_angle))
                {
#ifdef DEBUG_MODE
                    std::cout << "fail to get the ptz angle!" << std::endl;
#endif // DEBUG_MODE
                    sendStopMsg2Stm(pack_data, serial, t1_param);
                    return;
                }
#ifdef DEBUG_MODE
                std::cout << "pit_code_angle: " << getPitAbsAngle() << std::endl;
                std::cout << "yaw_code_angle: " << getYawAbsAngle() << std::endl;
#endif // DEBUG_MODE
                cv::RotatedRect center_rect(getRuneCenter(), cv::Size(15, 15), 0);
                getNormalShiftAngle(center_rect, angle_solver, 1);
                center_point_3d = calCurPosCoordinate(pit_abs_angle, yaw_abs_angle);

                if (frame_count == 0)
                {
                    center_capacity.clear();
                    rotation_capacity.clear();
                    center_capacity.push_back(center_point_3d);
                    frame_count++;
                }
                else if (frame_count < FRAME_FILTER_NUM)
                {
                    angle_diff = angle_recent - angle_old;
                    setRuneRotDirection();  // ???????????????????????????
                    center_capacity.push_back(center_point_3d);
                    rotation_capacity.push_back(getRotation());
                    frame_count++;
                }
                else if (frame_count == FRAME_FILTER_NUM)
                {
                    angle_diff = angle_recent - angle_old;
                    setRuneRotDirection();//???????????????????????????
                    center_capacity.push_back(center_point_3d);
                    rotation_capacity.push_back(getRotation());
                    center_point_3d = selectRuneCenter3d(); //????????????
                    updateRuneRotDirection(selectRuneRotation()); //????????????
                    frame_count++;
                }
                else //????????????????????????????????????
                {
                    rotated_angle  = PI / 3 * T;
                    predictShiftProcWithCompansate(rot_rect, angle_solver, pack_data, unpack_data, serial, t1_param, rotated_angle, gc);
                    state_change = shift_state;
                    gettimeofday(&t1_difftime, NULL); //????????????
                }
            }
            else
            {
                sendStopMsg2Stm(pack_data, serial, t1_param);
#ifdef DEBUG_MODE
                std::cout << "not find rune center!" << std::endl;
#endif // DEBUG_MODE
            }
        }
        break;
    }
    case shift_state:
    {
        gettimeofday(&t2_difftime, NULL); //????????????
        double diff_time = (long)(1000000 * (t2_difftime.tv_sec - t1_difftime.tv_sec) + t2_difftime.tv_usec - t1_difftime.tv_usec) / 1000000.0;

#ifdef DEBUG_MODE
        std::cout << "shift diff_time: "<< diff_time << std::endl;
#endif // DEBUG_MODE
        if(diff_time < rune_settings->rune_param.rune_time_delay.PTZ_SHIFT_DELAY * 0.01)  //???????????????????????????
        {
            sendShiftMsg2Stm(pack_data, serial, t1_param);
        }
        else //?????????????????????????????????
        {
            sendHitMsg2Stm(pack_data, serial, t1_param);
            state_change = shoot_state;
        }
        break;
    }
    case shoot_state:
    {
        if(shoot_count == SHOOT_NUM)
        {
            gettimeofday(&t1_difftime, NULL); //??????????????????
            sendStopMsg2Stm(pack_data, serial, t1_param);
            state_change = stop_state;
            shoot_count = 0;
            std::cout << "shoot" << std::endl;
            break;
        }

        sendHitMsg2Stm(pack_data, serial, t1_param);
        shoot_count++;
        break;
    }
    case stop_state:
    {
        gettimeofday(&t2_difftime, NULL); //????????????
        double diff_time = (long)(1000000 * (t2_difftime.tv_sec - t1_difftime.tv_sec) + t2_difftime.tv_usec - t1_difftime.tv_usec) / 1000000.0;

#ifdef DEBUG_MODE
        std::cout << "stop diff_time: "<< diff_time << std::endl;
#endif // DEBUG_MODE
        if (diff_time < rune_settings->rune_param.rune_time_delay.SHOOT2HIT_DELAY * 0.01)
        {
            sendStopMsg2Stm(pack_data, serial, t1_param);
        }
        else
        {
            pit_abs_angle = -unpack_data->getStm2PcMesg()->stm32_info_data.pitch;
            yaw_abs_angle = -unpack_data->getStm2PcMesg()->stm32_info_data.yaw;

            if(std::isnan(pit_abs_angle) || std::isnan(yaw_abs_angle))
            {
#ifdef DEBUG_MODE
                std::cout << "fail to get the ptz angle!" << std::endl;
#endif // DEBUG_MODE
                sendStopMsg2Stm(pack_data, serial, t1_param);

                return;
            }

            //?????????????????????????????????
            bool is_find_rune = getCurPos(src, rot_rect, cur_pos);

            if (is_find_rune) //?????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
            {
                predictShiftProcWithCompansate(rot_rect, angle_solver, pack_data, unpack_data, serial, t1_param, rotated_angle, gc);
                state_change = shift_state;
                gettimeofday(&t1_difftime, NULL); //??????????????????
            }
            else
            {
                sendStopMsg2Stm(pack_data, serial, t1_param);
#ifdef DEBUG_MODE
                std::cout << "not find rune!" << std::endl;
#endif // DEBUG_MODE
            }

        }
        break;
    }
    default:
        break;
    }
}

void RuneHitLogic::bigRuneHitProc(cv::Mat src, AngleSolver &angle_solver,
                                  PackData *pack_data, UnpackData *unpack_data,
                                  Serial *serial, long t1_param, GravityCompensateResolve &gc)
{
    if(main_settings->main_mode_flag != 2)
    {
        state_change = start_state;
        frame_count = 0;
    }
    static int shoot_count;

    switch(state_change)
    {
    case start_state:
    {
        bool is_find_rune = getCurPos(src, rot_rect, cur_pos);
        if(is_find_rune)
        {
            bool is_find_rune_center = false;
            if(getPrepTargetRotRect().size() == 1)
                is_find_rune_center = setRuneCenter(src, rot_rect);
            if(is_find_rune_center)
            {
                center_2d = getRuneCenter();
                angle_old = angle_recent;
                angle_old_2PI = angle_recent_2PI;
                setAngleRecent(rot_rect.center);
                pit_abs_angle = -unpack_data->getStm2PcMesg()->stm32_info_data.pitch;
                yaw_abs_angle = -unpack_data->getStm2PcMesg()->stm32_info_data.yaw;

                if(std::isnan(pit_abs_angle) || std::isnan(yaw_abs_angle))
                {
#ifdef DEBUG_MODE
                    std::cout << "fail to get the ptz angle!" << std::endl;
#endif // DEBUG_MODE
                    sendStopMsg2Stm(pack_data, serial, t1_param);
                    return;
                }
#ifdef DEBUG_MODE
                std::cout << "pit_code_angle: " << getPitAbsAngle() << std::endl;
                std::cout << "yaw_code_angle: " << getYawAbsAngle() << std::endl;
#endif // DEBUG_MODE
                cv::RotatedRect center_rect(getRuneCenter(), cv::Size(15, 15), 0);
                getNormalShiftAngle(center_rect, angle_solver, 1);
                center_point_3d = calCurPosCoordinate(pit_abs_angle, yaw_abs_angle);

                if (frame_count == 0)
                {
                    //??????????????????????????????
                    angle_old_for_bigRune = angle_recent_2PI;
                    gettimeofday(&t1_difftime_for_period, NULL); //????????????

                    center_capacity.clear();
                    rotation_capacity.clear();
                    center_capacity.push_back(center_point_3d);
                    frame_count++;
                }
                else if (frame_count < FRAME_FILTER_NUM)
                {
                    angle_diff = angle_recent - angle_old;
                    setRuneRotDirection();//???????????????????????????

                    center_capacity.push_back(center_point_3d);
                    rotation_capacity.push_back(getRotation());
                    frame_count++;
                }
                else if (frame_count == FRAME_FILTER_NUM)
                {
                    angle_diff = angle_recent - angle_old;

                    setRuneRotDirection();//???????????????????????????

                    center_capacity.push_back(center_point_3d);
                    rotation_capacity.push_back(getRotation());

                    center_point_3d = selectRuneCenter3d(); //????????????
                    updateRuneRotDirection(selectRuneRotation()); //????????????

                    gettimeofday(&t2_difftime_for_period, NULL); //????????????
                    double diff_time = (long)(1000000 * (t2_difftime_for_period.tv_sec - t1_difftime_for_period.tv_sec) + t2_difftime_for_period.tv_usec - t1_difftime_for_period.tv_usec) / 1000000.0;
                    //??????????????????????????????
                    double angle_diff_for_bigRune_init = getAngleDiffForBigRuneInit();

#ifdef DEBUG_MODE
                    std::cout << "angle_diff_for_bigRune_init: " << angle_diff_for_bigRune_init << std::endl;
#endif // DEBUG_MODE
                    // ??????????????????????????????
                    time_begin_in_period = calStartTimeInPeriod(angle_diff_for_bigRune_init, diff_time); //??????t1_difftime_for_period??????????????????????????????
#ifdef DEBUG_MODE
                    std::cout << "time_begin_in_period: " << time_begin_in_period << std::endl;
#endif // DEBUG_MODE
                    // ????????????????????????????????????????????????nan???
                    if(std::isnan(time_begin_in_period) || std::isnan(angle_diff_for_bigRune_init))
                        frame_count = 0;
                    else
                        frame_count++;
                }
                else
                {
                    //??????????????????????????????
                    rotated_angle = predictAngleForSin(); //???????????????????????????T??????????????????
                    predictShiftProcWithCompansate(rot_rect, angle_solver, pack_data, unpack_data, serial, t1_param, rotated_angle, gc);
                    state_change = shift_state;
                    gettimeofday(&t1_difftime, NULL); //????????????
                }
            }
            else
            {
                sendStopMsg2Stm(pack_data, serial, t1_param);
#ifdef DEBUG_MODE
                std::cout << "not find rune center!" << std::endl;
#endif // DEBUG_MODE
            }
        }
        break;
    }
    case shift_state:
    {
        gettimeofday(&t2_difftime, NULL); //????????????
        double diff_time = (long)(1000000 * (t2_difftime.tv_sec - t1_difftime.tv_sec) + t2_difftime.tv_usec - t1_difftime.tv_usec) / 1000000.0;

#ifdef DEBUG_MODE
        std::cout << "shift diff_time: " << diff_time << std::endl;
#endif // DEBUG_MODE
        if (diff_time < rune_settings->rune_param.rune_time_delay.PTZ_SHIFT_DELAY * 0.01)//???????????????????????????
        {
            sendShiftMsg2Stm(pack_data, serial, t1_param);
        }
        else //?????????????????????????????????
        {
            sendHitMsg2Stm(pack_data, serial, t1_param);
            state_change = shoot_state;
        }
        break;
    }
    case shoot_state:
    {
        if(shoot_count == SHOOT_NUM)
        {
            gettimeofday(&t1_difftime, NULL); //??????????????????
            sendStopMsg2Stm(pack_data, serial, t1_param);
            state_change = stop_state;
            shoot_count = 0;
            break;
        }

        sendHitMsg2Stm(pack_data, serial, t1_param);
        shoot_count++;
        break;
    }
    case stop_state:
    {
        gettimeofday(&t2_difftime, NULL); //????????????
        double diff_time = (long)(1000000 * (t2_difftime.tv_sec - t1_difftime.tv_sec) + t2_difftime.tv_usec - t1_difftime.tv_usec) / 1000000.0;

#ifdef DEBUG_MODE
        std::cout << "stop_diff_time: "<< diff_time << std::endl;
#endif // DEBUG_MODE

        if (diff_time < rune_settings->rune_param.rune_time_delay.SHOOT2HIT_DELAY * 0.01)
        {
            sendStopMsg2Stm(pack_data, serial, t1_param);
        }
        else
        {
            pit_abs_angle = -unpack_data->getStm2PcMesg()->stm32_info_data.pitch;
            yaw_abs_angle = -unpack_data->getStm2PcMesg()->stm32_info_data.yaw;

            if(std::isnan(pit_abs_angle) || std::isnan(yaw_abs_angle))
            {
#ifdef DEBUG_MODE
                std::cout << "fail to get the ptz angle!" << std::endl;
#endif // DEBUG_MODE
                sendStopMsg2Stm(pack_data, serial, t1_param);
                return;
            }

            //?????????????????????????????????
            bool is_find_rune = getCurPos(src, rot_rect, cur_pos);

            if (is_find_rune) //?????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
            {
                //??????????????????????????????
                rotated_angle = predictAngleForSin(); //???????????????????????????T??????????????????

                predictShiftProcWithCompansate(rot_rect, angle_solver, pack_data, unpack_data, serial, t1_param, rotated_angle, gc);

                state_change = shift_state;
                gettimeofday(&t1_difftime, NULL); //??????????????????
            }
            else
            {
                sendStopMsg2Stm(pack_data, serial, t1_param);
#ifdef DEBUG_MODE
                std::cout << "not find rune!" << std::endl;
#endif // DEBUG_MODE
            }
        }
        break;
    }
    default:
        break;

    }
}

void RuneHitLogic::staticRuneHitProc(cv::Mat src, AngleSolver &angle_solver,
                                     PackData *pack_data, UnpackData *unpack_data,
                                     Serial *serial, long t1_param, GravityCompensateResolve &gc)
{
    if(main_settings->main_mode_flag != 3)
    {
        state_change = start_state;
        active_rune_num = 0;
    }
    static int shoot_count;

    switch(state_change)
    {
    case start_state:
    {
        //?????????????????????????????????
        bool is_find_rune = getCurPos(src, rot_rect, cur_pos);

        if (is_find_rune) //?????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
        {
            pit_abs_angle = -unpack_data->getStm2PcMesg()->stm32_info_data.pitch;
            yaw_abs_angle = -unpack_data->getStm2PcMesg()->stm32_info_data.yaw;

            if(std::isnan(pit_abs_angle) || std::isnan(yaw_abs_angle))
            {
#ifdef DEBUG_MODE
                std::cout << "fail to get the ptz angle!" << std::endl;
#endif // DEBUG_MODE
                sendStopMsg2Stm(pack_data, serial, t1_param);
                return;
            }

#ifdef DEBUG_MODE
            std::cout << "pit_code_angle: " << getPitAbsAngle() << std::endl;
            std::cout << "yaw_code_angle: " << getYawAbsAngle() << std::endl;
#endif // DEBUG_MODE

            normalShiftProcWithCompansate(rot_rect, angle_solver, pack_data, unpack_data, serial, t1_param, gc);
            state_change = shift_state;
            gettimeofday(&t1_difftime, NULL); //????????????
        }
        else
        {
            sendStopMsg2Stm(pack_data, serial, t1_param);
#ifdef DEBUG_MODE
            std::cout << "not find rune!" << std::endl;
#endif // DEBUG_MODE
        }
        break;
    }
    case shift_state:
    {
        gettimeofday(&t2_difftime, NULL); //????????????
        double diff_time = (long)(1000000 * (t2_difftime.tv_sec - t1_difftime.tv_sec) + t2_difftime.tv_usec - t1_difftime.tv_usec) / 1000000.0;

        if (diff_time < rune_settings->rune_param.rune_time_delay.PTZ_SHIFT_DELAY * 0.01)//???????????????????????????
        {
            sendShiftMsg2Stm(pack_data, serial, t1_param);
        }
        else //?????????????????????????????????
        {
            sendHitMsg2Stm(pack_data, serial, t1_param);
            state_change = shoot_state;
        }
        break;
    }
    case shoot_state:
    {
        if(shoot_count == SHOOT_NUM)
        {
            gettimeofday(&t1_difftime, NULL); //??????????????????
            sendStopMsg2Stm(pack_data, serial, t1_param);
            state_change = stop_state;
            shoot_count = 0;
            break;
        }

        sendHitMsg2Stm(pack_data, serial, t1_param);
        shoot_count++;
        break;
    }
    case stop_state:
    {
        gettimeofday(&t2_difftime, NULL); //????????????
        double diff_time = (long)(1000000 * (t2_difftime.tv_sec - t1_difftime.tv_sec) + t2_difftime.tv_usec - t1_difftime.tv_usec) / 1000000.0;

        if (diff_time < rune_settings->rune_param.rune_time_delay.SHOOT2HIT_DELAY * 0.01)
        {
            sendStopMsg2Stm(pack_data, serial, t1_param);
        }
        else
        {
            pit_abs_angle = -unpack_data->getStm2PcMesg()->stm32_info_data.pitch;
            yaw_abs_angle = -unpack_data->getStm2PcMesg()->stm32_info_data.yaw;

            if(std::isnan(pit_abs_angle) || std::isnan(yaw_abs_angle))
            {
#ifdef DEBUG_MODE
                std::cout << "fail to get the ptz angle!" << std::endl;
#endif // DEBUG_MODE
                sendStopMsg2Stm(pack_data, serial, t1_param);
                return;
            }

            judgeHitOrShift(src, angle_solver, pack_data, unpack_data, serial, t1_param, gc);
        }
        break;
    }
    }
}
