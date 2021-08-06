#include "Settings/Settings.h"

Settings::Settings()
{

}

ArmorSettings::ArmorSettings(const char *param)
{
    this->readArmorParam(param);
}

MainSettings::MainSettings(const char *param)
{
    this->readOtherParam(param);
}

RuneSettings::RuneSettings(const char *param)
{
    this->readRuneParam(param);
}

void RuneSettings::setRuneParam(MainSettings *main_setting)
{
    if(main_setting->debug.b_show_bin_rune)
        setPreprocessParam(WIN_NAME_RUNEP_PREPROCESS);
    else
    {
        if(-1 != cv::getWindowProperty(WIN_NAME_RUNEP_PREPROCESS, 1))
            cv::destroyWindow(WIN_NAME_RUNEP_PREPROCESS);
    }

    if(main_setting->debug.b_show_compensate_rune)
        setRuneCompensate(WIN_NAME_RUNE_COMPENSATE, rune_param.rune_compensate);
    else
    {
        if(-1 != cv::getWindowProperty(WIN_NAME_RUNE_COMPENSATE, 1))
            cv::destroyWindow(WIN_NAME_RUNE_COMPENSATE);
    }

    if(main_setting->debug.b_show_rune_time_delay)
        setRuneTimeParam(WIN_NAME_RUNE_TIME, rune_param.rune_time_delay);
    else
    {
        if(-1 != cv::getWindowProperty(WIN_NAME_RUNE_TIME, 1))
            cv::destroyWindow(WIN_NAME_RUNE_TIME);
    }

    if(main_setting->debug.b_show_param_rune)
    {
        setTgtSizeParamRune(WIN_NAME_RUNEP, rune_param.tgt_size_param_u); //待激活
        setTgtSizeParamRune(WIN_NAME_RUNEP_R, rune_param.tgt_size_param_c);//中心
    }
    else
    {
        if(-1 != cv::getWindowProperty(WIN_NAME_RUNEP, 1))
            cv::destroyWindow(WIN_NAME_RUNEP);
        if(-1 != cv::getWindowProperty(WIN_NAME_RUNEP_R, 1))
            cv::destroyWindow(WIN_NAME_RUNEP_R);
        return;
    }
}

void RuneSettings::setRuneCompensate(const char *win_name, RuneCompensate &rune_compensate)
{
    cv::namedWindow(win_name);
    cv::createTrackbar("is_ptz_raise", win_name, &rune_compensate.is_ptz_raise, 1);
    cv::createTrackbar("pitch_offset", win_name, &rune_compensate.pitch_offset, 100);
    cv::createTrackbar("is_ptz_right", win_name, &rune_compensate.is_ptz_right, 1);
    cv::createTrackbar("yaw_offset", win_name, &rune_compensate.yaw_offset, 100);
}

void RuneSettings::setRuneTimeParam(const char *win_name, RuneTimeParam &rune_time)
{
    cv::namedWindow(win_name);
    cv::createTrackbar("PTZ_SHIFT_DELAY", win_name, &rune_time.PTZ_SHIFT_DELAY, 200);
    cv::createTrackbar("SHOOT2HIT_DELAY", win_name, &rune_time.SHOOT2HIT_DELAY, 200);
    cv::createTrackbar("TRIG2SHOOT2HIT_DELAY", win_name, &rune_time.TRIG2SHOOT2HIT_DELAY, 200);
}

void RuneSettings::readRuneParam(const char *param_path)
{
    cv::FileStorage fs(param_path, cv::FileStorage::READ);
    fs["rune_center_gray_thre_min"] >> preprocess_param.gray_thre_min;
    fs["rune_center_gray_thre_max"] >> preprocess_param.gray_thre_max;

    // 中心
    fs["rune_center_len_min"] >> rune_param.tgt_size_param_c.len_min;
    fs["rune_center_len_max"] >> rune_param.tgt_size_param_c.len_max;
    fs["rune_center_light_hw_ratio_min"] >> rune_param.tgt_size_param_c.hw_ratio_min;
    fs["rune_center_light_hw_ratio_max"] >> rune_param.tgt_size_param_c.hw_ratio_max;
    fs["rune_center_light_area_min"] >> rune_param.tgt_size_param_c.area_min;
    fs["rune_center_light_area_max"] >> rune_param.tgt_size_param_c.area_max;
    fs["rune_center_area_ratio_min"] >> rune_param.tgt_size_param_c.area_ratio_min;
    fs["rune_center_area_ratio_max"] >> rune_param.tgt_size_param_c.area_ratio_max;
    fs["rune_center_center_distance_min"] >> rune_param.tgt_size_param_c.center_distance_min;
    fs["rune_center_center_distance_max"] >> rune_param.tgt_size_param_c.center_distance_max;

    // 待激活
    fs["rune_activating_len_min"] >> rune_param.tgt_size_param_u.len_min;
    fs["rune_activating_len_max"] >> rune_param.tgt_size_param_u.len_max;
    fs["rune_activating_light_hw_ratio_min"] >> rune_param.tgt_size_param_u.hw_ratio_min;
    fs["rune_activating_light_hw_ratio_max"] >> rune_param.tgt_size_param_u.hw_ratio_max;
    fs["rune_activating_light_area_min"] >> rune_param.tgt_size_param_u.area_min;
    fs["rune_activating_light_area_max"] >> rune_param.tgt_size_param_u.area_max;
    fs["rune_activating_area_ratio_min"] >> rune_param.tgt_size_param_u.area_ratio_min;
    fs["rune_activating_area_ratio_max"] >> rune_param.tgt_size_param_u.area_ratio_max;
    fs["rune_activating_parent_area_min"] >> rune_param.tgt_size_param_u.parent_area_min;
    fs["rune_activating_parent_area_max"] >> rune_param.tgt_size_param_u.parent_area_max;
    fs["rune_activating_parent_hw_ratio_min"] >> rune_param.tgt_size_param_u.parent_hw_ratio_min;
    fs["rune_activating_parent_hw_ratio_max"] >> rune_param.tgt_size_param_u.parent_hw_ratio_max;

    // 补偿
    fs["rune_compensate_is_ptz_raise"] >> rune_param.rune_compensate.is_ptz_raise;
    fs["rune_compensate_pitch_offset"] >> rune_param.rune_compensate.pitch_offset;
    fs["rune_compensate_is_ptz_right"] >> rune_param.rune_compensate.is_ptz_right;
    fs["rune_compensate_yaw_offset"] >> rune_param.rune_compensate.yaw_offset;

    // 延时
    fs["rune_time_delay_PTZ_SHIFT_DELAY"] >> rune_param.rune_time_delay.PTZ_SHIFT_DELAY;
    fs["rune_time_delay_SHOOT2HIT_DELAY"] >> rune_param.rune_time_delay.SHOOT2HIT_DELAY;
    fs["rune_time_delay_TRIG2SHOOT2HIT_DELAY"] >> rune_param.rune_time_delay.TRIG2SHOOT2HIT_DELAY;

#ifdef DEBUG_MODE
    std::cout << "Read rune param finished!" << std::endl;
#endif
    fs.release();
}

void RuneSettings::writeParamRune(const char *param_path)
{
    cv::FileStorage fs(param_path, cv::FileStorage::WRITE);

    fs << "rune_center_gray_thre_min" << preprocess_param.gray_thre_min;
    fs << "rune_center_gray_thre_max" << preprocess_param.gray_thre_max;

    // 中心
    fs << "rune_center_len_min" << rune_param.tgt_size_param_c.len_min;
    fs << "rune_center_len_max" << rune_param.tgt_size_param_c.len_max;
    fs << "rune_center_light_hw_ratio_min" << rune_param.tgt_size_param_c.hw_ratio_min;
    fs << "rune_center_light_hw_ratio_max" << rune_param.tgt_size_param_c.hw_ratio_max;
    fs << "rune_center_light_area_min" << rune_param.tgt_size_param_c.area_min;
    fs << "rune_center_light_area_max" << rune_param.tgt_size_param_c.area_max;
    fs << "rune_center_area_ratio_min" << rune_param.tgt_size_param_c.area_ratio_min;
    fs << "rune_center_area_ratio_max" << rune_param.tgt_size_param_c.area_ratio_max;
    fs << "rune_center_center_distance_min" << rune_param.tgt_size_param_c.center_distance_min;
    fs << "rune_center_center_distance_max" << rune_param.tgt_size_param_c.center_distance_max;

    // 待激活
    fs << "rune_activating_len_min" << rune_param.tgt_size_param_u.len_min;
    fs << "rune_activating_len_max" << rune_param.tgt_size_param_u.len_max;
    fs << "rune_activating_light_hw_ratio_min" << rune_param.tgt_size_param_u.hw_ratio_min;
    fs << "rune_activating_light_hw_ratio_max" << rune_param.tgt_size_param_u.hw_ratio_max;
    fs << "rune_activating_light_area_min" << rune_param.tgt_size_param_u.area_min;
    fs << "rune_activating_light_area_max" << rune_param.tgt_size_param_u.area_max;
    fs << "rune_activating_area_ratio_min" << rune_param.tgt_size_param_u.area_ratio_min;
    fs << "rune_activating_area_ratio_max" << rune_param.tgt_size_param_u.area_ratio_max;
    fs << "rune_activating_parent_area_min" << rune_param.tgt_size_param_u.parent_area_min;
    fs << "rune_activating_parent_area_max" << rune_param.tgt_size_param_u.parent_area_max;
    fs << "rune_activating_parent_hw_ratio_min" << rune_param.tgt_size_param_u.parent_hw_ratio_min;
    fs << "rune_activating_parent_hw_ratio_max" << rune_param.tgt_size_param_u.parent_hw_ratio_max;

    // 补偿
    fs << "rune_compensate_is_ptz_raise" << rune_param.rune_compensate.is_ptz_raise;
    fs << "rune_compensate_pitch_offset" << rune_param.rune_compensate.pitch_offset;
    fs << "rune_compensate_is_ptz_right" << rune_param.rune_compensate.is_ptz_right;
    fs << "rune_compensate_yaw_offset" << rune_param.rune_compensate.yaw_offset;

    // 延时
    fs << "rune_time_delay_PTZ_SHIFT_DELAY" << rune_param.rune_time_delay.PTZ_SHIFT_DELAY;
    fs << "rune_time_delay_SHOOT2HIT_DELAY" << rune_param.rune_time_delay.SHOOT2HIT_DELAY;
    fs << "rune_time_delay_TRIG2SHOOT2HIT_DELAY" << rune_param.rune_time_delay.TRIG2SHOOT2HIT_DELAY;

#ifdef DEBUG_MODE
    std::cout << "Save rune param finished!" << std::endl;
#endif
    fs.release();
}


void MainSettings::readOtherParam(const char *param_path)
{
    cv::FileStorage fs(param_path, cv::FileStorage::READ);

    fs["main_mode"] >> this->main_mode;
    fs["enemy_color"] >> this->enemy_color;
    fs["debug-b_show_src"] >> this->debug.b_show_src;
    fs["debug-b_show_bin"] >> this->debug.b_show_bin;
    fs["debug-b_show_target"] >> this->debug.b_show_target;
    fs["debug-b_show_armor"] >> this->debug.b_show_armor;
    fs["debug-b_show_result"] >> this->debug.b_show_result;
    fs["debug-b_show_fps"] >> this->debug.b_show_fps;
    fs["debug-b_save_result"] >> this->debug.b_save_result;
    fs["debug-n_save_result"] >> this->debug.n_save_result;
    fs["debug-b_save_pic"] >> this->debug.b_save_pic;
    fs["debug-f_save_pic_inter"] >> this->debug.f_save_pic_inter;
    fs["debug-expore_time"] >> this->debug.expore_time;
    fs["debug-b_show_param_rune"] >> this->debug.b_show_param_rune;
    fs["debug-b_show_bin_rune"] >> this->debug.b_show_bin_rune;
    fs["debug-b_show_td_rune"] >> this->debug.b_show_td_rune;
    fs["debug-b_show_cp_rune_result"] >> this->debug.b_show_cp_rune_result;
    fs["debug-b_show_cp_rune"] >> this->debug.b_show_cp_rune;
    fs["debug-b_show_center_rune"] >> this->debug.b_show_center_rune;
    fs["debug-b_show_center_rune_result"] >> this->debug.b_show_center_rune_result;
    fs["debug-b_show_center_distance"] >> this->debug.b_show_rune_center;
    fs["debug-b_show_dc"] >> this->debug.b_show_dc;
    fs["camera_param-x"] >> this->camera_param.x;
    fs["camera_param-y"] >> this->camera_param.y;
    fs["camera_param-z"] >> this->camera_param.z;
    fs["camera_param-z_ptz2bul"] >> this->camera_param.z_offset_ptz2bul;
    fs["camera_param-y_offset"] >> this->camera_param.y_offset;
    fs["camera_param-bullet_speed"] >> this->camera_param.bullet_speed;

#ifdef DEBUG_MODE
    std::cout << "Read other param finished!" << std::endl;
#endif
    fs.release();
}

void MainSettings::setMainParam(const char *win_name)
{
    static int is_shut_down_main = 0;
    static int is_shut_down_show = 1;
    static int is_shut_down_save = 1;
    static int is_shut_down_rune = 1;

    if(is_shut_down_main)
    {
        if(-1 != cv::getWindowProperty(win_name, 1))
            cv::destroyWindow(win_name);
        if(-1!=cv::getWindowProperty("展示调试窗口", 1))
            cv::destroyWindow("展示调试窗口");
        if(-1!=cv::getWindowProperty("调整曝光参数等", 1))
            cv::destroyWindow("调整曝光参数等");
        return;
    }

    cv::namedWindow(win_name);
    cv::createTrackbar("main_mode", win_name, &this->main_mode, 5);
    cv::createTrackbar("enemy_color", win_name, &this->enemy_color, 1);
    cv::createTrackbar("is_shut_down_main", win_name, &is_shut_down_main, 1);
    cv::createTrackbar("is_shut_down_other", win_name, &is_shut_down_show, 1);
    cv::createTrackbar("is_shut_down_rune", win_name, &is_shut_down_rune, 1);
    cv::createTrackbar("is_shut_down_save", win_name, &is_shut_down_save, 1);

    if(is_shut_down_save)
    {
        if(-1 != cv::getWindowProperty("调整曝光参数等",1))
            cv::destroyWindow("调整曝光参数等");
    }
    else
    {
        cv::namedWindow("调整曝光参数等");
        cv::createTrackbar("debug-b_save_result", "调整曝光参数等", &debug.b_save_result, 1);
        cv::createTrackbar("debug-b_save_pic", "调整曝光参数等", &debug.b_save_pic, 1);
        cv::createTrackbar("debug-f_save_pic_inter", "调整曝光参数等", &debug.f_save_pic_inter, 10000);
        cv::createTrackbar("debug-expore_time", "调整曝光参数等", &debug.expore_time, 100000);
    }

    if(is_shut_down_show)
    {
        if(-1 != cv::getWindowProperty("展示调试窗口", 1))
            cv::destroyWindow("展示调试窗口");
    }
    else
    {
        cv::namedWindow("展示调试窗口");
        cv::createTrackbar("debug-b_show_src", "展示调试窗口", &debug.b_show_src, 1);
        cv::createTrackbar("debug-b_show_bin", "展示调试窗口", &debug.b_show_bin, 1);
        cv::createTrackbar("debug-b_show_target", "展示调试窗口", &debug.b_show_target, 1);
        cv::createTrackbar("debug-b_show_armor", "展示调试窗口", &debug.b_show_armor, 1);
        cv::createTrackbar("debug-b_show_result", "展示调试窗口", &debug.b_show_result, 1);
        cv::createTrackbar("debug-b_show_ex_armor", "展示调试窗口", &debug.b_show_ex_armor, 1);
        cv::createTrackbar("debug-b_show_fps", "展示调试窗口", &debug.b_show_fps, 1);
        cv::createTrackbar("debug-b_show_dc", "展示调试窗口", &debug.b_show_dc, 1);
    }
    if(is_shut_down_rune)
    {
        if(-1 != cv::getWindowProperty("debug_rune_main", 1))
            cv::destroyWindow("debug_rune_main");
    }
    else
    {
        cv::namedWindow("debug_rune_main");
        cv::createTrackbar("debug-b_show_param_rune", "debug_rune_main", &debug.b_show_param_rune, 1);
        cv::createTrackbar("debug-b_show_bin_rune", "debug_rune_main", &debug.b_show_bin_rune, 1);
        cv::createTrackbar("debug-b_show_td_rune", "debug_rune_main", &debug.b_show_td_rune, 1);
        cv::createTrackbar("debug-b_show_cp_rune", "debug_rune_main", &debug.b_show_cp_rune, 1);
        cv::createTrackbar("debug-b_show_cp_rune_result", "debug_rune_main", &debug.b_show_cp_rune_result, 1);
        cv::createTrackbar("debug-b_show_center_rune", "debug_rune_main", &debug.b_show_center_rune, 1);
        cv::createTrackbar("debug-b_show_center_rune_result", "debug_rune_main", &debug.b_show_center_rune_result, 1);
        cv::createTrackbar("debug-b_show_center_distance", "debug_rune_main", &debug.b_show_rune_center, 1);
        cv::createTrackbar("debug-b_show_compensate", "debug_rune_main", &debug.b_show_compensate_rune, 1);
        cv::createTrackbar("debug-b_show_rune_time_delay", "debug_rune_main", &debug.b_show_rune_time_delay, 1);
        cv::createTrackbar("debug-b_show_shoot", "debug_rune_main", &debug.b_show_shoot, 1);
    }
}

bool Settings::setPreprocessParam(const char *win_name)
{
    cv::namedWindow(win_name);
    cv::createTrackbar("gray_thre_min",win_name,&preprocess_param.gray_thre_min, 255);
    cv::createTrackbar("gray_thre_max",win_name,&preprocess_param.gray_thre_max, 255);

    cv::createTrackbar("gray_max_w",win_name,&preprocess_param.gray_max_w, 10);
    cv::createTrackbar("gray_avg_w",win_name,&preprocess_param.gray_avg_w, 10);

    return true;
}

bool Settings::setPreprocessParam(const char *win_name, MainSettings *main_setting)
{
    if(main_setting->debug.b_show_bin)
        cv::namedWindow(win_name);
    else
    {
        if(-1 != cv::getWindowProperty(win_name, 1))
            cv::destroyWindow(win_name);
        return false;
    }
    cv::createTrackbar("gray_thre_min", win_name, &preprocess_param.gray_thre_min, 255);
    cv::createTrackbar("gray_thre_max", win_name, &preprocess_param.gray_thre_max, 255);
    cv::createTrackbar("gray_max_w", win_name, &preprocess_param.gray_max_w, 10);
    cv::createTrackbar("gray_avg_w", win_name, &preprocess_param.gray_avg_w, 10);
    return true;
}

void Settings::setTgtSizeParamRune(const char *win_name, struct TargetSizeParamRune &tgt_size_param)
{
    cv::namedWindow(win_name);

    cv::createTrackbar("len_min", win_name, &tgt_size_param.len_min, 200);
    cv::createTrackbar("len_max", win_name, &tgt_size_param.len_max, 500);
    cv::createTrackbar("hw_radio_min", win_name, &tgt_size_param.hw_ratio_min, 300);
    cv::createTrackbar("hw_radio_max", win_name, &tgt_size_param.hw_ratio_max, 300);
    cv::createTrackbar("area_ratio_min", win_name, &tgt_size_param.area_ratio_min, 100);
    cv::createTrackbar("area_ratio_max", win_name, &tgt_size_param.area_ratio_max, 100);
    cv::createTrackbar("area_min", win_name, &tgt_size_param.area_min, 2000);
    cv::createTrackbar("area_max", win_name, &tgt_size_param.area_max, 3200);
    cv::createTrackbar("parent_area_min", win_name, &tgt_size_param.parent_area_min, 2000);
    cv::createTrackbar("parent_area_max", win_name, &tgt_size_param.parent_area_max, 3200);
    cv::createTrackbar("parent_hw_ratio_min", win_name, &tgt_size_param.parent_hw_ratio_min, 300);
    cv::createTrackbar("parent_hw_ratio_max", win_name, &tgt_size_param.parent_hw_ratio_max, 300);
    cv::createTrackbar("center_distance_min", win_name, &tgt_size_param.center_distance_min, 100);
    cv::createTrackbar("center_distance_max", win_name, &tgt_size_param.center_distance_max, 100);
}

bool Settings::setTgtSizeParam(const char *win_name, MainSettings *main_setting)
{
    if(main_setting->debug.b_show_target)
        cv::namedWindow(win_name);
    else
    {
        if(-1 != cv::getWindowProperty(win_name, 1))
            cv::destroyWindow(win_name);
        return false;
    }
    cv::createTrackbar("len_min", win_name,&tgt_size_param.len_min, 200);
    cv::createTrackbar("len_max", win_name, &tgt_size_param.len_max, 1000);
    cv::createTrackbar("ratio_min", win_name, &tgt_size_param.ratio_min, 20);
    cv::createTrackbar("ratio_max", win_name, &tgt_size_param.ratio_max, 20);
    cv::createTrackbar("area_min", win_name, &tgt_size_param.area_min, 1000);
    cv::createTrackbar("area_max", win_name, &tgt_size_param.area_max, 10000);
    cv::createTrackbar("area_ratio_min", win_name, &tgt_size_param.area_ratio_min, 100);
    cv::createTrackbar("area_ratio_max", win_name, &tgt_size_param.area_ratio_max, 100);
    cv::createTrackbar("area_len_ratio_min", win_name, &tgt_size_param.area_len_ratio_min, 100);
    cv::createTrackbar("area_len_ratio_max", win_name, &tgt_size_param.area_len_ratio_max, 100);
    cv::createTrackbar("corners_size_min", win_name, &tgt_size_param.corners_size_min, 10);
    cv::createTrackbar("slope_offset", win_name, &tgt_size_param.slope_offset, 90);
    cv::createTrackbar("color_th", win_name, &tgt_size_param.color_th, 500);
    return true;
}

bool ArmorSettings::setArmorParam(MainSettings *main_setting)
{
    setPreprocessParam(WIN_NAME_ARMORP_PREPROCESS, main_setting);
    setTgtSizeParam(WIN_NAME_ARMORP_TARGET, main_setting);

    if(main_setting->debug.b_show_armor)
        cv::namedWindow(WIN_NAME_ARMORP);
    else
    {
        if(-1 != cv::getWindowProperty(WIN_NAME_ARMORP, 1))
            cv::destroyWindow(WIN_NAME_ARMORP);
        return false;
    }
    cv::createTrackbar("delta_height_max", WIN_NAME_ARMORP, &armor_param.delta_height_max, 600);
    cv::createTrackbar("delta_angle_max", WIN_NAME_ARMORP, &armor_param.delta_angle_max, 500);
    cv::createTrackbar("delta_w_ratio_min", WIN_NAME_ARMORP, &armor_param.delta_w_ratio_min, 200);
    cv::createTrackbar("delta_w_ratio_max", WIN_NAME_ARMORP, &armor_param.delta_w_ratio_max, 200);
    cv::createTrackbar("delta_h_max", WIN_NAME_ARMORP, &armor_param.delta_h_max, 1000);
    cv::createTrackbar("armor_hw_ratio_min", WIN_NAME_ARMORP, &armor_param.armor_hw_ratio_min, 100);
    cv::createTrackbar("armor_hw_ratio_max", WIN_NAME_ARMORP, &armor_param.armor_hw_ratio_max, 100);

    return true;
}

void ArmorSettings::readArmorParam(const char *param_path)
{
    cv::FileStorage fs(param_path, cv::FileStorage::READ);

    fs["armor_height"] >> armor_height;
    fs["armor_width"] >> armor_width;
    fs["small_armor_height"] >> small_armor_height;
    fs["small_armor_width"] >> small_armor_width;

    fs["gray_thre_min"] >> preprocess_param.gray_thre_min;
    fs["gray_thre_max"] >> preprocess_param.gray_thre_max;
    fs["gray_max_w"] >> preprocess_param.gray_max_w;
    fs["gray_avg_w"] >> preprocess_param.gray_avg_w;

    fs["len_min"] >> tgt_size_param.len_min;
    fs["len_max"] >> tgt_size_param.len_max;
    fs["light_ratio_min"] >> tgt_size_param.ratio_min;
    fs["light_ratio_max"] >> tgt_size_param.ratio_max;
    fs["light_area_min"] >> tgt_size_param.area_min;
    fs["light_area_max"] >> tgt_size_param.area_max;
    fs["area_ratio_min"] >> tgt_size_param.area_ratio_min;
    fs["area_ratio_max"] >> tgt_size_param.area_ratio_max;
    fs["area_len_ratio_min"] >> tgt_size_param.area_len_ratio_min;
    fs["area_len_ratio_max"] >> tgt_size_param.area_len_ratio_max;
    fs["corners_size_min"] >> tgt_size_param.corners_size_min;
    fs["slope_offset"] >> tgt_size_param.slope_offset;
    fs["color_th"] >> tgt_size_param.color_th;

    fs["delta_height_max"] >> armor_param.delta_height_max;
    fs["delta_angle_max"] >> armor_param.delta_angle_max;
    fs["delta_w_ratio_min"] >> armor_param.delta_w_ratio_min;
    fs["delta_w_ratio_max"] >> armor_param.delta_w_ratio_max;
    fs["delta_h_max"] >> armor_param.delta_h_max;

    fs["armor_hw_ratio_min"] >> armor_param.armor_hw_ratio_min;
    fs["armor_hw_ratio_max"] >> armor_param.armor_hw_ratio_max;


#ifdef DEBUG_MODE
    std::cout << "Read armor param finished!" << std::endl;
#endif
    fs.release();
}

void MainSettings::writeOtherParam(const char *param_path)
{
    cv::FileStorage fs(param_path, cv::FileStorage::WRITE);

    fs << "main_mode" << this->main_mode;
    fs << "enemy_color" << this->enemy_color;
    fs << "debug-b_show_src" << this->debug.b_show_src;
    fs << "debug-b_show_bin" << this->debug.b_show_bin;
    fs << "debug-b_show_target" << this->debug.b_show_target;
    fs << "debug-b_show_armor" << this->debug.b_show_armor;
    fs << "debug-b_show_result" << this->debug.b_show_result;
    fs << "debug-b_show_fps" << this->debug.b_show_fps;
    fs << "debug-b_save_result" << this->debug.b_save_result;
    fs << "debug-n_save_result" << this->debug.n_save_result;
    fs << "debug-b_save_pic" << this->debug.b_save_pic;
    fs << "debug-f_save_pic_inter" << this->debug.f_save_pic_inter;
    fs << "debug-expore_time" << this->debug.expore_time;
    fs << "debug-b_show_bin_rune" << this->debug.b_show_bin_rune;
    fs << "debug-b_show_td_rune" << this->debug.b_show_td_rune;
    fs << "debug-b_show_param_rune" << this->debug.b_show_param_rune;
    fs << "debug-b_show_cp_rune_result" << this->debug.b_show_cp_rune_result;
    fs << "debug-b_show_cp_rune" << this->debug.b_show_cp_rune;
    fs << "debug-b_show_center_rune" << this->debug.b_show_center_rune;
    fs << "debug-b_show_center_rune_result" << this->debug.b_show_center_rune_result;
    fs << "debug-b_show_center_distance" << this->debug.b_show_rune_center;
    fs << "debug-b_show_dc" << this->debug.b_show_dc;

    fs << "camera_param-x" << this->camera_param.x;
    fs << "camera_param-y" << this->camera_param.y;
    fs << "camera_param-z" << this->camera_param.z;
    fs << "camera_param-y_offset" << this->camera_param.y_offset;
    fs << "camera_param-z_ptz2bul" << this->camera_param.z_offset_ptz2bul;
    fs << "camera_param-bullet_speed" << this->camera_param.bullet_speed;

    fs << "port_param-dev_name" << this->port_param.dev_name;
    fs << "port_param-baud_rate" << this->port_param.baud_rate;
    fs << "port_param-databits" << this->port_param.databits;
    fs << "port_param-stopbits" << this->port_param.stopbits;
    fs << "port_param-parity" << this->port_param.parity;

    std::cout << "Sava other param finished!" << std::endl;
    fs.release();
}

void ArmorSettings::writeArmorParam(const char *param_path)
{
    cv::FileStorage fs(param_path, cv::FileStorage::WRITE);

    fs << "armor_height" << armor_height;
    fs << "armor_width" << armor_width;
    fs << "small_armor_height" << small_armor_height;
    fs << "small_armor_width" << small_armor_width;
    fs << "gray_thre_min" << preprocess_param.gray_thre_min;
    fs << "gray_thre_max" << preprocess_param.gray_thre_max;
    fs << "gray_max_w" << preprocess_param.gray_max_w;
    fs << "gray_avg_w" << preprocess_param.gray_avg_w;

    fs << "len_min" << tgt_size_param.len_min;
    fs << "len_max" << tgt_size_param.len_max;
    fs << "light_ratio_min" << tgt_size_param.ratio_min;
    fs << "light_ratio_max" << tgt_size_param.ratio_max;
    fs << "light_area_min" << tgt_size_param.area_min;
    fs << "light_area_max" << tgt_size_param.area_max;
    fs << "area_ratio_min" << tgt_size_param.area_ratio_min;
    fs << "area_ratio_max" << tgt_size_param.area_ratio_max;
    fs << "area_len_ratio_min" << tgt_size_param.area_len_ratio_min;
    fs << "area_len_ratio_max" << tgt_size_param.area_len_ratio_max;
    fs << "corners_size_min" << tgt_size_param.corners_size_min;
    fs << "slope_offset" << tgt_size_param.slope_offset;
    fs << "color_th" << tgt_size_param.color_th;

    fs << "delta_height_max" << armor_param.delta_height_max;
    fs << "delta_angle_max" << armor_param.delta_angle_max;
    fs << "delta_w_ratio_min" << armor_param.delta_w_ratio_min;
    fs << "delta_w_ratio_max" << armor_param.delta_w_ratio_max;
    fs << "delta_h_max" << armor_param.delta_h_max;

    fs << "armor_hw_ratio_min" << armor_param.armor_hw_ratio_min;
    fs << "armor_hw_ratio_max" << armor_param.armor_hw_ratio_max;

    std::cout << "Save armor param finished!" << std::endl;
    fs.release();
}

void MainSettings::setCameraParam(const char *win_name)
{
    cv::namedWindow(win_name);
    static int x = camera_param.x * 100.;
    cv::createTrackbar("cp-x", win_name, &x, 1000);
    camera_param.x = x / 100.;

    static int y = camera_param.y * 100.;
    cv::createTrackbar("cp-y", win_name, &y, 1000);
    camera_param.y = y / 100.;

    static int z = camera_param.z * 100.;
    cv::createTrackbar("cp-z", win_name, &z, 3000);
    camera_param.z = z / 100.;

    static int z_offset_ptz2bul = camera_param.z_offset_ptz2bul * 100.;
    cv::createTrackbar("cp-y_offset", win_name, &z_offset_ptz2bul, 3000);
    camera_param.z_offset_ptz2bul = z_offset_ptz2bul / 100.;

    static int y_offset = camera_param.y_offset * 100.;
    cv::createTrackbar("cp-y_offset", win_name, &y_offset, 1000);
    camera_param.y_offset = y_offset / 100.;

    static int overlap_dist = camera_param.overlap_dist * 1.;
    cv::createTrackbar("cp-overlap_dist", win_name, &overlap_dist, 1000000);
    camera_param.overlap_dist = overlap_dist / 1.;

    static int bullet_speed = camera_param.bullet_speed * 100.;
    cv::createTrackbar("cp-bullet_speed", win_name, &bullet_speed, 3000);
    camera_param.bullet_speed = bullet_speed / 100.;
}

bool MainSettings::grabImg(cv::Mat &img, char order, long interval)
{
    cv::Mat img_show = img.clone();
    //1 s
    //=1000       ms = 10e3 ms
    //=1000000    us = 10e6 us
    //=1000000000 ns = 10e9 ns
    static long start;
    static int img_filename;
    if(cv::getTickCount() - start >= interval)
    {
        start=cv::getTickCount();

        std::ostringstream s;
        s << SAVE_PIC_DIR << ++img_filename << ".png";
        cv::imwrite(s.str(), img);
        img_show = -img_show;
    }
    return true;
}
