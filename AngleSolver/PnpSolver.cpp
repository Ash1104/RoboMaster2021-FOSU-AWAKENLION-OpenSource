#include "PnpSolver.h"

AngleSolver::AngleSolver(const char *camera_param_file_name, double z_scale, double min_dist, double max_dist)
{
    rectPnpSolver(camera_param_file_name);
    scale_z = z_scale;
    min_distance = min_dist;
    max_distance = max_dist;

    rot_camera2ptz = cv::Mat::eye(3, 3, CV_64FC1);
    trans_camera2ptz = cv::Mat::zeros(3, 1, CV_64FC1);
    offset_y_barrel_ptz = 0;
}

double AngleSolver::getPitRef()
{
    return pit_ref;
}

double AngleSolver::getYawRef()
{
    return yaw_ref;
}

void AngleSolver::setRelationPoseCameraPTZ(const double ptz_camera_x, const double ptz_camera_y,
                                           const double ptz_camera_z, double y_offset_barrel_ptz)
{

    double r_data[] = {1, 0, 0,
                       0, 1, 0,
                       0, 0, 1};

    cv::Mat rot_camera_ptz(3, 3, CV_64FC1, r_data);
    rot_camera_ptz.copyTo(rot_camera2ptz);

    // 云台相对于相机的偏移向量和平移向量
    double t_data[] = {ptz_camera_x, ptz_camera_y, -ptz_camera_z};
    cv::Mat trans_camera_ptz(3, 1, CV_64FC1, t_data);
    trans_camera_ptz.copyTo(trans_camera2ptz);

    offset_y_barrel_ptz = y_offset_barrel_ptz;
}

cv::Point2f AngleSolver::getImageCenter()
{
    return cv::Point2f(cam_matrix.at<double>(0, 2), cam_matrix.at<double>(1, 2));
}

//能量机关专用
bool AngleSolver::getAngleWithRectify(const cv::RotatedRect &rect,
                                      double wh_ratio,
                                      const cv::Point2f &offset)
{
    cv::RotatedRect rect_rectifid = rect;

    rect_rectifid.size.width = rect_rectifid.size.height / wh_ratio;
    setTargetSize(rect_rectifid.size.height, rect_rectifid.size.width);

    std::vector<cv::Point2f> target2d;
    //求物体坐标系相对于相机坐标系，求向量矩阵
    getTarget2dPoinstion(rect_rectifid, target2d, offset);
    return getAngleWithoutGavity(target2d);
}

//能量机关专用
void AngleSolver::adjustPTZ2BarrelWithoutGavity(const cv::Mat &pos_in_ptz, double &angle_x, double &angle_y)
{
    const double *xyz = (const double *)pos_in_ptz.data;
    double alpha = 0.0, theta = 0.0;

    angle_x = atan2(xyz[0], xyz[2]);

    alpha = asin(offset_y_barrel_ptz / sqrt(xyz[1] * xyz[1] + xyz[2] * xyz[2]));

    if (xyz[1] < 0)
    {
        theta = atan(-xyz[1] / xyz[2]);
        angle_y = - alpha - theta;
    }
    else
    {
        theta = atan(xyz[1] / xyz[2]);
        angle_y = -alpha + theta;
    }
    angle_x = angle_x * 180.0 / PI;
    angle_y = angle_y * 180.0 / PI;
}


void AngleSolver::solvePnP4Points_rune(const std::vector<cv::Point2f> &points2d, cv::Mat &rot, cv::Mat &trans)
{
    cv::Mat rvec;
    cv::solvePnP(point3d, points2d, cam_matrix, distortion_coeff, rvec, trans); //能量机关
    cv::Rodrigues(rvec, rot);
}

//能量机关专用
bool AngleSolver::getAngleWithoutGavity(std::vector<cv::Point2f> target2d)
{
    //根据检测出的目标在图像中的二维坐标，算出旋转矩阵与位移向量
    solvePnP4Points_rune(target2d, this->rot, position_in_camera);
    tranformationCamera2PTZ(position_in_camera, position_in_ptz);
    // 根据目标在PTZ坐标中的位置，计算偏移角度，使枪管瞄准目标
    adjustPTZ2BarrelWithoutGavity(position_in_ptz, yaw_ref,pit_ref);
    return true;
}

void AngleSolver::getTarget2dPoinstion(const cv::RotatedRect &rect,
    std::vector<cv::Point2f> &target2d,
    const cv::Point2f &offset)
{
    cv::Point2f vertices[4];
    rect.points(vertices);
    cv::Point2f lu, ld, ru, rd;
    std::sort(vertices, vertices + 4, [](const cv::Point2f & p1, const cv::Point2f & p2) { return p1.x < p2.x; });
    if (vertices[0].y < vertices[1].y)
    {
        lu = vertices[0];
        ld = vertices[1];
    }
    else
    {
        lu = vertices[1];
        ld = vertices[0];
    }
    if (vertices[2].y < vertices[3].y)
    {
        ru = vertices[2];
        rd = vertices[3];
    }
    else
    {
        ru = vertices[3];
        rd = vertices[2];
    }

    target2d.clear();
    target2d.push_back(lu + offset);
    target2d.push_back(ru + offset);
    target2d.push_back(rd + offset);
    target2d.push_back(ld + offset);
}

void AngleSolver::rectPnpSolver(const char *camera_param_file_name)
{
    cv::FileStorage fs(camera_param_file_name, cv::FileStorage::READ);
    if(!fs.isOpened())
        std::cout << "Could not open the configuration file" << std::endl;

    fs["Camera_Matrix"] >> cam_matrix;
    fs["Distortion_Coefficients"] >> distortion_coeff;
    fs["board_Width"] >> width_target;
    fs["board_Height"] >> height_target;

    if(cam_matrix.empty() || distortion_coeff.empty())
    {
        std::cout << "cam_matrix or distortion_coeff is empty!!!" << std::endl;
        return;
    }

    //根据目标矩形的宽高设置三维坐标
    double half_x = width_target / 2.0;
    double half_y = height_target / 2.0;

    point3d.push_back(cv::Point3f(-half_x, -half_y, 0));
    point3d.push_back(cv::Point3f(half_x, -half_y, 0));
    point3d.push_back(cv::Point3f(half_x, half_y, 0));
    point3d.push_back(cv::Point3f(-half_x, half_y, 0));
}

void AngleSolver::setTargetSize(double width, double height)
{
    width_target = width;
    height_target = height;

    //根据目标矩形的宽高设置三维坐标
    double half_x = width_target / 2.0;
    double half_y = height_target / 2.0;

    point3d.clear();
    point3d.push_back(cv::Point3f(-half_x, -half_y, 0));
    point3d.push_back(cv::Point3f(-half_x, half_y, 0));
    point3d.push_back(cv::Point3f(half_x, half_y, 0));
    point3d.push_back(cv::Point3f(half_x, -half_y, 0));
}

bool AngleSolver::getAngle(std::vector<cv::Point2f> target2d,
                           double bullet_speed, double current_ptz_angle, const cv::Point2f &offset)
{
    //根据检测出的目标在图像中的二维坐标，算出旋转矩阵与位移向量
    solvePnP4Points(target2d, this->rot, position_in_camera);

    position_in_camera.at<double>(0, 0) = scale_z * position_in_camera.at<double>(0, 0);
    position_in_camera.at<double>(1, 0) = scale_z * position_in_camera.at<double>(1, 0);
    position_in_camera.at<double>(2, 0) = scale_z * position_in_camera.at<double>(2, 0);
    //超出检测范围
    if (position_in_camera.at<double>(2, 0) < min_distance || position_in_camera.at<double>(2, 0) > max_distance)
    {
        std::cout << position_in_camera.at<double>(2, 0) << "out of range: [" << min_distance << ", " << max_distance << "]\n";
        return false;
    }

    // 相机坐标转换到PTZ坐标
    tranformationCamera2PTZ(position_in_camera, position_in_ptz);
    // 根据目标在PTZ坐标中的位置，计算偏移角度，使枪管瞄准目标
    adjustPTZ2Barrel(position_in_ptz, yaw_ref,pit_ref, bullet_speed, current_ptz_angle);
    return true;
}

void AngleSolver::tranformationCamera2PTZ(const cv::Mat &pos, cv::Mat &transed_pos)
{
    transed_pos = rot_camera2ptz * pos - trans_camera2ptz;
}

void AngleSolver::adjustPTZ2Barrel(const cv::Mat &pos_in_ptz,
    double &angle_x, double &angle_y,
    double bullet_speed,
    double current_ptz_angle)
{
    const double *_xyz = (const double *)pos_in_ptz.data;
    angle_x = atan2(_xyz[0], _xyz[2]);

    double xyz[3] = { _xyz[0], _xyz[1], _xyz[2] };

    double alpha = 0.0, theta = 0.0;

    alpha = asin(offset_y_barrel_ptz / sqrt(xyz[1] * xyz[1] + xyz[2] * xyz[2]));

    if (xyz[1] < 0)
    {
        theta = atan(-xyz[1] / xyz[2]);
        angle_y = - alpha - theta;
    }
    else
    {
        theta = atan(xyz[1] / xyz[2]);
        angle_y = -alpha + theta;
    }
    angle_x = angle_x * 180.0 / PI;
    angle_y = angle_y * 180.0 / PI;
}

void AngleSolver::solvePnP4Points(const std::vector<cv::Point2f> &points2d,
                                  cv::Mat &rot, cv::Mat &trans)
{
    cv::Mat rvec;
    cv::solvePnP(point3d, points2d, cam_matrix, distortion_coeff, rvec, trans/*, true, CV_P3P*/); //自瞄
    cv::Rodrigues(rvec, rot);
}

void AngleSolver::getDistanceDanmu(std::vector<cv::Point2f>armor_rect, double &dist)
{
    cv::RotatedRect armor = cv::minAreaRect(armor_rect);
    float p_w = std::max(armor.size.width, armor.size.height);
    float p_h = std::min(armor.size.width, armor.size.height);

    float fx_w = cam_matrix.at<double>(0, 0) * width_target;
    float fx_h = cam_matrix.at<double>(1, 1) * height_target;

    float dist_w = fx_w / p_w;
    float dist_h = fx_h / p_h;

    dist = (dist_w + dist_h) / 2;
}
