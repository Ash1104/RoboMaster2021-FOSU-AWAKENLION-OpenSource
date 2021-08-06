#ifndef GRAVITYCOMPENSATERESOLVE_H
#define GRAVITYCOMPENSATERESOLVE_H
#include<iostream>
#include "bayesEstimateVelocity.h"
#define PI 3.14159265358979323846

class GravityCompensateResolve{
public:
    //构造函数
    GravityCompensateResolve(float velocityMean = 17.0f, float velocityStd = 1.0f,
                             float goodProb = 0.9f, float badProb = 0.1f,
                             int strainCapacity = 2, int filterCapacity = 3,
                             std::string meanAndStdSavePath = "velocityMeanAndStd.txt");

    /**
     * @brief  重力补偿（重载1）需要枪口相对水平线偏差角
     * @return 补偿角度
     * @author 参与开发人员
     * @date   2018-
     */
    float solveAngleWithGravity(float cameraSolveAngle, float gunPitchAngleOffsetOfHorizon,
                                float absoluteDistance, float velocity);

    /**
     * @brief  重力补偿（重载2）默认射速27
     * @return 补偿角度
     * @author 参与开发人员
     * @date   2018-
     */
    float solveAngleWithGravity(float decentAngle, float absoluteDistance);

    /**
     * @brief  重力补偿（重载3）常用
     * @return 补偿角度（double）
     * @author 参与开发人员
     * @date   2018-
     */
    double solveAngleWithGravity(double decentAngle, double absoluteDistance,
                                 float latestVelocity);

private:
    float g = 9.7887;              // 重力加速度  深圳重力加速度大概是 9.79.
    BayseEstimate *bayseEstimate;  // 贝叶斯估计
};
#endif
