#include "GravityCompensateResolve.h"
#include<math.h>

using namespace std;
GravityCompensateResolve::GravityCompensateResolve(float velocityMean, float velocityStd,
                                                   float goodProb, float badProb,
                                                   int strainCapacity, int filterCapacity,
                                                   std::string meanAndStdSavePath)
{
    bayseEstimate = new BayseEstimate(velocityMean, velocityStd, goodProb, badProb, strainCapacity, filterCapacity, meanAndStdSavePath);
}

float GravityCompensateResolve::solveAngleWithGravity(float cameraSolveAngle, float gunPitchAngleOffsetOfHorizon,
                                                      float absoluteDistance, float latestVelocity)
{
    float decentAngle = (cameraSolveAngle + gunPitchAngleOffsetOfHorizon) * PI / 180.0f;
    printf("dist=%f\n", absoluteDistance);
    float relativeDistance = absoluteDistance * cos(decentAngle);
    float height = absoluteDistance * sin(decentAngle);

    if(latestVelocity < 11)
        latestVelocity = bayseEstimate->getMean() + 5;

    float velocity = fabs(bayseEstimate->getbayesEstimateVelocity(latestVelocity - 5));
    printf("bayesVelocity=%f\n", velocity);

    float a = (g * relativeDistance * relativeDistance) / (2.0f * velocity * velocity);
    float b = relativeDistance * relativeDistance - 4.0 * a * (height + a);
    if(b < 0)
        return cameraSolveAngle;

    float x = (relativeDistance - sqrt(b)) / (2.0 * a);
    float angle = atan(x) * 180.0f / PI;
    printf("solve=%f\n",angle);

    return angle - gunPitchAngleOffsetOfHorizon;
}

float GravityCompensateResolve::solveAngleWithGravity(float decentAngle,
                                                      float absoluteDistance)
{
    decentAngle = decentAngle * PI / 180.0f;
    float relativeDistance = absoluteDistance * cos(decentAngle);
    float height = absoluteDistance * sin(decentAngle);
    float velocity = 27;
    float a = (g * relativeDistance * relativeDistance) / (2.0f * velocity * velocity);
    float b = relativeDistance * relativeDistance - 4.0 * a * (height + a);
    if (b >= 0)
    {
        float x = (relativeDistance - sqrt(b)) / (2.0 * a);
        float angle = atan(x) * 180.0f / PI;

        return angle;
    }
    else  // 无解，只能选原来角度
        return decentAngle;
}

double GravityCompensateResolve::solveAngleWithGravity(double decentAngle, double absoluteDistance,
                                                      float velocity)
{
    decentAngle = decentAngle * PI / 180.0f;
    double relativeDistance = absoluteDistance * cos(decentAngle);
    double height = absoluteDistance * sin(decentAngle);

    double a = (g * relativeDistance * relativeDistance) / (2.0f * velocity * velocity);
    double b = relativeDistance * relativeDistance - 4.0 * a * (height + a);
    if (b >= 0)
    {
        double x = (relativeDistance - sqrt(b)) / (2.0 * a);
        double angle = atan(x) * 180.0 / PI;
        return angle;
    }
    else  // 无解，只能选原来角度
        return decentAngle;
}

