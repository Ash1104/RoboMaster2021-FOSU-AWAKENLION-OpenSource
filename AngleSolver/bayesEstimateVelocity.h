#ifndef BAYESESTIMATEVELOCITY
#define BAYESESTIMATEVELOCITY

#include<iostream>
#include<vector>
class BayseEstimate {
public:
    BayseEstimate(float velocityMean = 17.0f, float velocityStd = 1.0f,
                  float goodProb = 0.9f, float badProb = 0.1f,
                  int strainCapacity = 1, int filterCapacity = 2,
                  std::string meanAndStdSavePath = "");

    /**
     * @brief  strTrim
     * @author author
     * @date   2018-
     */
    static std::string strTrim(std::string s);

    /**
     * @brief  getMean
     * @return velocityMean
     * @author author
     * @date   2018-
     */
    float getMean();

    /**
     * @brief  gaussian
     * @return gaussian
     * @author author
     * @date   2018-
     */
    float gaussian(float x, float u, float std);

    /**
     * @brief  bayesCompare
     * @param  x
     * @return gaussian()
     * @author 2018-
     */
    float bayesCompare(float x);

    /**
     * @brief  statisticAbnormalProb
     * @param  historyList
     * @return count / float(historyList.size())
     * @author author
     * @date   2018-
     */
    float statisticAbnormalProb(std::vector<std::vector<float>> historyList);

    /**
     * @brief  calMeanAndStd
     * @param  historyList
     * @param  onlyCalNormalVelocity
     * @return meanAndStd
     * @author author
     * @date   2018-
     */
    float *calMeanAndStd(std::vector<std::vector<float>>historyList, bool onlyCalNormalVelocity=false);

    /**
     * @brief  getbayesEstimateVelocity
     * @param  latestVelocity
     * @return estimateMean
     * @author author
     * @date   2018-
     */
    float getbayesEstimateVelocity(float latestVelocity);

private:
    float goodProb = 0.9;
    float badProb = 0.1;
    float velocityMean = 17;
    float velocityStd = 1.0;
    int updateParams[2] = {2,2};
    std::vector<std::vector<float>> bufferList;
    int runTimes = 0;
    std::string meanAndStdSavePath = "velocityMeanAndStd.txt";
    bool meanOrStdHaveUpdate = true;
};

#endif
