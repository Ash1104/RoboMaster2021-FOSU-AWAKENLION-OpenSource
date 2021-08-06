#include "bayesEstimateVelocity.h"
#include<math.h>
#include<fstream>
#include<sstream>
using namespace std;

BayseEstimate::BayseEstimate(float velocityMean, float velocityStd, float goodProb,
                             float badProb, int strainCapacity, int filterCapacity,
                             std::string meanAndStdSavePath)
{
    this->goodProb = goodProb;
    this->badProb = badProb;
    this->velocityMean = velocityMean;
    this->velocityStd = velocityStd;
    this->updateParams[0] = strainCapacity;
    this->updateParams[1] = filterCapacity;
    this->meanAndStdSavePath = meanAndStdSavePath;
    stringstream stream;
    ifstream in(this->meanAndStdSavePath);
    string line;
    if(in)
    {
        while(getline(in, line))
        {
            if(strTrim(line)!= "")
            {
                int index = line.find(' ');
                this->velocityMean = atof(line.substr(0, index).c_str());
                this->velocityStd = atof(line.substr(index + 1).c_str());
                break;
            }
        }
    }
}

std::string BayseEstimate::strTrim(std::string s)
{
    int beginIndex = -1;
    int endIndex = -1;
    for(size_t i = 0; i < s.length(); i++)
    {
        if (s[i] != ' ' && s[i] != '\n')
        {
            beginIndex = i;
            break;
        }
    }
    for(size_t i = s.length() - 1; i >= 0; i--)
    {
        if(s[i] != ' ' && s[i] != '\n')
        {
            endIndex = i;
            break;
        }
    }
    if(beginIndex != -1)
        return s.substr(beginIndex, endIndex - beginIndex + 1);

    return "";
}

float BayseEstimate::getMean()
{
    return velocityMean;
}

float BayseEstimate::gaussian(float x, float u, float std)
{
    if(std <= 0) std = 0.1f;
    return expf(-((x - u) * (x-u)) / (2.0 * (std * std)));
}

float BayseEstimate::bayesCompare(float x)
{
    return gaussian(x, velocityMean, velocityStd) * goodProb - (1 - gaussian(x, velocityMean, velocityStd * 1.2)) * badProb;
}

float BayseEstimate::statisticAbnormalProb(vector<vector<float>> historyList){
    int count = 0;
    for(int i = 0; i < historyList.size(); i++)
    {
        if(historyList[i][1] < 0) count++;
    }
    return count / float(historyList.size());
}

float *BayseEstimate::calMeanAndStd(vector<vector<float>> historyList, bool onlyCalNormalVelocity)
{
    static float meanAndStd[2];
    float mean = 0;
    float std = 0;
    int count = 0;
    for(int i = 0; i < historyList.size(); i++)
    {
        if(onlyCalNormalVelocity && historyList[i][1] < 0)
            continue;
        mean += historyList[i][0];
        count++;
    }
    mean /= count;
    for(int i = 0; i < historyList.size(); i++)
    {
        if(onlyCalNormalVelocity && historyList[i][1] < 0)
            continue;
        std += ((historyList[i][0] - mean) * (historyList[i][0] - mean));
    }
    if (count == 1) std /= count;
    else std /= (count - 1);
    meanAndStd[0] = mean;
    meanAndStd[1] = sqrtf(std);
    return meanAndStd;
}

float BayseEstimate::getbayesEstimateVelocity(float latestVelocity)
{
    float estimateMean = velocityMean;
    float estimateStd = velocityStd;
    if(bufferList.size() == 0 || latestVelocity != bufferList[bufferList.size() - 1][0])
    {
        float bayesCompareValue = bayesCompare(latestVelocity);
        if(bayesCompareValue != 0)
        {
            vector<float>velocityParam = {latestVelocity, bayesCompareValue};
            bufferList.push_back(velocityParam);
            if(bufferList.size() % updateParams[0] == 0)
            {
                if(statisticAbnormalProb(bufferList) >= 0.01)
                {
                    float *estimateMeanAndValue = calMeanAndStd(bufferList, false);
                    estimateMean = estimateMeanAndValue[0];
                    estimateStd = estimateMeanAndValue[1];
                    velocityMean = estimateMean;
                    velocityStd = estimateStd;
                    bufferList.clear();
                    meanOrStdHaveUpdate = true;
                }
                else
                {
                    if(bufferList.size() % updateParams[1] == 0)
                    {
                        float *estimateMeanAndValue = calMeanAndStd(bufferList, true);
                        estimateMean = estimateMeanAndValue[0];
                        estimateStd = estimateMeanAndValue[1];
                        velocityMean = estimateMean;
                        velocityStd = estimateStd;
                        meanOrStdHaveUpdate = true;
                    }
                }

            }
        }
        if(runTimes % 5 == 0 && meanOrStdHaveUpdate == true && this->meanAndStdSavePath != "")
        {
            ofstream OutFile(meanAndStdSavePath);
            OutFile << to_string(velocityMean) + " " + to_string(velocityStd);
            OutFile.close();
            meanOrStdHaveUpdate = false;
        }
        runTimes++;
    }
    return estimateMean;
}
