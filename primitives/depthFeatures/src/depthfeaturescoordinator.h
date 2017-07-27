#ifndef DEPTHFEATURESCOORDINATOR_H
#define DEPTHFEATURESCOORDINATOR_H

#include <vector>
#include <iostream>
#include <string>

#include "scanfuser.h"


using namespace std;

class DepthFeaturesCoordinator
{
    ScanFuser sf;
//    scanViz viz;
//    ICP icp;

public:
    DepthFeaturesCoordinator();
    virtual ~DepthFeaturesCoordinator();
    void config();
    void runOnce();
    void setScan(vector<float> &ranges,  vector<char> &valid);
    void setPose(double &x , double &y, double &theta);
    bool kill;
};

#endif // DEPTHFEATURESCOORDINATOR_H
