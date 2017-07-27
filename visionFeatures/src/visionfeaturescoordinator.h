#ifndef VISIONFEATURESCOORDINATOR_H
#define VISIONFEATURESCOORDINATOR_H

#include "arucodetectorcv2.h"


class visionFeaturesCoordinator
{

    arucoDetectorCV2 markerD;


public:
    visionFeaturesCoordinator();
    ~visionFeaturesCoordinator();
    void config();
    void run();
    void runOnce();
    void visualize();
    bool killVision;
    void setCurrentFrame(const Mat& newFrame);
    void getDetections(vector<double> &x, vector<double> &z, vector<int> &id);


};

#endif // VISIONFEATURESCOORDINATOR_H
