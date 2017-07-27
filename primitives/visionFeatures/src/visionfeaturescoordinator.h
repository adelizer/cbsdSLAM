#ifndef VISIONFEATURESCOORDINATOR_H
#define VISIONFEATURESCOORDINATOR_H

#include "arucodetectorcv2.h"


class visionFeaturesCoordinator
{
//    visionFeatureExtractor extractor;
    arucoDetectorCV2 markerD;
//    VisualOdometry vo;
//    FrameMatcherVO fm;

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
