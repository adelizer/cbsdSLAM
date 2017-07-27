#ifndef FRAMEMATCHER_H
#define FRAMEMATCHER_H

// header to all helper functions
#include "vo_utils.h"

class FrameMatcher
{
    // Frame variables
    Mat currentFrame;
    Mat dispFrame;
    Mat pathFrame;

    string configFileName;
    Mat intrinsicParams, distortionCoeff;
    bool displayImage;

    Ptr<BriefDescriptorExtractor> brief = BriefDescriptorExtractor::create(32);
    vector<Point2f> train_pts, query_pts;
    vector<KeyPoint> train_kpts, query_kpts;
    vector<unsigned char> match_mask;
    Mat frame;

    vector<float> pathX, pathY, pathTheta;

    vector<DMatch> matches;

    BFMatcher desc_matcher;
    Mat train_desc, query_desc;
    Ptr<FastFeatureDetector> detector;
    Mat H_prev;
    double x, y, z, theta;

    bool initDone;

public:
    FrameMatcher();
    virtual ~FrameMatcher();
    bool kill;

    void config();
    void setCurrentFrame(const Mat &frame);
    void runVO();
    void printInfo();
    void visualize();
};

#endif // FRAMEMATCHER_H
