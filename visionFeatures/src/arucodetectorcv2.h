#ifndef ARUCODETECTORCV2_H
#define ARUCODETECTORCV2_H

// consider only if opencv2 
#if ! USE_OPENCV3 

#include <iostream>
#include <math.h>

// OpenCV2 specifics
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/core.hpp"
// Aruco specifics
#include "aruco/aruco.h"
#include "aruco/dictionary.h"
#include "aruco/markerlabeler.h"
#include "aruco/cvdrawingutils.h"
#include "aruco/exports.h"
#include "aruco/marker.h"
#include "aruco/ippe.h"
#include "aruco/levmarq.h"
#include "aruco/posetracker.h"

#include "eigen3/Eigen/Eigen"



using namespace std;
using namespace cv;
using namespace aruco;
using namespace Eigen;


class arucoDetectorCV2
{
    //Frame variables
    Mat currentFrame;
    Mat previousFrame;
    Mat dispFrame;
    int frameCounter;

    //configuration parameters
    string configFileName;
    bool displayImage;
    Mat intrinsicParams, distortionCoeff;
    double markerSize;

    //detection variables
    vector<double> markerX, markerZ;
    Matrix<float, Dynamic, 2> centers;
    vector<int> ids;

    //Aruco specifics
    aruco::MarkerDetector markerDetector;

    //private member functions
    void printFrameInfo(Mat &frame);

public:

    arucoDetectorCV2();
    virtual ~arucoDetectorCV2();

    bool kill;
    void config();
    void detectMarkers();
    void visualize();

    //setters
    void setCurrentFrame(const Mat &frame);
    void setIntrinsicParams(const Mat &value);


    //getters
    Mat getIntrinsicParams() const;
    void getDetections(vector<double> &x, vector<double> &z, vector<int> &id);
    Mat getCurrentFrame() const;
};

#endif
#endif // ARUCODETECTORCV2_H
