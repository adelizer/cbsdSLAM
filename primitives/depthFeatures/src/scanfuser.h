#ifndef SCANFUSER_H
#define SCANFUSER_H

#include <vector>
#include <iostream>
#include <string>

#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/slam/CICP.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/gui.h>
#include <mrpt/math/utils.h>
#include <mrpt/gui/CDisplayWindowPlots.h>
#include <mrpt/slam/CRangeBearingKFSLAM2D.h>
 #include <mrpt/utils/COutputLogger.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace mrpt::gui;
using namespace std;
using namespace mrpt::bayes;

class ScanFuser : COutputLogger
{
    CDisplayWindowPlots scanWindow;
    CDisplayWindow frame;
    int scanSize;
    CObservation2DRangeScan currentScan;
    CPose3D scannerPose;

    COccupancyGridMap2DPtr gridMap;

    CSimplePointsMapPtr map;
public:
    ScanFuser();
    virtual ~ScanFuser();
    void config();
    void addToMap();
    void setPose(double &x , double &y, double &theta);
    void setScan(vector<float> &ranges,  vector<char> &valid);
    void visualize();
    void printInfo();

    bool kill;
};

#endif // SCANFUSER_H
