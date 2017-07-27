#ifndef GRIDMAPVIZ_H
#define GRIDMAPVIZ_H

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


class GridMapViz : COutputLogger
{
    CDisplayWindow frame;

    CObservation2DRangeScan currentScan;
    CPose3D scannerPose;
    int scanSize;

    COccupancyGridMap2DPtr gridMap;
    CSimplePointsMapPtr map;

public:
    GridMapViz();
    virtual ~GridMapViz();
    void config();
    void addToMap();
    void visualize();
    void setPose(double &x , double &y, double &theta);
    void setScan(vector<float> &ranges,  vector<char> &valid);
    void printInfo();
    bool kill;
};

#endif // GRIDMAPVIZ_H
