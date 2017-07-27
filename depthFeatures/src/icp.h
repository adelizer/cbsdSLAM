#ifndef ICP_H
#define ICP_H

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


class ICP
{
    CDisplayWindowPlots scanWindow;
    CDisplayWindowPlots pathWindow;
    int scanSize;
    CObservation2DRangeScan currentScan;
    CObservation2DRangeScan previousScan;
    CPose3D scannerPose;
    CPose2D s;
    CICP icpSolver;
    CSimplePointsMap currentMap, previousMap;

    float incX, incY, incPhi, x, y, phi;
    vector<double> pathx, pathy;

public:
    ICP();
    virtual ~ICP();
    void config();
    void peroformICP();
    void setScan(vector<float> &ranges,  vector<char> &valid);
    void setPose(double &x , double &y, double &theta);
    void visualize();
    void printInfo();

    bool kill;
};

#endif // ICP_H
