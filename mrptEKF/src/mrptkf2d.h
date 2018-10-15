#ifndef MRPTKF2D_H
#define MRPTKF2D_H

#include <vector>

#include <mrpt/system/os.h>
#include <mrpt/system/threads.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/math/distributions.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/CObservationBearingRange.h>
#include <mrpt/slam/CRangeBearingKFSLAM2D.h>
#include <mrpt/gui/CDisplayWindowPlots.h>
#include <mrpt/math/distributions.h>
#include <mrpt/utils/CConfigFile.h>
#include <Eigen/Core>

using namespace mrpt;
using namespace mrpt::bayes;
using namespace mrpt::gui;
using namespace mrpt::math;
using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::poses;
using namespace mrpt::maps;
using namespace std;

class mrptKF2d : public mrpt::slam::CRangeBearingKFSLAM2D
{
    string configFileName;
    KFArray_ACT input;
    CDisplayWindowPlots winSLAM2d;
    vector<double> pathx, pathy;
    CSensoryFramePtr observations;
    CActionCollectionPtr actions;
    KFVector alpha;

    double minSensorRange, maxSensorRange;
    double rangeNoise, bearingNoise;
    double x,y,theta,v,w;
    double deltaTime;

public:
    mrptKF2d();
    ~mrptKF2d();
    void config();
    void setInput(double x, double y, double theta); // <--- Entry point to inputs
    void setObservations(vector<double>& x, vector<double>& y, vector<double>& z, vector<int>& ids); // <--- Entry point to observations
    void doProcess(); //<--- Entry point
    void visualize();
    void printInfo();
    void getPose(double &x, double &y, double &theta);
    bool kill;

protected:
    void OnGetAction( KFArray_ACT &out_u ) const;

    void OnTransitionModel(
            const KFArray_ACT &in_u,
            KFArray_VEH       &inout_x,
            bool &out_skipPrediction
            ) const;

    void OnTransitionJacobian(KFMatrix_VxV  &out_F ) const;

    void OnTransitionNoise(KFMatrix_VxV &out_Q ) const;

    void OnGetObservationNoise(KFMatrix_OxO &out_R) const;

    void OnNormalizeStateVector();
};

#endif // MRPTKF2D_H
