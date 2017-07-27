#ifndef MRPTPF_H
#define MRPTPF_H

#include <vector>

#include <mrpt/system/os.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/threads.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/math/distributions.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/CObservationBearingRange.h>
#include <mrpt/obs/CObservationBeaconRanges.h>
#include <mrpt/obs/CObservationRange.h>
#include <mrpt/slam/CRangeBearingKFSLAM2D.h>
#include <mrpt/slam/CMetricMapBuilderRBPF.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/random.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/utils/CConfigFile.h>

// GUI and stream headers
#include <mrpt/gui/CDisplayWindowPlots.h>
#include <mrpt/gui.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CEllipsoid.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/utils/CConfigFile.h>

// Eigen headers
#include <unsupported/Eigen/MatrixFunctions>
#include <Eigen/Cholesky>


using namespace mrpt;
using namespace mrpt::bayes;
using namespace mrpt::gui;
using namespace mrpt::math;
using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::poses;
using namespace mrpt::maps;
using namespace mrpt::opengl;
using namespace mrpt::system;
using namespace mrpt::random;
using namespace std;
using namespace Eigen;

class Visualizer; // forward declaration

// creates a class that extends the MRPT rbpf class
// Functions can not be overriden
class mrptPF : public CMetricMapBuilderRBPF
{
    CActionRobotMovement2D::TMotionModelOptions odo_opts;  // options for odo input
    CMatrixDouble22 observationNoise; // Observation noise in the range bearing sensor
    CSensoryFramePtr observations; // container for all observations
    CActionCollectionPtr actions; //container for all actions
    CObservation2DRangeScanPtr scanSensor;
    double scanSize;

    // Variables to store the current data after each iteration
    CPose3DPDFPtr currentPDFptr;
    CPose3DPDFParticles currentPDF;
    CPose3D currentPose;
    COccupancyGridMap2D::TEntropyInfo entropy;
    const CMultiMetricMap *mostLikMap;
    const CMultiMetricMap *currentMap; // probably the same as the most likely map

    // bool to check if the map type is suitabl for observation
    bool pointsMapAvailable;
    bool landmarksMapAvailable;
    bool gridMapAvailable;


    vector<int> observedLandmarks;

    long int step;

    // Display variables
    friend class Visualizer;

    double computeWeight();
    void wrapAngle(double &angle);
    int getLandmarkOrder(int id, int particleID);
    void updateParticle(double range, double bearing, int id, int i);
    void testBeaconMap();
public:
    mrptPF(CMetricMapBuilderRBPF::TConstructionOptions  newOptions);
    virtual ~mrptPF();
    void config();
    void setInput(double x, double y, double theta);
    void setObservations(vector<double>& x, vector<double>& y, vector<double>& z, vector<int>& ids);
    void setObservations(vector<float> &ranges, vector<char> &valid);
    void getPose(double &x, double &y, double &theta);
    void doProcess();
    bool kill;
    CPose3DPDFParticles getCurrentPDF() const;
    void setCurrentPDF(const CPose3DPDFParticles &value);
};

#endif // MRPTPF_H
