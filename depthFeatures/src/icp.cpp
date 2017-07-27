#include "icp.h"

ICP::ICP(){
    kill = false;
    config();
}

ICP::~ICP(){}

void ICP::config(){
    x = 0; y = 0; phi = 0;
    scanWindow.setPos(1000,10);
    scanWindow.resize(640, 480);
    int axisRange =8;
    scanWindow.axis(-axisRange, axisRange,-axisRange, axisRange);
    pathWindow.axis(-axisRange, axisRange,-axisRange, axisRange);
    scanWindow.axis_equal();

    currentScan.aperture = 2.351831 + 2.351831; // min and max angles
    currentScan.rightToLeft = true;
    currentScan.beamAperture = 0.004363; // angle increment
    currentScan.maxRange = 60;
    scannerPose.setFromValues(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    icpSolver.options.ICP_algorithm = TICPAlgorithm::icpClassic;
    icpSolver.options.maxIterations			= 200;
    icpSolver.options.thresholdAng			= DEG2RAD(0.1f);
    icpSolver.options.thresholdDist			= 0.75f;
    icpSolver.options.ALFA					= 0.9f;
    icpSolver.options.smallestThresholdDist	= 0.05f;
    icpSolver.options.doRANSAC = true;

}

void ICP::peroformICP(){
    //calculate the trasnformation to align both scans
    float runningTime; // store the returned values
    CICP::TReturnInfo info;
    CPose2D dummyPose;
    CPosePDFPtr pdf = icpSolver.Align(
                &previousMap,
                &currentMap,
                dummyPose,
                &runningTime, (void*)&info);

    CPosePDFGaussian  gPdf;
    gPdf.copyFrom(*pdf);

    incX = gPdf.getMeanVal().x();
    incY =  gPdf.getMeanVal().y();
    incPhi = gPdf.getMeanVal().phi();
    double dist = sqrt(incX*incX + incY*incY);
    x = x + dist * cos(phi) ;
    y = y + dist * sin(phi);
    phi += incPhi;
}

void ICP::setScan(vector<float> &ranges, vector<char> &valid){
    assert(valid.size() == ranges.size() && "The sizes of ranges and valids are not equal");
    float* r = &ranges[0];
    char* v = &valid[0];
    scanSize = ranges.size();
    if(previousMap.empty()){ // initialization
        previousScan.loadFromVectors(scanSize, r, v);
        previousMap.insertObservation(&previousScan);
    }
    currentScan.loadFromVectors(scanSize, r, v);
    currentMap.clear();
    currentMap.insertObservation(&currentScan);
}

void ICP::setPose(double &x, double &y, double &theta){

}

void ICP::visualize(){
    if(!scanWindow.isOpen()){
        kill = true;
        return;
    }


    if(pathx.empty()){
        pathx.push_back(x);
        pathy.push_back(y);
    }else{
        double dist = sqrt(incX*incX + incY*incY);
        if(dist > 0.01){ // only add to path if displacement is significant
            pathx.push_back(x);
            pathy.push_back(y);
        }
    }

    vector<float> map1_xs, map1_ys, map1_zs;
    vector<float> map2_xs, map2_ys, map2_zs;
    previousMap.getAllPoints(map1_xs,map1_ys,map1_zs);
    currentMap.getAllPoints(map2_xs,map2_ys,map2_zs);

    scanWindow.clf();
    scanWindow.plot( map1_xs, map1_ys, "r.1", "map1");
    scanWindow.hold_on();
    scanWindow.plot( map2_xs, map2_ys, "b.1", "map2");
    // replace previous map with current
    previousMap.clear();
    previousMap.insertObservation(&currentScan);
    pathWindow.clf();
    pathWindow.plot( pathx, pathy,"m.4","path");
}

void ICP::printInfo(){
    printf("ICP increments %f, %f, %f \t", incX, incY, incPhi);
    printf("Pose estimate %f, %f, %f ", x, y, phi);
    cout << endl;
}
