#include "gridmapviz.h"

GridMapViz::GridMapViz(){
    gridMap = COccupancyGridMap2D::Create();
    kill = false;
    config();
}

GridMapViz::~GridMapViz(){
    // save grid map as an image
}

void GridMapViz::config(){
    MRPT_LOG_DEBUG("in GridMapViz::config()");
    scanSize = 1;
    currentScan.aperture = 1.56466+1.5708;
    currentScan.rightToLeft = true;
    currentScan.beamAperture = 0.00613592;
    currentScan.maxRange = 5.6;
    currentScan.stdError = 0.1;
    scannerPose.setFromValues(0.0, 0.0, 0.0, 0.0, 0.0,0.0);
    MRPT_LOG_DEBUG("out GridMapViz::config()");
}

void GridMapViz::addToMap(){
    MRPT_LOG_DEBUG("in GridMapViz::addToMap()");
    gridMap->insertObservation(&currentScan, &scannerPose);
    MRPT_LOG_DEBUG("out GridMapViz::addToMap()");
}


void GridMapViz::setPose(double &x, double &y, double &theta){
    MRPT_LOG_DEBUG("in GridMapViz::setPose()");
    scannerPose.setFromValues(x, y, 0.0, theta, 0.0, 0.0);
    currentScan.setSensorPose(scannerPose);
    MRPT_LOG_DEBUG("out GridMapViz::setPose()");
}

void GridMapViz::setScan(vector<float> &ranges, vector<char> &valid){
    MRPT_LOG_DEBUG("in GridMapViz::setScan()");
    assert(valid.size() == ranges.size() && "The sizes of ranges and valids are not equal");
    float* r = &ranges[0];
    char* v = &valid[0];
    scanSize = ranges.size();
    currentScan.loadFromVectors(scanSize, r, v);
    MRPT_LOG_DEBUG("out GridMapViz::setScan()");
}

void GridMapViz::visualize(){
    MRPT_LOG_DEBUG("in GridMapViz::visualize()");
    if(!frame.isOpen()){
        kill = true;
        return;
    }
    CImage	img;
    gridMap->getAsImage(img);
    frame.showImage(img);
    MRPT_LOG_DEBUG("out GridMapViz::visualize()");
}


void GridMapViz::printInfo()
{

}
