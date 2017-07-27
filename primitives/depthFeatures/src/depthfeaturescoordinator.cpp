#include "depthfeaturescoordinator.h"

DepthFeaturesCoordinator::DepthFeaturesCoordinator(){
    kill = false;
    config();
}

DepthFeaturesCoordinator::~DepthFeaturesCoordinator(){}

void DepthFeaturesCoordinator::config(){
    sf.config();
}

void DepthFeaturesCoordinator::runOnce(){
        sf.addToMap();
        sf.visualize();
//        sf.printInfo();
    //    viz.visualize();
//    icp.peroformICP();
//    icp.visualize();
//    icp.printInfo();
    kill = sf.kill;
}

void DepthFeaturesCoordinator::setScan(vector<float> &ranges, vector<char> &valid){
        sf.setScan(ranges, valid);
//    icp.setScan(ranges, valid);
}

void DepthFeaturesCoordinator::setPose(double &x, double &y, double &theta){
    sf.setPose(x, y, theta);
}
