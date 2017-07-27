#include "mrptcoordinator.h"

mrptCoordinator::mrptCoordinator(){
    config();
}

mrptCoordinator::~mrptCoordinator(){}

void mrptCoordinator::config(){
    ekfSLAM.config();
}

void mrptCoordinator::runOnce(){
    ekfSLAM.doProcess();
    ekfSLAM.visualize();
    //ekfSLAM.printInfo();
    kill = ekfSLAM.kill;
}

void mrptCoordinator::setInput(double x, double y, double theta){
    ekfSLAM.setInput(x, y, theta);
}

void mrptCoordinator::setObservations(vector<double> &x, vector<double> &y, vector<double> &z, vector<int> &ids){
    ekfSLAM.setObservations(x, y, z, ids);
}

void mrptCoordinator::getPose(double &x, double &y, double &theta){
    ekfSLAM.getPose(x, y, theta);
}
