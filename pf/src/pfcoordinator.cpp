#include "pfcoordinator.h"

PFCoordinator::PFCoordinator(){
    CMetricMapBuilderRBPF::TConstructionOptions  newOptions;

    CConfigFile iniFile("/home/adel/workspace/slam/primitives/pf/config/config.ini");

    newOptions.loadFromConfigFile(iniFile, "RBPFSLAM");
    pf = new mrptPF(newOptions);
    kill = false;
    config();
}

PFCoordinator::~PFCoordinator(){}

void PFCoordinator::config(){
    viz.config();
    viz.update(pf); //
}

void PFCoordinator::runOnce(){
    pf->doProcess();
    viz.showParticles();
    viz.showGrid();
    viz.showLandmarks();
    kill = viz.checkKill();
    if(kill)
        pf->~mrptPF();
}

void PFCoordinator::setInput(double x, double y, double theta){
    pf->setInput(x, y, theta);
}

void PFCoordinator::setObservations(vector<double> &x, vector<double> &y, vector<double> &z, vector<int> &ids){
    pf->setObservations(x, y, z, ids);
}

void PFCoordinator::setObservations(vector<float> &ranges, vector<char> &valid){
    pf->setObservations(ranges, valid);
}

void PFCoordinator::getPose(double &x, double &y, double &theta){
    pf->getPose(x,y,theta);
}

