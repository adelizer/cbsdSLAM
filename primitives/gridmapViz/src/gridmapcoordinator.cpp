#include "gridmapcoordinator.h"

GridMapCoordinator::GridMapCoordinator(){
    kill = false;
}

GridMapCoordinator::~GridMapCoordinator(){}

void GridMapCoordinator::config(){
    grid.config();
}

void GridMapCoordinator::runOnce(){
    grid.addToMap();
    grid.visualize();

    kill = grid.kill;
}

void GridMapCoordinator::setScan(vector<float> &ranges, vector<char> &valid){
    grid.setScan(ranges, valid);
}

void GridMapCoordinator::setPose(double &x, double &y, double &theta){
    grid.setPose(x, y, theta);
}
