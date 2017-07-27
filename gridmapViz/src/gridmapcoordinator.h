#ifndef GRIDMAPCOORDINATOR_H
#define GRIDMAPCOORDINATOR_H

#include "gridmapviz.h"

class GridMapCoordinator
{
    GridMapViz grid;
public:
    GridMapCoordinator();
    virtual ~GridMapCoordinator();
    void config();
    void runOnce();
    void setScan(vector<float> &ranges,  vector<char> &valid);
    void setPose(double &x , double &y, double &theta);
    bool kill;
};

#endif // GRIDMAPCOORDINATOR_H
