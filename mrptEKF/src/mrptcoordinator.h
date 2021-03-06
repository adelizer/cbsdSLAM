#ifndef MRPTCOORDINATOR_H
#define MRPTCOORDINATOR_H

#include "mrptkf2d.h"

class mrptCoordinator
{
    mrptKF2d ekfSLAM;
public:
    mrptCoordinator();
    ~mrptCoordinator();
    void config();
    void runOnce();
    void setInput(double x, double y, double theta);
    void setObservations(vector<double>& x, vector<double>& y, vector<double>& z, vector<int>& ids);
    void getPose(double &x, double &y, double &theta);
    bool kill;
};

#endif // MRPTCOORDINATOR_H
