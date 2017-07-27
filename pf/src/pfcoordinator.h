#ifndef PFCOORDINATOR_H
#define PFCOORDINATOR_H

#include "mrptpf.h"
#include "visualizer.h"

class PFCoordinator
{
    mrptPF *pf;
    Visualizer viz;
public:
    PFCoordinator();
    virtual ~PFCoordinator();
    void config();
    void runOnce();
    void setInput(double x, double y, double theta);
    void setObservations(vector<double>& x, vector<double>& y, vector<double>& z, vector<int>& ids);
    void setObservations(vector<float> &ranges, vector<char> &valid);
    void getPose(double &x, double &y, double &theta);

    bool kill;
};

#endif // PFCOORDINATOR_H
