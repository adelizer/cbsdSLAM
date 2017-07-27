#ifndef VISUALIZER_H
#define VISUALIZER_H

#include "mrptpf.h"

class Visualizer
{
    mrptPF *pf;
    CDisplayWindowPlots winSLAM2d;

    // Display variables
    CDisplayWindow frame;
    CDisplayWindow3DPtr win3D;
    COpenGLScenePtr scene;
    void printRandom();

public:
    Visualizer();
    virtual ~Visualizer();
    void config();
    void showParticles();
    void showGrid();
    void showPointsMap();
    void show3D();
    void showLandmarks();
    void update(mrptPF *in){pf = in;}
    bool checkKill();
    void printInfo();
};

#endif // VISUALIZER_H
