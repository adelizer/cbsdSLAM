#ifndef VOCOORDINATOR_H
#define VOCOORDINATOR_H

#include "framematcher.h"

class VOCoordinator
{
    FrameMatcher fm;
public:
    VOCoordinator();
    virtual ~VOCoordinator();
    void config();
    void runOnce();
    bool kill;
    void setCurrentFrame(const Mat& newFrame);
};

#endif // VOCOORDINATOR_H
