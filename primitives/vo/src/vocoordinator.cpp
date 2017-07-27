#include "vocoordinator.h"

VOCoordinator::VOCoordinator(){

}

VOCoordinator::~VOCoordinator(){}

void VOCoordinator::config(){
    fm.config();
}

void VOCoordinator::runOnce(){
    fm.runVO();
    fm.visualize();
    fm.printInfo();
    kill = fm.kill;
}

void VOCoordinator::setCurrentFrame(const Mat &newFrame){
    fm.setCurrentFrame(newFrame);
}
