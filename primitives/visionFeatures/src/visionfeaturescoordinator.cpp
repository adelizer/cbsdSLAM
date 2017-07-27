#include "visionfeaturescoordinator.h"

visionFeaturesCoordinator::visionFeaturesCoordinator(){
    killVision = false;
}

visionFeaturesCoordinator::~visionFeaturesCoordinator(){}

void visionFeaturesCoordinator::config(){
     markerD.config();

    // vision coordinator specific configuration
}

void visionFeaturesCoordinator::run(){

}

void visionFeaturesCoordinator::runOnce(){

    //change according to method required
          markerD.detectMarkers();
    	  killVision = markerD.kill;

}

void visionFeaturesCoordinator::visualize(){
    markerD.visualize();
}

void visionFeaturesCoordinator::setCurrentFrame(const Mat &newFrame)
{
    // pass on the frame to the extractor
         markerD.setCurrentFrame(newFrame);

}

void visionFeaturesCoordinator::getDetections(vector<double> &x, vector<double> &z, vector<int> &id)
{
    markerD.getDetections(x, z, id);
}

