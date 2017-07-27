/*
_            _           _     _                __           _
| |          | |         (_)   (_)              / _|         | |
| |_ ___  ___| |_  __   ___ ___ _  ___  _ __   | |_ ___  __ _| |_ _   _ _ __ ___  ___
| __/ _ \/ __| __| \ \ / / / __| |/ _ \| '_ \  |  _/ _ \/ _` | __| | | | '__/ _ \/ __|
| ||  __/\__ \ |_   \ V /| \__ \ | (_) | | | | | ||  __/ (_| | |_| |_| | | |  __/\__ \
 \__\___||___/\__|   \_/ |_|___/_|\___/|_| |_| |_| \___|\__,_|\__|\__,_|_|  \___||___/

  */


#include <iostream>
#include <unistd.h>

#include "visionfeaturescoordinator.h"

using namespace std;

void testVO(){
    cout << "Testing Vision Features Component " << endl;
    visionFeaturesCoordinator vfCoordinator;
    vfCoordinator.config();

    VideoCapture cap(0);
    Mat frame;
    while(!vfCoordinator.killVision){
        cap >> frame;
        vfCoordinator.setCurrentFrame(frame);
        vfCoordinator.runOnce();
        usleep(1000);
    }
}

int main(int argc, char **argv){
    try{
        testVO();
        cout << "Exited cleanly " <<  endl;
        return 0;
    }catch (std::exception &e){
        cout << "STD exception caught: " << e.what() << endl;
        return -1;
    }catch (...){
        printf("Untyped exception!!");
        return -1;
    }
    return 0;
}
