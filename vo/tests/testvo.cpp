#include <iostream>
#include <unistd.h>

#include "vocoordinator.h"

using namespace std;

void testVO(){
    cout << "Testing Visual Odometry Component " << endl;
    VOCoordinator vo;
    vo.config();

    VideoCapture cap(0);
    Mat frame;
    while(!vo.kill){
        cap >> frame;
        vo.setCurrentFrame(frame);
        vo.runOnce();
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

