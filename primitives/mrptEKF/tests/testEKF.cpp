#include <unistd.h>
#include <iostream>

#include "mrptcoordinator.h"

using namespace std;


void testEKF(){
    cout << "Testing MRPT EKF component " << endl;
    mrptCoordinator m;
    int counter = 0;
    vector<double> x, y, z;
    vector<int> ids;
    x.push_back(1); y.push_back(1); z.push_back(1);
    ids.push_back(1);
    while(!m.kill){
        m.setInput(0.1, 0, 0);
        if(counter == 100){
            m.setObservations(x,y,z,ids);
        }
        m.runOnce();
        usleep(10000);
        counter++;
    }
}

int main(int argc, char **argv){
    try{
        testEKF();
        cout << "Exited cleanly " <<  endl;
        return 0;
    } catch (std::exception &e){
        std::cout << "MRPT exception caught: " << e.what() << std::endl;
        return -1;
    }
    catch (...){
        printf("Untyped exception!!");
        return -1;
    }
    return 0;
}
