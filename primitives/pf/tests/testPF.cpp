#include <unistd.h>
#include <iostream>

#include "pfcoordinator.h"

using namespace std;

void readScan(vector<vector<float> > &data, vector<vector<char> > &valid);

void testPF(){
    cout << "Testing MRPT PF component " << endl;
    PFCoordinator coord;
    vector<double> x, y, z;
    vector<int> ids;
    x.push_back(0); z.push_back(2); y.push_back(1); ids.push_back(14);
    int counter = 0;

    vector<vector<float> > data;
    vector<vector<char> > valid;

    readScan(data, valid);
    cout << "Scan file has been parsed" << endl;
    cout << "The file contains " <<  data.size() << " each scan is " << data[0].size() << endl;


    while(!coord.kill){
        coord.setInput(0.01, 0.0 , 0.00);
        vector<float> r = data[0];
        vector<char> v = valid[0];
        coord.setObservations(r, v);
        if(counter > 10)
            coord.setObservations(x,y,z,ids);
        coord.runOnce();
        usleep(100000);
        counter++;
    }
}

int main(int argc, char **argv){
    try{
        testPF();
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


void readScan(vector<vector<float> >& data, vector<vector<char> > &valid){
    ifstream infile( "../tests/data/csvScan.dat" );

    while (infile){
        string s;
        if (!getline( infile, s )) break;

        istringstream ss( s );
        vector <float> record;
        vector<char> v;

        while (ss){
            string s;
            if (!getline( ss, s, ',' )) break;
            float range = stod(s);
            if(range < 60 && range > 0.02)
                v.push_back(1);
            else
                v.push_back(0);
            record.push_back(range);
        }

        data.push_back(record);
        valid.push_back(v);
    }
    if (!infile.eof()){
        cerr << "Error!\n";
    }
}

