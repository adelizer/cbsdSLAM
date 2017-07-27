#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <unistd.h>

#include "depthfeaturescoordinator.h"

using namespace std;

void readScan(vector<vector<float> > &data, vector<vector<char> > &valid);

void testScanFusion(){
    cout << "Testing testScanFusion Component " << endl;
    DepthFeaturesCoordinator dp;
    dp.config();

    vector<vector<float> > data;
    vector<vector<char> > valid;
    readScan(data, valid);
    cout << "Scan file has been parsed" << endl;
    cout << "The file contains " <<  data.size() << " each scan is " << data[0].size() << endl;

    for(int i = 0; i < data.size(); i++){
        if(dp.kill){
            break;
        }
        dp.setScan(data[i], valid[i]);
        dp.runOnce();
        usleep(10000);
    }

}

int main(int argc, char **argv){
    try{
        testScanFusion();
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
