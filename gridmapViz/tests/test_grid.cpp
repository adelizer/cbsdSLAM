#include <iostream>
#include <unistd.h>

#include "gridmapcoordinator.h"

using namespace std;
void readScan(vector<vector<float> > &data, vector<vector<char> > &valid);


void testGrid(){
    cout << "Testing Grid Map Component " << endl;
    GridMapCoordinator c;

    vector<vector<float> > data;
    vector<vector<char> > valid;
    readScan(data, valid);
    cout << "Scan file has been parsed" << endl;
    cout << "The file contains " <<  data.size() << " each scan is " << data[0].size() << endl;

    for(int i = 0; i < data.size(); i++){
        if(c.kill){
            break;
        }
        c.setScan(data[i], valid[i]);
        c.runOnce();
        usleep(10000);
    }
}

int main(int argc, char **argv){
    try{
        testGrid();
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
