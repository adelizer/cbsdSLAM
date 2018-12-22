//
//  KDTrees.hpp
//  SLAM
//
//  Created by Mohamed Adel on 22/12/18.
//  Copyright © 2018 Mohamed A. Abdelhady. All rights reserved.
//

#ifndef KDTrees_hpp
#define KDTrees_hpp

#include <cassert>
#include <climits>
#include <cstring>
#include <cfloat>
#include <cstdlib>
#include <vector>
#include <iostream>
#include <memory>

#define DEBUG 0

struct Landmark{
    float x,y;
    int id;
    Landmark(float _x, float _y){x = _x; y = _y;};
    ~Landmark(){};
};

struct Region{
    float x1,y1,x2,y2,width,height;
    Region(){}
    Region(float _x1, float _y1, float _x2, float _y2){
        x1 = _x1; y1= _y1; x2 = _x2; y2=_y2;
        width = x2-x1; height = y2-y1;
    }
    bool contains(Region &r) const {
        // checks if r is a subset of this region
        bool result = true;
        if(r.x1 < x1 || r.x2 > x2)
            result = false;
        if(r.y1 < y1 || r.y2 > y2)
            result = false;
        return result;
    }
    bool contains(const Landmark &l) const {
        // check if p lies within the region
        bool result = false;
        if(l.x > x1 && l.x < x2 ){
            if(l.y > y1 && l.y < y2)
                result = true;
        }
        return result;
    }
};

class KdTrees
{
public:
    KdTrees(){}
};

#include <stdio.h>

#endif /* KDTrees_hpp */
