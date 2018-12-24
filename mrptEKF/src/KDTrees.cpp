//
//  KDTrees.cpp
//  PlaygroundSTL
//
//  Created by Mohamed Adel on 22/12/18.
//  Copyright © 2018 Mohamed Adel. All rights reserved.
//

#include "KDTrees.hpp"

/// Order 2 points considering their x values
struct OrderByX {
    template<typename T>
    bool operator()(T a, T b) const { return a.x < b.x; }
};

/// Order 2 points considering their y values
struct OrderByY {
    template<typename T>
    bool operator()(T a, T b) const { return a.y < b.y; }
};

/// find the minimum and max x,y
static void minmax_xy(const Landmarks_vector& V,
                      float* minx, float* maxx,
                      float* miny, float* maxy)
{
    float minX = FLT_MAX, minY = FLT_MAX;
    float maxX = -FLT_MAX, maxY = -FLT_MAX;
    for(int i=0; i<V.size(); i++)
    {
        const Landmark& l = V[i];
        if( l.x < minX )
            minX = l.x;
        if( l.x > maxX )
            maxX = l.x;
        if( l.y < minY )
            minY = l.y;
        if( l.y > maxY )
            maxY = l.y;
    }
    /// validate that points lie in the FLT range
    *minx = minX;
    *maxx = maxX;
    *miny = minY;
    *maxy = maxY;
}

