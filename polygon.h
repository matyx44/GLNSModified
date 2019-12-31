//
// Created by David Woller on 4.10.18.
//

#ifndef GLNS_SET_H
#define GLNS_SET_H

#include <vector>
#include <random>
#include "vertex.h"

namespace glns {

class Polygon {
public:
    Polygon();
    Polygon(std::vector<Vertex> vertices, int id);

    std::vector<Vertex> vertices;
    int id; // sets are indexed from 0; id should correspond with position in vector "sets"
    std::vector<float> rgb;
    float minDist;

    static std::minstd_rand generator;

};


}

#endif //GLNS_SET_H
