//
// Created by David Woller on 4.10.18.
//

#ifndef GLNS_SET_H
#define GLNS_SET_H

#include <vector>
#include <random>
#include "utils.hpp"

namespace glns {

class Polygon {
public:
    Polygon() = default;
    Polygon(std::vector<pmap::geom::FPoint> points, int id) : points(std::move(points)), id(id) {};

    std::vector<pmap::geom::FPoint> points;
    int id; // id of the polygon
    double removalCost;

};


}

#endif //GLNS_SET_H
