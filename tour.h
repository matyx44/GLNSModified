//
// Created by David Woller on 5.10.18.
//

#ifndef GLNS_TOUR_H
#define GLNS_TOUR_H

#include <utility>

#include "utils.hpp"
#include "polygon.h"

namespace glns {
class Tour {
public:
    Tour() = default;;
    Tour(std::vector<int> polygons, std::vector<pmap::geom::FPoint> points) : polygons(std::move(polygons)), points(std::move(points)) {};

    std::vector<int> polygons;
    std::vector<pmap::geom::FPoint> points;
};

}


#endif //GLNS_TOUR_H
