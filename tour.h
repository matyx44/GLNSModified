//
// Created by David Woller on 5.10.18.
//

#ifndef GLNS_TOUR_H
#define GLNS_TOUR_H

#include "vertex.h"
#include "edge.h"
#include "set.h"

namespace glns {
class Tour {
public:
    Tour() = default;;

    std::vector<Vertex> vertices;
    std::vector<Edge> edges;
};

}


#endif //GLNS_TOUR_H
