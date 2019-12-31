//
// Created by David Woller on 2.10.18.
//

#ifndef GLNS_VERTEX_H
#define GLNS_VERTEX_H

/*
 * This class represents vertex in Rattled grid GTSP problem definition.
 * Each vertex has unique id and 2D euclidean coordinates.
 */

#include <cairomm/context.h>

namespace glns {

    class Vertex {
public:
    Vertex() = default;;
    Vertex(double x, double y, int id): x(x), y(y), id(id) {};
    float getDistanceFrom(Vertex v2);
    void draw(const Cairo::RefPtr<Cairo::Context>& cr, int size);

    int id; // vertices are indexed according to id given in input data
    int setId; // id of set containing this vertex and also index of its position in vector Planner.sets
    double x, y;
    float removalCost;
    float BFSWeight;
    int BFSPrevId;

};

}



#endif //GLNS_VERTEX_H
