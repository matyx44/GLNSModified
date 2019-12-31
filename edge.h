//
// Created by David Woller on 2.10.18.
//

#ifndef GLNS_EDGE_H
#define GLNS_EDGE_H


#include "vertex.h"

/*
 * This class represents edge in Rattled grid GTSP problem definition.
 */
namespace glns {

class Edge {
public:
    Edge() = default;;
    Edge(int vFromId, int vToId, float weigth) : vFromId(vFromId), vToId(vToId), weight(weigth) {};
    void draw(const Cairo::RefPtr<Cairo::Context>& cr, std::vector<Vertex> &vertices);

    int vFromId;
    int vToId;
    float weight;
};

}

#endif //GLNS_EDGE_H
