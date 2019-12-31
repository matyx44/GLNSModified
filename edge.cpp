//
// Created by David Woller on 5.10.18.
//

#include "edge.h"

using namespace glns;

void Edge::draw(const Cairo::RefPtr<Cairo::Context> &cr, std::vector<Vertex> &vertices) {
    cr->save();
    cr->set_line_width(0.5);
    cr->move_to(vertices[vFromId].x, -vertices[vFromId].y);
    cr->line_to(vertices[vToId].x, -vertices[vToId].y);
    cr->stroke();
    cr->restore();
}
