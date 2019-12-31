//
// Created by David Woller on 3.10.18.
//

#include <cmath>
#include <iostream>
#include "vertex.h"

using namespace glns;

float Vertex::getDistanceFrom(Vertex v2) {
    float distance = sqrt(pow(x - v2.x, 2) + pow(y - v2.y, 2));
    return distance;
}

void Vertex::draw(const Cairo::RefPtr<Cairo::Context> &cr, int size) {
    cr->save();
    cr->arc(x, -y, size, 0, 2*M_PI);
    cr->fill_preserve();
    cr->stroke();
    cr->restore();
}
