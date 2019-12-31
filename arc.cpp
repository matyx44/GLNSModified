//
// Created by David Woller on 18.9.18.
//

#include <iostream>
#include "arc.h"

using namespace glns;

static int numSegments = 0;

Arc::Arc(double x, double y, double r, double alpha, double omega) :
center(x, y), r(r), alpha(alpha), omega(omega), id(numSegments++) {}

void Arc::print() {
    center.print();
    std::cout << "radius: " << r << std::endl;
    std::cout << "alpha: " << alpha << std::endl;
    std::cout << "omega: " << omega << std::endl;
    std::cout << "id: " << id << std::endl;
}

void Arc::draw(const Cairo::RefPtr<Cairo::Context> &cr) {
    cr->save();
    cr->set_source_rgb(0, 0, 0);
    cr->arc(center.x, -center.y, r, 2*M_PI - alpha - omega/2, 2*M_PI - alpha + omega/2);
    cr->stroke();
    cr->restore();

};


