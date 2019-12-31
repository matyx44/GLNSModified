//
// Created by David Woller on 18.9.18.
//

#include <iostream>
#include "point.h"

using namespace glns;

Point::Point(float x, float y) {
    this->x = x;
    this->y = y;
}

void Point::print() {
    std::cout << "x: " << x << std::endl;
    std::cout << "y: " << y << std::endl;
}
