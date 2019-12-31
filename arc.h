//
// Created by David Woller on 18.9.18.
//

#ifndef GLNS_ARC_H
#define GLNS_ARC_H

#include <cairomm/context.h>
#include "point.h"

namespace glns {


    class Arc {
public:
    Arc() = default;;
    Arc(double x, double y, double r, double alpha, double omega);
    void print();
    void draw(const Cairo::RefPtr<Cairo::Context>& cr);
    private:
    Point center;
    double r;
    double alpha;
    double omega;
    int id;
    };



}

#endif //GLNS_ARC_H
