//
// Created by David Woller on 18.9.18.
//

#ifndef GLNS_POINT_H
#define GLNS_POINT_H

namespace glns {

class Point {
public:
    Point() = default;;
    Point(float x, float y);
    void print();
    float x;
    float y;
};

}

#endif //GLNS_POINT_H
