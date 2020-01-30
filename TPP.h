//
// Created by h on 01.12.19.
//

#ifndef GLNSMODIFIED_TPP_H
#define GLNSMODIFIED_TPP_H

#include <utility>

#include "triangulation.hpp"
#include "polygon.h"
using namespace pmap::geom;
using namespace glns;

class TPP {
private:
    std::vector<Polygon>  convexHulls;

public:
    TPP(std::vector<Polygon> polygons, float accuracy) : inputPolygons(std::move(polygons)), accuracy(accuracy) {};
    void run();
    std::vector<FPoint> algorithm3(std::vector<Polygon> &polygons);
    std::vector<FPoint> algorithm3WithInitialPath(std::vector<Polygon> &polygons, std::vector<FPoint> &inputPoints);
    static FPoint findOptimalConnectingPointInPolygon(FPoint &pointA, FPoint &pointB, Polygon &polygon);
    //FPoint findOptimalConnectingPointInPolygonNaive(FPoint &pointA, FPoint &pointB, FPolygon &polygon);
    //FPoint findOptimalConnectingPointInPolygonBruteForce(FPoint &pointA, FPoint &pointB, FPolygon &polygon);
    static double calcCycleLength(std::vector<FPoint> &points, int k);
    int simpleHull_2D( FPolygon P, FPolygon &H );
    std::vector<Polygon>  inputPolygons;
    std::vector<FPoint> outputPoints;
    float accuracy;
};

#endif //GLNSMODIFIED_TPP_H

