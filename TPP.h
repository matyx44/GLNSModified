//
// Created by h on 01.12.19.
//

#ifndef GLNSMODIFIED_TPP_H
#define GLNSMODIFIED_TPP_H

#include "triangulation.hpp"
using namespace pmap::geom;

class TPP {
private:
    std::vector<FPolygon>  convexHulls;

public:
    TPP(std::vector<FPolygon> &polygons, float accuracy) : inputPolygons(polygons), accuracy(accuracy) {};
    void run();
    std::vector<FPoint> algorithm3(std::vector<FPolygon> &polygons);
    std::vector<FPoint> algorithm3WithInitialPath(std::vector<FPolygon> &polygons, std::vector<FPoint> &inputPoints);
    FPoint findOptimalConnectingPointInPolygon(FPoint &pointA, FPoint &pointB, FPolygon &polygon);
    //FPoint findOptimalConnectingPointInPolygonNaive(FPoint &pointA, FPoint &pointB, FPolygon &polygon);
    FPoint findOptimalConnectingPointInPolygonBruteForce(FPoint &pointA, FPoint &pointB, FPolygon &polygon);
    double calcCycleLength(std::vector<FPoint> &points, int k);
    int simpleHull_2D( FPolygon P, FPolygon &H );
    std::vector<FPolygon>  &inputPolygons;
    std::vector<FPoint> outputPoints;
    float accuracy;
};

#endif //GLNSMODIFIED_TPP_H

