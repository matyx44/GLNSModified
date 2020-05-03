//
// Created by Jan Vidasic on 01.12.19.
//

#include <limits>
#include <iostream>
#include "TPP.h"
#include "triangulation.hpp"
#include "simple_intersection.h"
using namespace pmap::geom;
using namespace imr::geom;


void printPoints(std::vector<FPoint> &points){
    for (auto &point : points) {
        std::cout << "Point: " << point.x << " " << point.y << std::endl;
    }
}


void TPP::run() {
    //Step 1 - Make polygons convex using Melkman's algorithm.
    for(auto &polygon : inputPolygons){
        std::vector<FPoint> ps;
        simpleHull_2D(polygon.points, ps);
        Polygon convexPolygon(ps, 0);
        convexHulls.emplace_back(convexPolygon);
    }
    //Step 2 - Find the length L1
    std::vector<FPoint> points = algorithm3(convexHulls);

    for (int i = 1; i < inputPolygons.size(); ++i) {
        points[i] = findOptimalConnectingPointInPolygon(points[i-1], points[i+1], inputPolygons[i]);
    }

    points = algorithm3WithInitialPath(inputPolygons, points);

    outputPoints = points;
    //std::cout << "Final final length: " << calcCycleLength(points, inputPolygons.size()) << std::endl;


}

double pointToPointDistance(FPoint &a, FPoint &b){
    return std::sqrt(CIntersection<FPoint>::squared_distance(a, b));
}

double TPP::calcCycleLength(std::vector<FPoint> &points, int k){
    double L1 = 0;
    for (int i = 0; i < k; ++i) {
        L1 += pointToPointDistance(points[i], points[i+1]);
    }
    return L1;
}

//Points a,b and line (c,d)
static bool pointsOnTheOppositeSidesOfLine(FPoint & a, FPoint & b, FPoint & c, FPoint & d){
    return ((c.y - d.y) * (a.x - c.x) + (d.x - c.x) * (a.y - c.y)) *
           ((c.y - d.y) * (b.x - c.x) + (d.x - c.x) * (b.y - c.y)) < 0;
}

//Reflect point a over a line(b,c)
static void reflectPointOverLine(FPoint & a, FPoint & b, FPoint & c, FPoint &reflectedPoint){
    FPoint projection;
    CIntersection<FPoint>::get_point_line_projections(a, b, c, projection);
    double distX = projection.x - a.x;
    double distY = projection.y - a.y;
    reflectedPoint.x = projection.x + distX;
    reflectedPoint.y = projection.y + distY;
}

static bool lineSegmentLineIntersect(FPoint & p1, FPoint & p2, FPoint & p3, FPoint & p4, FPoint & pa, double &param)
{
    static const double EPS = 0.00001;
    FPoint p13, p43, p21;
    double d1343, d4321, d1321, d4343, d2121;
    double numer, denom;
    double mua, mub;

    p13.x = p1.x - p3.x;
    p13.y = p1.y - p3.y;
    p43.x = p4.x - p3.x;
    p43.y = p4.y - p3.y;
    if (fabs(p43.x) < EPS && fabs(p43.y) < EPS)
        return false;
    p21.x = p2.x - p1.x;
    p21.y = p2.y - p1.y;
    if (fabs(p21.x) < EPS && fabs(p21.y) < EPS)
        return false;

    d1343 = p13.x * p43.x + p13.y * p43.y;
    d4321 = p43.x * p21.x + p43.y * p21.y;
    d1321 = p13.x * p21.x + p13.y * p21.y;
    d4343 = p43.x * p43.x + p43.y * p43.y;
    d2121 = p21.x * p21.x + p21.y * p21.y;

    denom = d2121 * d4343 - d4321 * d4321;

    if (fabs(denom) < EPS)
    {
        return false;
    }
    numer = d1343 * d4321 - d1321 * d4343;

    mua = numer / denom;
    mub = (d1343 + d4321 * mua) / d4343;

    pa.x = p1.x + mua * p21.x;
    pa.y = p1.y + mua * p21.y;
    //pb.x = p3.x + mub * p43.x;
    //pb.y = p3.y + mub * p43.y;

    param = mub;

    return true;
}

FPoint TPP::findOptimalConnectingPointInPolygon(FPoint &pointA, FPoint &pointB, Polygon &polygon){
    FPoint optimalPoint;
    if(CIntersection<FPoint>::cn_PnPoly(pointA, polygon.points) && CIntersection<FPoint>::cn_PnPoly(pointB, polygon.points)){
        optimalPoint = FPoint((pointA.x + pointB.x)/2, (pointA.y + pointB.y)/2);
        if(CIntersection<FPoint>::cn_PnPoly(optimalPoint, polygon.points))
            return optimalPoint;
        //else return FPoint(pointA.x,pointA.y);
    }


    double minDist = std::numeric_limits<double>::max();
    int n = polygon.points.size();
    for (int i = 0; i < n; i++) {
        int j = (i < (n - 1)) ? i + 1 : 0;
        FPoint newB(pointB.x, pointB.y);
        if(!pointsOnTheOppositeSidesOfLine(pointA, pointB, polygon.points[i], polygon.points[j])) {
            reflectPointOverLine(pointB, polygon.points[i], polygon.points[j], newB);
        }
        FPoint tmpOptimalPoint;
        double k;
        lineSegmentLineIntersect(pointA, newB, polygon.points[i], polygon.points[j], tmpOptimalPoint, k);
        if(k < 0){
            tmpOptimalPoint = polygon.points[i];
        }
        else if(k > 1){
            tmpOptimalPoint = polygon.points[j];
        }
        double tmpDist = pointToPointDistance(pointA, tmpOptimalPoint) + pointToPointDistance(newB, tmpOptimalPoint);
        if(tmpDist < minDist){
            minDist = tmpDist;
            optimalPoint = tmpOptimalPoint;
        }
    }
    return optimalPoint;
}

/*
FPoint TPP::findOptimalConnectingPointInPolygonNaive(FPoint &pointA, FPoint &pointB, FPolygon &polygon){
    FPoint optimalPoint;
    //If the line segment between the two points intersects the polygon, the work is done.
    if(CIntersection<FPoint>::linePolyIntersect(pointA, pointB, polygon, optimalPoint)){
        return optimalPoint;
    }
    else{
        int n = polygon.size();
        int closestVerticeIndexInPolygon = 0;
        double minDist = std::numeric_limits<double>::max();
        for (int i = 0; i < n; i++) {
            double tmpDist = pointToPointDist(pointA, polygon[i]) + pointToPointDist(pointB, polygon[i]);
            if(minDist > tmpDist){
                minDist = tmpDist;
                optimalPoint = polygon[i];
                closestVerticeIndexInPolygon = i;
            }
        }

        int nextIndex = (closestVerticeIndexInPolygon == (n - 1)) ? 0 : closestVerticeIndexInPolygon + 1;
        int prevIndex = (closestVerticeIndexInPolygon == 0) ? n - 1 : closestVerticeIndexInPolygon - 1;

        for (double t = 0.01; t < 1; t+=0.01) {
            double x = (1 - t)*polygon[closestVerticeIndexInPolygon].x + t*polygon[prevIndex].x;
            double y = (1 - t)*polygon[closestVerticeIndexInPolygon].y + t*polygon[prevIndex].y;
            FPoint tmpPoint(x,y);
            double tmpDist = pointToPointDist(pointA, tmpPoint) + pointToPointDist(pointB, tmpPoint);
            if(minDist > tmpDist){
                minDist = tmpDist;
                optimalPoint = tmpPoint;
            }
            else break;
        }

        for (double t = 0.01; t < 1; t+=0.01) {
            double x = (1 - t)*polygon[closestVerticeIndexInPolygon].x + t*polygon[nextIndex].x;
            double y = (1 - t)*polygon[closestVerticeIndexInPolygon].y + t*polygon[nextIndex].y;
            FPoint tmpPoint(x,y);
            double tmpDist = pointToPointDist(pointA, tmpPoint) + pointToPointDist(pointB, tmpPoint);
            if(minDist > tmpDist){
                minDist = tmpDist;
                optimalPoint = tmpPoint;
            }
            else break;
        }

        return optimalPoint;
    }
}*/

/*
FPoint TPP::findOptimalConnectingPointInPolygonBruteForce(FPoint &pointA, FPoint &pointB, FPolygon &polygon){
    FPoint optimalPoint;
    //If the line segment between the two points intersects the polygon, the work is done.
    //if(CIntersection<FPoint>::linePolyIntersect(pointA, pointB, polygon, optimalPoint)){
    //    return optimalPoint;
    //}
    //else{
        int n = polygon.size();
        double minDist = std::numeric_limits<double>::max();
        for (int i = 0; i < n; i++) {
            int j = (i < (n - 1)) ? i + 1 : 0;
            for (double t = 0; t < 1; t+=0.01) {
                double x = (1 - t)*polygon[i].x + t*polygon[j].x;
                double y = (1 - t)*polygon[i].y + t*polygon[j].y;
                FPoint tmpPoint(x,y);
                double tmpDist = pointToPointDist(pointA, tmpPoint) + pointToPointDist(pointB, tmpPoint);
                if(minDist > tmpDist){
                    minDist = tmpDist;
                    optimalPoint = tmpPoint;
                }
            }
        }
        return optimalPoint;
   // }
}*/

std::vector<FPoint> TPP::algorithm3WithInitialPath(std::vector<glns::Polygon> &polygons, std::vector<FPoint> &inputPoints){
    int k = polygons.size();

    ///Step 1 - Get initial points
    std::vector<FPoint> points = inputPoints;

    ///Step 2 - Get initial lengths L0 and L1
    double L0 = std::numeric_limits<double>::max();
    double L1 = calcCycleLength(points, k);
    //std::cout << "Initial length L1 - " << L1 << std::endl;
    while(L0 - L1 >= accuracy){
        FPoint point0 = findOptimalConnectingPointInPolygon(points[k-1], points[1], polygons[0]);
        points[0] = point0;
        points[k] = point0;
        for (int i = 1; i < k; ++i) {
            points[i] = findOptimalConnectingPointInPolygon(points[i-1], points[i+1], polygons[i]);
        }

        L0 = L1;
        L1 = calcCycleLength(points, k);
    }
    //std::cout << "Points: " << std::endl;
    //printPoints(points);
    //std::cout << "Final length L1 - " << L1 << std::endl;
    return points;
}

std::vector<FPoint> TPP::algorithm3(std::vector<glns::Polygon> &polygons) {
    int k = polygons.size();

    ///Step 1 - Get initial points of the path (random vertex for each polygon)
    std::vector<FPoint> points;
    points.reserve(polygons.size()+1);
    for(auto &polygon : polygons){
        points.emplace_back(polygon.points[0]);
    }
    //Make cycle
    points.emplace_back(polygons[0].points[0]);


    ///Step 2 - Get initial lengths L0 and L1
    double L0 = std::numeric_limits<double>::max();
    double L1 = calcCycleLength(points, k);
    //std::cout << "Initial length L1 - " << L1 << std::endl;
    while(L0 - L1 >= accuracy){
        FPoint point0 = findOptimalConnectingPointInPolygon(points[k-1], points[1], polygons[0]);
        points[0] = point0;
        points[k] = point0;
        for (int i = 1; i < k; ++i) {
            points[i] = findOptimalConnectingPointInPolygon(points[i-1], points[i+1], polygons[i]);
        }

        L0 = L1;
        L1 = calcCycleLength(points, k);
    }
    //std::cout << "Points: " << std::endl;
    //printPoints(points);
    //std::cout << "Final length L1 - " << L1 << std::endl;
    return points;
}



/// http://geomalgorithms.com/a12-_hull-3.html
// isLeft(): test if a point is Left|On|Right of an infinite line.
//    Input:  three points P0, P1, and P2
//    Return: >0 for P2 left of the line through P0 and P1
//            =0 for P2 on the line
//            <0 for P2 right of the line
//    See: Algorithm 1 on Area of Triangles
double isLeft( FPoint P0, FPoint P1, FPoint P2 )
{
    return (P1.x - P0.x)*(P2.y - P0.y) - (P2.x - P0.x)*(P1.y - P0.y);
}

// simpleHull_2D(): Melkman's 2D simple polyline O(n) convex hull algorithm
//    Input:  P[] = array of 2D vertex points for a simple polyline
//            n   = the number of points in V[]
//    Output: H[] = output convex hull array of vertices (max is n)
//    Return: h   = the number of points in H[]
int TPP::simpleHull_2D( FPolygon P, FPolygon &H )
{
    // initialize a deque D[] from bottom to top so that the
    // 1st three vertices of P[] are a ccw triangle
    int n = P.size();
    FPoint* D = new FPoint[2*n+1];
    int bot = n-2, top = bot+3;    // initial bottom and top deque indices
    D[bot] = D[top] = P[2];        // 3rd vertex is at both bot and top
    if (isLeft(P[0], P[1], P[2]) > 0) {
        D[bot+1] = P[0];
        D[bot+2] = P[1];           // ccw vertices are: 2,0,1,2
    }
    else {
        D[bot+1] = P[1];
        D[bot+2] = P[0];           // ccw vertices are: 2,1,0,2
    }

    // compute the hull on the deque D[]
    for (int i=3; i < n; i++) {   // process the rest of vertices
        // test if next vertex is inside the deque hull
        if ((isLeft(D[bot], D[bot+1], P[i]) > 0) &&
            (isLeft(D[top-1], D[top], P[i]) > 0) )
            continue;         // skip an interior vertex

        // incrementally add an exterior vertex to the deque hull
        // get the rightmost tangent at the deque bot
        while (isLeft(D[bot], D[bot+1], P[i]) <= 0)
            ++bot;                 // remove bot of deque
        D[--bot] = P[i];           // insert P[i] at bot of deque

        // get the leftmost tangent at the deque top
        while (isLeft(D[top-1], D[top], P[i]) <= 0)
            --top;                 // pop top of deque
        D[++top] = P[i];           // push P[i] onto top of deque
    }

    // transcribe deque D[] to the output hull array H[]
    int h;        // hull vertex counter
    for (h=0; h <= (top-bot); h++)
        H.emplace_back((D[bot + h]));

    delete D;
    return h-1;
}

















/*
//Shewchuk - https://www.cs.cmu.edu/~quake/robust.html
double orient2dfast(FPoint pa, FPoint pb, FPoint pc){
    double acx = pa.x - pc.x;
    double bcx = pb.x - pc.x;
    double acy = pa.y - pc.y;
    double bcy = pb.y - pc.y;
    return acx * bcy - acy * bcx;
}

void melkmann(FPolygon pol){
    int t = -1;
    int b = 0;
    deque<FPoint> d;
    if(orient2dfast(pol[0], pol[1], pol[2]) > 0){
        t++;
        d.push_front(pol[2]);
        d.push_front(pol[0]);
        d.push_front(pol[1]);
        d.push_front(pol[2]);
    }
    else{
        d.push_front(pol[2]);
        d.push_front(pol[1]);
        d.push_front(pol[0]);
        d.push_front(pol[2]);
    }
}*/