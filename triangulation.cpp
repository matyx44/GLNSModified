/**
 * File:    triangulation.cpp
 *
 * Date:    10.10.19
 * Author:  Jan Mikula
 * E-mail:  mikula.miki@gmail.com
 *
 */

#include "triangulation.hpp"
#include "simple_intersection.h"
#include "utils.hpp"

#include "clipper.hpp"

#include <limits>

#define VOID int
#define REAL double

extern "C" {
#include "triangle.h"
}

/**
 *
 * from: http://apodeline.free.fr/FAQ/CGAFAQ/CGAFAQ-3.html
 *
 * How do I find a single point inside a simple polygon?
 *
 * Given a simple polygon, find some point inside it.
 * Here is a method based on the proof that there exists an internal diagonal.
 * The idea is that the midpoint of a diagonal is interior to the polygon.
 *
 * 1) Identify a convex vertex v; let its adjacent vertices be u and w.
 * 2) For each other vertex q do
 * 2a) if q is inside uvw, compute distance to v (orthogonal to uw).
 * 2b) Save point q if distance is a new min.
 * 3) If no point is inside, return centroid of uvw.
 * 4) Else if some point inside, qv is internal: return its midpoint.
 *
 */
pmap::geom::FPoint findPointInsidePolygon(const pmap::geom::FPolygon &polygon) {
    pmap::geom::FPoint ret{};
    /// 1) Identify a convex vertex v
    unsigned int idxYMinVertex = 0;
    for (unsigned int i = 0; i < polygon.size(); ++i)
        if (polygon[i].y < polygon[idxYMinVertex].y ||
            (polygon[i].y == polygon[idxYMinVertex].y && polygon[i].x < polygon[idxYMinVertex].x))
            idxYMinVertex = i;
    unsigned int idxU, idxV, idxW;
    idxU = (idxYMinVertex == 0) ? (unsigned int) polygon.size() - 1 : idxYMinVertex - 1;
    idxV = idxYMinVertex;
    idxW = (idxYMinVertex == (unsigned int) polygon.size() - 1) ? 0 : idxYMinVertex + 1;
    if (polygon[idxU].x > polygon[idxW].x) {
        unsigned int idxTemp;
        idxTemp = idxU;
        idxU = idxW;
        idxW = idxTemp;
    }
    /// let its adjacent vertices be u and w.
    pmap::geom::FPoint u = {polygon[idxU].x, polygon[idxU].y};
    pmap::geom::FPoint v = {polygon[idxV].x, polygon[idxV].y};
    pmap::geom::FPoint w = {polygon[idxW].x, polygon[idxW].y};
    /// 2) For each other vertex q do
    imr::geom::CIntersection<pmap::geom::FPoint> cI;
    int q = -1;
    double minDistanceToV = std::numeric_limits<double>::max();
    for (unsigned int i = 0; i < polygon.size(); ++i) {
        pmap::geom::FPoint a = {polygon[i].x, polygon[i].y};
        /// 2a) if q is inside uvw,
        if (i != idxU && i != idxV && i != idxW
            && !cI.collinear(u, v, a) && !cI.collinear(v, w, a) && cI.in_cone(u, v, w, a)) {
            /// compute distance to v (orthogonal to uw).
            double distanceToV = a.distanceTo(v);
            /// 2b) Save point q if distance is a new min.
            if (distanceToV < minDistanceToV) {
                minDistanceToV = distanceToV;
                q = i;
            }
        }
    }
    /// 3) If no point is inside, return centroid of uvw.
    if (q == -1) {
        ret = {(u.x + v.x + w.x) / 3, (u.y + v.y + w.y) / 3};
    } else { /// 4) Else if some point inside, qv is internal: return its midpoint.
        ret = {(polygon[q].x + v.x) / 2, (polygon[q].y + v.y) / 2};
    }
    return ret;
}

void pmap::triangulation::generateTriangularMesh(const pmap::geom::FMap &areaIn,
                                                 double visibilityRangeIn,
                                                 pmap::geom::FPoints &locationsOut,
                                                 pmap::geom::FPolygons &polygonsOut) {

    struct triangulateio in{}, out{};

    /// Compute total number of points:
    int number_of_points = 0;
    for (auto &point : areaIn) {
        number_of_points += point.size();
    }

    std::cout<< "Number of points: "<< number_of_points;

    /// in. allocations:
    in.numberofpoints = number_of_points;
    in.pointlist = (REAL *) malloc(in.numberofpoints * 2 * sizeof(REAL));
    in.numberofpointattributes = 0;
    in.pointmarkerlist = (int *) nullptr;

    in.numberofsegments = in.numberofpoints;
    in.segmentlist = (int *) malloc(in.numberofsegments * 2 * sizeof(int));
    in.segmentmarkerlist = (int *) nullptr;

    in.numberofholes = (int) areaIn.size() - 1;
    in.holelist = (REAL *) malloc(in.numberofholes * 2 * sizeof(REAL));

    in.numberofregions = 0;
    in.regionlist = (REAL *) nullptr;

    /// Insert points, segments and holes:
    int idxPoints = 0, idxSegments = 0, idxHoles = 0;
    int numberOfPointsInserted = 0;
    for (auto &polygon : areaIn) {
        int firstPointOfPolygon = numberOfPointsInserted;
        for (unsigned int j = 0; j < polygon.size(); ++j, ++numberOfPointsInserted) {

            /// Insert points:
            in.pointlist[idxPoints++] = polygon[j].x;
            in.pointlist[idxPoints++] = polygon[j].y;

            /// Insert segments:
            if (j != polygon.size() - 1) { // if not the last segment
                in.segmentlist[idxSegments++] = numberOfPointsInserted;
                in.segmentlist[idxSegments++] = numberOfPointsInserted + 1;
            } else {
                in.segmentlist[idxSegments++] = numberOfPointsInserted;
                in.segmentlist[idxSegments++] = firstPointOfPolygon;
            }
        }

        /// Insert holes:
        if (!ClipperLib::Orientation(pmap::toClipper(polygon))) {
            pmap::geom::FPoint hole_point = findPointInsidePolygon(polygon);
            in.holelist[idxHoles++] = hole_point.x;
            in.holelist[idxHoles++] = hole_point.y;
        }
    }

    /// out. allocations
    out.pointlist = (REAL *) nullptr;
    out.pointmarkerlist = (int *) nullptr;
    out.trianglelist = (int *) nullptr;
    out.segmentlist = (int *) nullptr;
    out.segmentmarkerlist = (int *) nullptr;

    char str[20];
    sprintf(str, "pQza%fs", visibilityRangeIn);

    triangulate(str, &in, &out, (struct triangulateio *) nullptr);

    for (int i = 0, j = 0; i < out.numberoftriangles * 3; i += 3, j++) {
        pmap::geom::FPoint p1, p2, p3;
        p1 = {out.pointlist[out.trianglelist[i] * 2], out.pointlist[out.trianglelist[i] * 2 + 1]};
        p2 = {out.pointlist[out.trianglelist[i + 1] * 2], out.pointlist[out.trianglelist[i + 1] * 2 + 1]};
        p3 = {out.pointlist[out.trianglelist[i + 2] * 2], out.pointlist[out.trianglelist[i + 2] * 2 + 1]};
        locationsOut.push_back({(p1.x + p2.x + p3.x) / 3, (p1.y + p2.y + p3.y) / 3});
        pmap::geom::FPolygon triangle(3);
        triangle[0] = p1, triangle[1] = p2, triangle[2] = p3;
        polygonsOut.push_back(triangle);
    }

    /// Cleaning up:
    free(in.pointlist);
    free(in.pointmarkerlist);
    free(in.segmentlist);
    free(in.segmentmarkerlist);
    free(in.holelist);
    free(in.regionlist);
    free(out.pointlist);
    free(out.pointmarkerlist);
    free(out.trianglelist);
    free(out.segmentlist);
    free(out.segmentmarkerlist);

}



void pmap::triangulation::generateTriangularMeshFromPolygon(const pmap::geom::FPolygon &areaIn,
                                                 double visibilityRangeIn,
                                                 pmap::geom::FPoints &locationsOut,
                                                 pmap::geom::FPolygons &polygonsOut) {

    struct triangulateio in{}, out{};

    /// Compute total number of points:
    int number_of_points = areaIn.size();

    /// in. allocations:
    in.numberofpoints = number_of_points;
    in.pointlist = (REAL *) malloc(in.numberofpoints * 2 * sizeof(REAL));
    in.numberofpointattributes = 0;
    in.pointmarkerlist = (int *) nullptr;

    in.numberofsegments = in.numberofpoints;
    in.segmentlist = (int *) malloc(in.numberofsegments * 2 * sizeof(int));
    in.segmentmarkerlist = (int *) nullptr;

    in.numberofholes = (int) areaIn.size() - 1;
    in.holelist = (REAL *) malloc(in.numberofholes * 2 * sizeof(REAL));

    in.numberofregions = 0;
    in.regionlist = (REAL *) nullptr;

    /// Insert points, segments and holes:
    int idxPoints = 0, idxSegments = 0, idxHoles = 0;
    int numberOfPointsInserted = 0;

    int firstPointOfPolygon = numberOfPointsInserted;
    for (unsigned int j = 0; j < areaIn.size(); ++j, ++numberOfPointsInserted) {

        /// Insert points:
        in.pointlist[idxPoints++] = areaIn[j].x;
        in.pointlist[idxPoints++] = areaIn[j].y;

        /// Insert segments:
        if (j != areaIn.size() - 1) { // if not the last segment
            in.segmentlist[idxSegments++] = numberOfPointsInserted;
            in.segmentlist[idxSegments++] = numberOfPointsInserted + 1;
        } else {
            in.segmentlist[idxSegments++] = numberOfPointsInserted;
            in.segmentlist[idxSegments++] = firstPointOfPolygon;
        }
    }

    /// Insert holes:
    if (!ClipperLib::Orientation(pmap::toClipper(areaIn))) {
        pmap::geom::FPoint hole_point = findPointInsidePolygon(areaIn);
        in.holelist[idxHoles++] = hole_point.x;
        in.holelist[idxHoles++] = hole_point.y;
    }


    /// out. allocations
    out.pointlist = (REAL *) nullptr;
    out.pointmarkerlist = (int *) nullptr;
    out.trianglelist = (int *) nullptr;
    out.segmentlist = (int *) nullptr;
    out.segmentmarkerlist = (int *) nullptr;

    char str[20];
    //sprintf(str, "pQza%fs", visibilityRangeIn);
    sprintf(str, "pQza%f", visibilityRangeIn);

    triangulate(str, &in, &out, (struct triangulateio *) nullptr);

    for (int i = 0, j = 0; i < out.numberoftriangles * 3; i += 3, j++) {
        pmap::geom::FPoint p1, p2, p3;
        p1 = {out.pointlist[out.trianglelist[i] * 2], out.pointlist[out.trianglelist[i] * 2 + 1]};
        p2 = {out.pointlist[out.trianglelist[i + 1] * 2], out.pointlist[out.trianglelist[i + 1] * 2 + 1]};
        p3 = {out.pointlist[out.trianglelist[i + 2] * 2], out.pointlist[out.trianglelist[i + 2] * 2 + 1]};
        locationsOut.push_back({(p1.x + p2.x + p3.x) / 3, (p1.y + p2.y + p3.y) / 3});
        pmap::geom::FPolygon triangle(3);
        triangle[0] = p1, triangle[1] = p2, triangle[2] = p3;
        polygonsOut.push_back(triangle);
    }

    /// Cleaning up:
    free(in.pointlist);
    free(in.pointmarkerlist);
    free(in.segmentlist);
    free(in.segmentmarkerlist);
    free(in.holelist);
    free(in.regionlist);
    free(out.pointlist);
    free(out.pointmarkerlist);
    free(out.trianglelist);
    free(out.segmentlist);
    free(out.segmentmarkerlist);

}
