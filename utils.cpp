/**
 * File:    utils.cpp
 *
 * Date:    4.12.18
 * Author:  Jan Mikula
 * E-mail:  mikula.miki@gmail.com
 *
 */

#include "utils.hpp"

#include <fstream>
#include <iostream>
#include <limits>

using namespace pmap;
using namespace pmap::geom;

FPolygon pmap::getRegularPolygon(FPoint center, double radius, unsigned int N) {
    FPolygon ret = {};
    double angle = 2 * M_PI / N;
    for (unsigned int i = 0; i < N; ++i) {
        double alpha = i * angle;
        FPoint point = {center.x + radius * sin(alpha), center.y + radius * cos(alpha)};
        ret.push_back(point);
    }
    return ret;
}

ClipperLib::Path pmap::getRegularClipPolygon(FPoint center, double radius, unsigned int N) {
    ClipperLib::Path ret = {};
    double angle = 2 * M_PI / N;
    for (unsigned int i = 0; i < N; ++i) {
        double alpha = i * angle;
        ClipperLib::IntPoint clipPoint = {(ClipperLib::cInt) ((center.x + radius * sin(alpha)) * F_TO_CLIP),
                                          (ClipperLib::cInt) ((center.y + radius * cos(alpha)) * F_TO_CLIP)};
        ret.push_back(clipPoint);
    }
    return ret;
}

ClipperLib::IntPoint pmap::toClipper(const FPoint &fPoint) {
    return {(ClipperLib::cInt) (fPoint.x * F_TO_CLIP), (ClipperLib::cInt) (fPoint.y * F_TO_CLIP)};
}

ClipperLib::Path pmap::toClipper(const FPolygon &fPolygon) {
    ClipperLib::Path r = {};
    for (auto &fPoint : fPolygon)
        r.push_back(pmap::toClipper(fPoint));
    return r;
}


ClipperLib::Paths pmap::toClipper(const FMap &fMap) {
    ClipperLib::Paths r = {};
    for (auto &fPolygon : fMap)
        r.push_back(pmap::toClipper(fPolygon));
    return r;
}

FPoint pmap::fromClipper(const ClipperLib::IntPoint &cp) {
    return {((geom_float) cp.X) / F_TO_CLIP, ((geom_float) cp.Y) / F_TO_CLIP};
}

FPolygon pmap::fromClipper(const ClipperLib::Path &path) {
    FPolygon r = {};
    for (auto &point : path)
        r.push_back({((geom_float) point.X) / F_TO_CLIP, ((geom_float) point.Y) / F_TO_CLIP});
    return r;
}

FMap pmap::fromClipper(const ClipperLib::Paths &paths) {
    FMap r = {};
    for (auto path : paths)
        r.push_back(pmap::fromClipper(path));
    return r;
}

void pmap::clip(ClipperLib::ClipType ct, ClipperLib::Paths &subj,
                ClipperLib::Paths &clip, ClipperLib::Paths &solution) {
    ClipperLib::Clipper c;
    c.AddPaths(subj, ClipperLib::ptSubject, true);
    c.AddPaths(clip, ClipperLib::ptClip, true);
    c.Execute(ct, solution, ClipperLib::pftNonZero);
    c.Clear();
}

void pmap::clip(ClipperLib::ClipType ct, ClipperLib::Paths &subj,
                ClipperLib::Path &clip, ClipperLib::Paths &solution) {
    ClipperLib::Paths paths(1);
    paths[0] = clip;
    pmap::clip(ct, subj, paths, solution);
}

void pmap::clip(ClipperLib::ClipType ct, ClipperLib::Path &subj,
                ClipperLib::Paths &clip, ClipperLib::Paths &solution) {
    ClipperLib::Paths paths(1);
    paths[0] = subj;
    pmap::clip(ct, paths, clip, solution);
}

void pmap::clip(ClipperLib::ClipType ct, ClipperLib::Path &subj,
                ClipperLib::Path &clip, ClipperLib::Paths &solution) {
    ClipperLib::Paths paths1(1), paths2(1);
    paths1[0] = subj;
    paths2[0] = clip;
    pmap::clip(ct, paths1, paths2, solution);
}

void changeOrientation(FPolygon &poly) {
    FPolygon ret;
    for (auto i = (int) poly.size() - 1; i >= 0; --i) {
        ret.push_back(poly[i]);
    }
    poly = ret;
}

void pmap::getMapLimits(pmap::geom::FMap &map, double &xMin, double &xMax, double &yMin, double &yMax) {
    xMin = std::numeric_limits<double>::max();
    xMax = std::numeric_limits<double>::min();
    yMin = std::numeric_limits<double>::max();
    yMax = std::numeric_limits<double>::min();
    for (auto &polygon : map) {
        for (auto &point : polygon) {
            if (point.y > yMax) yMax = point.y;
            if (point.x > xMax) xMax = point.x;
            if (point.y < yMin) yMin = point.y;
            if (point.x < xMin) xMin = point.x;
        }
    }
}

void pmap::getObstacles(pmap::geom::FMap &map, pmap::geom::FPolygons &obstacles) {
    for (auto &polygon : map) {
        ClipperLib::Path path = toClipper(polygon);
        if (!ClipperLib::Orientation(path)) {
            obstacles.push_back(fromClipper(path));
        } else {
            auto yMax = std::numeric_limits<ClipperLib::cInt>::min(), xMax = std::numeric_limits<ClipperLib::cInt>::min();
            auto yMin = std::numeric_limits<ClipperLib::cInt>::max(), xMin = std::numeric_limits<ClipperLib::cInt>::max();
            for (auto &point : path) {
                if (point.Y > yMax) yMax = point.Y;
                if (point.X > xMax) xMax = point.X;
                if (point.Y < yMin) yMin = point.Y;
                if (point.X < xMin) xMin = point.X;
            }
            auto split = (xMax - xMin) / 2;
            ClipperLib::Paths subj1(1), subj2(1), sol1 = {}, sol2 = {};

            subj1[0] << ClipperLib::IntPoint(0, 0)
                     << ClipperLib::IntPoint(xMin + split, 0)
                     << ClipperLib::IntPoint(xMin + split, yMin + yMax)
                     << ClipperLib::IntPoint(0, yMin + yMax);
            subj2[0] << ClipperLib::IntPoint(xMin + split, 0)
                     << ClipperLib::IntPoint(xMin + xMax, 0)
                     << ClipperLib::IntPoint(xMin + xMax, yMin + yMax)
                     << ClipperLib::IntPoint(xMin + split, yMin + yMax);
            clip(ClipperLib::ctDifference, subj1, path, sol1);
            clip(ClipperLib::ctDifference, subj2, path, sol2);
            FPolygons polys1 = fromClipper(sol1);
            FPolygons polys2 = fromClipper(sol2);
            for (auto &clipPoly : sol1) {
                FPolygon poly = pmap::fromClipper(clipPoly);
                if (ClipperLib::Orientation(clipPoly) == 1) {
                    changeOrientation(poly);
                }
                obstacles.push_back(poly);
            }
            for (auto &clipPoly : sol2) {
                FPolygon poly = pmap::fromClipper(clipPoly);
                if (ClipperLib::Orientation(clipPoly) == 1) {
                    changeOrientation(poly);
                }
                obstacles.push_back(poly);
            }
        }
    }
}

void pmap::offsetMap(pmap::geom::FMap &map, pmap::geom::FMap &submap, double offset) {
    if (offset > 0.00) {
        ClipperLib::Paths paths = pmap::toClipper(map);
        ClipperLib::Paths solution = {};
        ClipperLib::ClipperOffset co;
        co.AddPaths(paths, ClipperLib::jtSquare, ClipperLib::etClosedPolygon);
        co.Execute(solution, -offset * F_TO_CLIP);
        submap = pmap::fromClipper(solution);
    } else {
        submap = map;
    }
}

void pmap::loadMap(std::string filename, FMap &map) {
    std::ifstream ifs(filename.c_str());
    if (ifs.fail()) {
        std::cout << "File " << filename << " cannot be opened or found."  << std::endl;
        exit(EXIT_FAILURE);
    }
    /// Load the map:
    std::string token;
    FPolygon poly(0);
    bool isBorder = false;
    double scale = 1.0;
    while (!ifs.eof()) {
        ifs >> token;
        if (token == "[SCALE]") {
            ifs >> scale;
        } else if (token == "[BORDER]") {
            isBorder = true;
        } else if (token == "[OBSTACLE]") {
            ClipperLib::Path path = pmap::toClipper(poly);
            if (isBorder) {
                isBorder = false;
                if (ClipperLib::Orientation(path) == 0) {
                    changeOrientation(poly);
                }
            } else {
                if (ClipperLib::Orientation(path) == 1) {
                    changeOrientation(poly);
                }
            }
            map.push_back(poly);
            poly.clear();
        } else {
            if (!ifs.eof()) {
                double x, y;
                x = stod(token) * scale;
                ifs >> y;
                y *= scale;
                FPoint p(x, y);
                poly.push_back(p);
            }
        }
    }
    /// Last obstacle:
    ClipperLib::Path path = pmap::toClipper(poly);
    if (isBorder) {
        if (ClipperLib::Orientation(path) == 0) {
            changeOrientation(poly);
        }
    } else {
        if (ClipperLib::Orientation(path) == 1) {
            changeOrientation(poly);
        }
    }
    map.push_back(poly);
    /// Find width of the frame:
    auto ymax = std::numeric_limits<double>::min();
    auto ymin = std::numeric_limits<double>::max();
    for (auto &polygon : map) {
        for (auto &p : polygon) {
            if (p.y > ymax) ymax = p.y;
            if (p.y < ymin) ymin = p.y;
        }
    }
    double w = ((ymax - ymin) * 0.05);
    /// Apply the width and round:
    /*
    auto rounder = static_cast<int>(1e6);
    for (auto &polygon : map) {
        for (auto &p : polygon) {
            p.x = round((p.x + w) * rounder) / rounder;
            p.y = round((p.y + w) * rounder) / rounder;
        }
    }*/
}

double pmap::deg2rad(double deg) {
    return (deg * M_PI) / 180.0;
}

double pmap::rad2deg(double rad) {
    return (rad * 180) / M_PI;
}

