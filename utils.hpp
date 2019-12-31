/**
 * File:    utils.hpp
 *
 * Date:    4.12.18
 * Author:  Jan Mikula
 * E-mail:  mikula.miki@gmail.com
 *
 */

#ifndef ROB_MAPS_UTILS_HPP
#define ROB_MAPS_UTILS_HPP

#include <ctime>

#include "clipper.hpp"
#include "geom.hpp"

#define F_TO_CLIP 32768

namespace pmap {

    double deg2rad(double deg);

    double rad2deg(double rad);

    pmap::geom::FPolygon getRegularPolygon(pmap::geom::FPoint center, double radius, unsigned int N);

    ClipperLib::Path getRegularClipPolygon(pmap::geom::FPoint center, double radius, unsigned int N);

    ClipperLib::IntPoint toClipper(const pmap::geom::FPoint &fPoint);

    ClipperLib::Path toClipper(const pmap::geom::FPolygon &fPolygon);

    ClipperLib::Paths toClipper(const pmap::geom::FMap &fMap);

    pmap::geom::FPoint fromClipper(const ClipperLib::IntPoint &cp);

    pmap::geom::FPolygon fromClipper(const ClipperLib::Path &p);

    pmap::geom::FMap fromClipper(const ClipperLib::Paths &ps);

    void clip(ClipperLib::ClipType ct, ClipperLib::Path &subj, ClipperLib::Path &clip, ClipperLib::Paths &solution);

    void clip(ClipperLib::ClipType ct, ClipperLib::Paths &subj, ClipperLib::Path &clip, ClipperLib::Paths &solution);

    void clip(ClipperLib::ClipType ct, ClipperLib::Path &subj, ClipperLib::Paths &clip, ClipperLib::Paths &solution);

    void clip(ClipperLib::ClipType ct, ClipperLib::Paths &subj, ClipperLib::Paths &clip, ClipperLib::Paths &solution);

    /*
    bool pointInPolygon(pmap::geom::FPoint &p, pmap::geom::FPolygon &poly);
    */

    void offsetMap(pmap::geom::FMap &map, pmap::geom::FMap &submap, double offset);

    void getMapLimits(pmap::geom::FMap &map, double &xMin, double &xMax, double &yMin, double &yMax);

    void getObstacles(pmap::geom::FMap &map, pmap::geom::FPolygons &obstacles);

    void loadMap(std::string filename, pmap::geom::FMap &map);

}

#endif //ROB_MAPS_UTILS_HPP
