/**
 * File:    geom.hpp
 *
 * Date:    4.12.18
 * Author:  Jan Mikula
 * E-mail:  mikula.miki@gmail.com
 *
 */

#ifndef POLY_MAPS_GEOM_HPP
#define POLY_MAPS_GEOM_HPP

#include <cmath>
#include <vector>

namespace pmap {

    namespace geom {

        typedef double geom_float;

        struct FPoint {

            geom_float x;
            geom_float y;

            FPoint();

            FPoint(geom_float x, geom_float y);

            FPoint operator+(const FPoint &p) const;

            FPoint operator-(const FPoint &p) const;

            FPoint operator*(geom_float f) const;

            FPoint operator/(geom_float f) const;

            bool operator==(const FPoint &p) const;

            bool operator!=(const FPoint &p) const;

            geom_float vectNorm() const;

            geom_float distanceTo(FPoint p) const;
        };

        typedef std::vector<FPoint> FPoints;
        typedef FPoints FPolygon;
        typedef std::vector<FPolygon> FPolygons;
        typedef FPolygons FMap;

        struct OFPoint {

            geom_float x;
            geom_float y;
            geom_float phi;

            OFPoint();

            OFPoint(geom_float x, geom_float y, geom_float phi = 0.0);

            explicit OFPoint(FPoint p, geom_float phi = 0.0);

            OFPoint operator+(const OFPoint &p) const;

            OFPoint operator-(const OFPoint &p) const;

            bool operator==(const OFPoint &p) const;

            bool operator!=(const OFPoint &p) const;

            FPoint point() const;

            geom_float distanceTo(FPoint p) const;

            geom_float distanceTo(OFPoint p) const;
        };

        typedef std::vector<OFPoint> OFPoints;
        typedef OFPoint FPosition;
        typedef OFPoints FPositions;

        struct WFPoint {

            geom_float x;
            geom_float y;
            geom_float w;

            WFPoint();

            WFPoint(geom_float x, geom_float y, geom_float w = 0.0);

            explicit WFPoint(FPoint p, geom_float w = 0.0);

            bool operator==(const WFPoint &p) const;

            bool operator!=(const WFPoint &p) const;

            FPoint point() const;

            geom_float distanceTo(FPoint p) const;

            geom_float distanceTo(WFPoint p) const;
        };

        typedef std::vector<WFPoint> WFPoints;

        struct WOFPoint {

            geom_float x;
            geom_float y;
            geom_float phi;
            geom_float w;

            WOFPoint();

            WOFPoint(geom_float x, geom_float y, geom_float phi = 0.0, geom_float w = 0.0);

            explicit WOFPoint(FPoint p, geom_float phi = 0.0, geom_float w = 0.0);

            explicit WOFPoint(OFPoint p, geom_float w = 0.0);

            explicit WOFPoint(WFPoint p, geom_float phi = 0.0);

            bool operator==(const WOFPoint &p) const;

            bool operator!=(const WOFPoint &p) const;

            FPoint point() const;

            OFPoint opoint() const;

            WFPoint wpoint() const;

            geom_float distanceTo(FPoint p) const;

            geom_float distanceTo(OFPoint p) const;

            geom_float distanceTo(WFPoint p) const;

            geom_float distanceTo(WOFPoint p) const;
        };

        typedef std::vector<WOFPoint> WOFPoints;
        typedef WOFPoint FParticle;
        typedef WOFPoints FParticles;

    }
}

#endif //POLY_MAPS_GEOM_HPP
