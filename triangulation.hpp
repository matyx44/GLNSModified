/**
 * File:    triangulation.hpp
 *
 * Date:    10.10.19
 * Author:  Jan Mikula
 * E-mail:  mikula.miki@gmail.com
 *
 */

#ifndef POLY_MAPS_TRIANGULATION_HPP
#define POLY_MAPS_TRIANGULATION_HPP

#include "geom.hpp"

namespace pmap {

    namespace triangulation {

        void generateTriangularMesh(const pmap::geom::FMap &areaIn,
                                    double visibilityRangeIn,
                                    pmap::geom::FPoints &locationsOut,
                                    pmap::geom::FPolygons &polygonsOut);
        void generateTriangularMeshFromPolygon(const pmap::geom::FPolygon &areaIn,
                                    double visibilityRangeIn,
                                    pmap::geom::FPoints &locationsOut,
                                    pmap::geom::FPolygons &polygonsOut);

    }

}

#endif //POLY_MAPS_TRIANGULATION_HPP
