/**
 * File:    MapDrawer.hpp
 *
 * Date:    4.12.18
 * Author:  Jan Mikula
 * E-mail:  mikula.miki@gmail.com
 *
 */

#ifndef ROB_MAPS_MAPDRAWER_HPP
#define ROB_MAPS_MAPDRAWER_HPP

#include "geom.hpp"

#include <cairo/cairo.h>
#include <cairo/cairo-pdf.h>
#include <string>

#define PMAP_DRAW_DEFAULT_RESOLUTION 0.008
#define PMAP_DRAW_COL_BLACK {0, 0, 0}
#define PMAP_DRAW_COL_DIMGRAY {105, 105, 105}
#define PMAP_DRAW_COL_GRAY {128, 128, 128}
#define PMAP_DRAW_COL_DARKGRAY {169, 169, 169}
#define PMAP_DRAW_COL_SILVER {192, 192, 192}
#define PMAP_DRAW_COL_LIGHTGREY {211, 211, 211}
#define PMAP_DRAW_COL_GAINSBORO {220, 220, 220}
#define PMAP_DRAW_COL_WHITE {255, 255, 255}
#define PMAP_DRAW_COL_RED {255, 0, 0}
#define PMAP_DRAW_COL_GREEN {0, 255, 0}
#define PMAP_DRAW_COL_BLUE {0, 0, 255}
#define PMAP_DRAW_COL_YELLOW {255, 255, 0}
#define PMAP_DRAW_COL_LIGHTYELLOW {255, 255, 153}
#define PMAP_DRAW_COL_MAGENTA {255, 0, 255}
#define PMAP_DRAW_COL_CYAN {0, 255, 255}
#define PMAP_DRAW_COL_DEEPSKYBLUE {0, 191, 255}
#define PMAP_DRAW_COL_LIMEGREEN {50, 205, 50}
#define PMAP_DRAW_COL_ORANGE {255, 165, 0}
#define PMAP_DRAW_COL_KHAKI {240, 230, 140}
#define PMAP_DRAW_COL_PURPLE {128, 0, 128}
#define PMAP_DRAW_COL_INDIGO {75, 0, 130}

namespace pmap {
    namespace draw {

        struct RGB {
            double r;
            double g;
            double b;

            void devide255() {
                r = r / 255;
                g = g / 255;
                b = b / 255;
            }
        };

        class MapDrawer {
        public:
            explicit MapDrawer(pmap::geom::FMap &map, double resolution = PMAP_DRAW_DEFAULT_RESOLUTION);

            void init(pmap::geom::FMap &map, double resolution);

            void openPDF(std::string filePdf);

            void newImage();

            void closePDF();

            void drawPlane(draw::RGB fillColor = PMAP_DRAW_COL_WHITE);

            void drawPoint(pmap::geom::FPoint p, draw::RGB fillColor, double radius, double opacity = 1.0);

            void drawPoints(pmap::geom::FPoints &points, draw::RGB fillColor, double radius, double opacity = 1.0);

            void drawLine(pmap::geom::FPoint p1, pmap::geom::FPoint p2, double lineWidth, draw::RGB lineColor,
                          double opacity = 1.0);

            void drawTour(pmap::geom::FPoints &tour, double lineWidth, draw::RGB lineColor, double opacity = 1.0);

            void drawArc(pmap::geom::FPoint p, double radius, double angle1, double angle2,
                         double lineWidth, draw::RGB lineColor, double opacity = 1.0);

            void drawPolygon(pmap::geom::FPolygon &poly, draw::RGB fillColor, double lineWidth, draw::RGB lineColor,
                             double opacity = 1.0);

            void drawPolygon(pmap::geom::FPolygon &poly, draw::RGB fillColor, double opacity = 1.0);

            void drawPolygon(pmap::geom::FPolygon &poly, double lineWidth, draw::RGB lineColor, double opacity = 1.0);

            void drawPolygons(pmap::geom::FPolygons &polys, draw::RGB fillColor, double lineWidth, draw::RGB lineColor, double opacity = 1.0);

            void drawPolygons(pmap::geom::FPolygons &polys, draw::RGB fillColor, double opacity = 1.0);

            void
            drawPolygons(pmap::geom::FPolygons &polys, double lineWidth, draw::RGB lineColor, double opacity = 1.0);

            void drawObstacles(draw::RGB fillColor = PMAP_DRAW_COL_BLACK, double opacity = 1.0);

            void drawMap();

            void toPng(std::string strPNG);

            cairo_surface_t *getSurface() const;

        private:
            double resolution;
            unsigned int sizeX, sizeY;
            pmap::geom::FPolygons obstacles;
            cairo_surface_t *surface;

            void drawPolygon(pmap::geom::FPolygon &poly, bool fill, draw::RGB fillColor,
                             bool stroke, double lineWidth, draw::RGB lineColor, double opacity = 1.0);

            void drawPolygons(pmap::geom::FPolygons &polys, bool fill, draw::RGB fillColor,
                              bool stroke, double lineWidth, draw::RGB lineColor, double opacity = 1.0);
        };
    }
}

#endif //ROB_MAPS_MAPDRAWER_H
