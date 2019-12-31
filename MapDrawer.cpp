/**
 * File:    MapDrawer.cpp
 *
 * Date:    4.12.18
 * Author:  Jan Mikula
 * E-mail:  mikula.miki@gmail.com
 *
 */

#include <limits>
#include <iostream>

#include "MapDrawer.hpp"
#include "utils.hpp"

using namespace pmap;
using namespace pmap::draw;
using namespace pmap::geom;

MapDrawer::MapDrawer(pmap::geom::FMap &map, double resolution) {
    init(map, resolution);
}

void MapDrawer::init(pmap::geom::FMap &map, double resolution) {
    this->resolution = resolution;
    double xMin, xMax, yMin, yMax;
    getMapLimits(map, xMin, xMax, yMin, yMax);
    sizeX = (unsigned int) ((xMax + xMin) / resolution);
    sizeY = (unsigned int) ((yMax + yMin) / resolution);
    getObstacles(map, obstacles);
}

void MapDrawer::openPDF(std::string filePdf) {
    surface = cairo_pdf_surface_create(filePdf.c_str(), sizeX, sizeY);
}

void MapDrawer::closePDF() {
    cairo_surface_flush(surface);
    cairo_surface_destroy(surface);
}

void MapDrawer::newImage() {
    surface = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, sizeX, sizeY);
}

void MapDrawer::drawPlane(draw::RGB fillColor) {
    fillColor.devide255();
    cairo_t *cr = cairo_create(surface);
    cairo_move_to(cr, 0, 0);
    cairo_line_to(cr, sizeX, 0);
    cairo_line_to(cr, sizeX, sizeY);
    cairo_line_to(cr, 0, sizeY);
    cairo_close_path(cr);
    cairo_set_source_rgb(cr, fillColor.r, fillColor.g, fillColor.b);
    cairo_fill(cr);
    cairo_destroy(cr);
}

void MapDrawer::drawPoint(FPoint p, RGB fillColor, double radius, double opacity) {
    fillColor.devide255();
    cairo_t *cr = cairo_create(this->surface);
    cairo_arc(cr, p.x / resolution, sizeY - p.y / resolution, radius, 0, 2 * M_PI);
    cairo_set_source_rgba(cr, fillColor.r, fillColor.g, fillColor.b, opacity);
    cairo_fill(cr);
    cairo_destroy(cr);
}

void MapDrawer::drawPoints(FPoints &points, RGB fillColor, double radius, double opacity) {
    for (auto &p : points) {
        drawPoint(p, fillColor, radius, opacity);
    }
}

void MapDrawer::drawLine(FPoint p1, FPoint p2, double lineWidth, RGB lineColor, double opacity) {
    lineColor.devide255();
    cairo_t *cr = cairo_create(this->surface);
    cairo_move_to(cr, p1.x / resolution, sizeY - p1.y / resolution);
    cairo_line_to(cr, p2.x / resolution, sizeY - p2.y / resolution);
    cairo_set_line_width(cr, lineWidth);
    cairo_set_source_rgba(cr, lineColor.r, lineColor.g, lineColor.b, opacity);
    cairo_set_line_cap(cr, CAIRO_LINE_CAP_ROUND);
    cairo_set_line_join(cr, CAIRO_LINE_JOIN_ROUND);
    cairo_stroke(cr);
    cairo_destroy(cr);
}

void MapDrawer::drawTour(FPoints &tour, double lineWidth, RGB lineColor, double opacity) {
    for (unsigned int i = 0; i < tour.size() - 1; ++i) {
        drawLine(tour[i], tour[i + 1], lineWidth, lineColor, opacity);
    }
}

void MapDrawer::drawArc(FPoint p, double radius, double angle1, double angle2,
                        double lineWidth, RGB lineColor, double opacity) {
    lineColor.devide255();
    cairo_t *cr = cairo_create(this->surface);
    cairo_arc(cr, p.x / resolution, sizeY - p.y / resolution,
              radius / resolution, angle1,
              angle2);
    cairo_set_line_width(cr, lineWidth);
    cairo_set_source_rgba(cr, lineColor.r, lineColor.g, lineColor.b, opacity);
    cairo_set_line_cap(cr, CAIRO_LINE_CAP_ROUND);
    cairo_set_line_join(cr, CAIRO_LINE_JOIN_ROUND);
    cairo_stroke(cr);
    cairo_destroy(cr);
}

void MapDrawer::drawPolygon(FPolygon &poly, bool fill, RGB fillColor, bool stroke, double lineWidth,
                            RGB lineColor, double opacity) {
    lineColor.devide255();
    fillColor.devide255();
    if (!poly.empty() && (fill || stroke)) {
        cairo_t *cr = cairo_create(this->surface);
        cairo_move_to(cr, poly[0].x / resolution, sizeY - poly[0].y / resolution);
        for (unsigned int i = 1; i < poly.size(); ++i) {
            cairo_line_to(cr, (poly[i].x) / resolution,
                          sizeY - poly[i].y / resolution);
        }
        cairo_close_path(cr);
        if (fill) {
            cairo_set_source_rgba(cr, fillColor.r, fillColor.g, fillColor.b, opacity);
            cairo_fill_preserve(cr);
        }
        if (stroke) {
            cairo_set_line_width(cr, lineWidth);
            cairo_set_source_rgba(cr, lineColor.r, lineColor.g, lineColor.b, opacity);
            cairo_set_line_cap(cr, CAIRO_LINE_CAP_ROUND);
            cairo_set_line_join(cr, CAIRO_LINE_JOIN_ROUND);
            cairo_stroke(cr);
        }
        cairo_destroy(cr);
    }
}

void MapDrawer::drawPolygon(FPolygon &poly, RGB fillColor, double lineWidth, RGB lineColor, double opacity) {
    drawPolygon(poly, true, fillColor, true, lineWidth, lineColor, opacity);
}

void MapDrawer::drawPolygon(FPolygon &poly, RGB fillColor, double opacity) {
    drawPolygon(poly, true, fillColor, false, 0, {}, opacity);
}

void MapDrawer::drawPolygon(FPolygon &poly, double lineWidth, RGB lineColor, double opacity) {
    drawPolygon(poly, false, {}, true, lineWidth, lineColor, opacity);
}

void
MapDrawer::drawPolygons(FPolygons &polys, bool fill, RGB fillColor, bool stroke, double lineWidth,
                        RGB lineColor, double opacity) {
    for (auto &poly : polys) {
        drawPolygon(poly, fill, fillColor, stroke, lineWidth, lineColor, opacity);
    }
}

void
MapDrawer::drawPolygons(FPolygons &polys, RGB fillColor, double lineWidth, RGB lineColor, double opacity) {
    drawPolygons(polys, true, fillColor, true, lineWidth, lineColor, opacity);
}

void MapDrawer::drawPolygons(FPolygons &polys, RGB fillColor, double opacity) {
    drawPolygons(polys, true, fillColor, false, 0, {}, opacity);
}

void MapDrawer::drawPolygons(FPolygons &polys, double lineWidth, RGB lineColor, double opacity) {
    drawPolygons(polys, false, {}, true, lineWidth, lineColor, opacity);
}

void MapDrawer::drawObstacles(RGB fillColor, double opacity) {
    for (auto &obstacle : obstacles) {
        drawPolygon(obstacle, fillColor, opacity);
    }
}

void MapDrawer::drawMap() {
    drawPlane();
    drawObstacles();
}

void MapDrawer::toPng(std::string strPNG) {
    cairo_surface_write_to_png(surface, strPNG.c_str());
}

cairo_surface_t *MapDrawer::getSurface() const {
    return surface;
}
