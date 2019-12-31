/**
 * File:    geom.cpp
 *
 * Date:    4.12.18
 * Author:  Jan Mikula
 * E-mail:  mikula.miki@gmail.com
 *
 */

#include "geom.hpp"

using namespace pmap::geom;

FPoint::FPoint() : x(0.0), y(0.0) {}

FPoint::FPoint(geom_float x, geom_float y) : x(x), y(y) {}

FPoint FPoint::operator+(const FPoint &p) const {
    FPoint r;
    r.x = this->x + p.x;
    r.y = this->y + p.y;
    return r;
}

FPoint FPoint::operator-(const FPoint &p) const {
    FPoint r;
    r.x = this->x - p.x;
    r.y = this->y - p.y;
    return r;
}

FPoint FPoint::operator*(const geom_float f) const {
    FPoint r;
    r.x = this->x * f;
    r.y = this->y * f;
    return r;
}

FPoint FPoint::operator/(const geom_float f) const {
    FPoint r;
    r.x = this->x / f;
    r.y = this->y / f;
    return r;
}

bool FPoint::operator==(const FPoint &p) const {
    return (this->x == p.x) && (this->y == p.y);
}

bool FPoint::operator!=(const FPoint &p) const {
    return !((this->x == p.x) && (this->y == p.y));
}

geom_float FPoint::vectNorm() const {
    return std::sqrt(this->x * this->x + this->y * this->y);
}

geom_float FPoint::distanceTo(const FPoint p) const {
    return (*this - p).vectNorm();
}

OFPoint::OFPoint() : x(0.0), y(0.0), phi(0.0) {}

OFPoint::OFPoint(geom_float x, geom_float y, geom_float phi) : x(x), y(y), phi(phi) {}

OFPoint::OFPoint(FPoint p, geom_float phi) : x(p.x), y(p.y), phi(phi) {}

OFPoint OFPoint::operator+(const OFPoint &p) const {
    OFPoint r;
    r.x = this->x + p.x;
    r.y = this->y + p.y;
    r.phi = this->phi + p.phi;
    return r;
}

OFPoint OFPoint::operator-(const OFPoint &p) const {
    OFPoint r;
    r.x = this->x - p.x;
    r.y = this->y - p.y;
    r.phi = this->phi - p.phi;
    return r;
}

bool OFPoint::operator==(const OFPoint &p) const {
    return (this->x == p.x) && (this->y == p.y) && (this->phi == p.phi);
}

bool OFPoint::operator!=(const OFPoint &p) const {
    return !((this->x == p.x) && (this->y == p.y) && (this->phi == p.phi));
}

FPoint OFPoint::point() const {
    return {this->x, this->y};
}

geom_float OFPoint::distanceTo(const FPoint p) const {
    return this->point().distanceTo(p);
}

geom_float OFPoint::distanceTo(const OFPoint p) const {
    return this->point().distanceTo(p.point());
}

WFPoint::WFPoint() : x(0.0), y(0.0), w(0.0) {}

WFPoint::WFPoint(geom_float x, geom_float y, geom_float w) : x(x), y(y), w(w) {}

WFPoint::WFPoint(FPoint p, geom_float w) : x(p.x), y(p.y), w(w) {}

bool WFPoint::operator==(const WFPoint &p) const {
    return (this->x == p.x) && (this->y == p.y) && (this->w == p.w);
}

bool WFPoint::operator!=(const WFPoint &p) const {
    return !((this->x == p.x) && (this->y == p.y) && (this->w == p.w));
}

FPoint WFPoint::point() const {
    return {this->x, this->y};
}

geom_float WFPoint::distanceTo(const FPoint p) const {
    return this->point().distanceTo(p);
}

geom_float WFPoint::distanceTo(const WFPoint p) const {
    return this->point().distanceTo(p.point());
}

WOFPoint::WOFPoint() : x(0.0), y(0.0), phi(0.0), w(0.0) {}

WOFPoint::WOFPoint(geom_float x, geom_float y, geom_float phi, geom_float w) : x(x), y(y), phi(phi), w(w) {}

WOFPoint::WOFPoint(FPoint p, geom_float phi, geom_float w) : x(p.x), y(p.y), phi(phi), w(w) {}

WOFPoint::WOFPoint(OFPoint p, geom_float w) : x(p.x), y(p.y), phi(p.phi), w(w) {}

WOFPoint::WOFPoint(WFPoint p, geom_float phi) : x(p.x), y(p.y), phi(phi), w(p.w) {}

bool WOFPoint::operator==(const WOFPoint &p) const {
    return (this->x == p.x) && (this->y == p.y) && (this->phi == p.phi) && (this->w == p.w);
}

bool WOFPoint::operator!=(const WOFPoint &p) const {
    return !((this->x == p.x) && (this->y == p.y) && (this->phi == p.phi) && (this->w == p.w));
}

FPoint WOFPoint::point() const {
    return {this->x, this->y};
}

OFPoint WOFPoint::opoint() const {
    return {this->x, this->y, this->phi};
}

WFPoint WOFPoint::wpoint() const {
    return {this->x, this->y, this->w};
}

geom_float WOFPoint::distanceTo(const FPoint p) const {
    return this->point().distanceTo(p);
}

geom_float WOFPoint::distanceTo(const OFPoint p) const {
    return this->point().distanceTo(p.point());
}

geom_float WOFPoint::distanceTo(const WFPoint p) const {
    return this->point().distanceTo(p.point());
}

geom_float WOFPoint::distanceTo(const WOFPoint p) const {
    return this->point().distanceTo(p.point());
}
