//
// Created by David Woller on 20.9.18.
//

#include "canvas.h"
#include <cairomm/context.h>
#include <cmath>
#include <utility>
#include <iostream>

using namespace glns;

Canvas::~Canvas() = default;

Canvas::Canvas(float scale, Point grab, Point shift, int argc, char *argv[]) : scale(scale), grab(grab), shift(shift){
    add_events(Gdk::BUTTON_PRESS_MASK | Gdk::BUTTON_RELEASE_MASK | Gdk::SCROLL_MASK | Gdk::SMOOTH_SCROLL_MASK |
               Gdk::BUTTON1_MOTION_MASK);
    Dispatcher.connect(sigc::mem_fun(*this, &Canvas::onNotification));

    plannerThread = new std::thread(
            [this, argc, argv]
            {
                planner.run(this, argc, argv);
            });
    plannerThread->detach();
    }

bool Canvas::on_draw(const Cairo::RefPtr<Cairo::Context> &cr) {
    // This is where we draw on the window
    Gtk::Allocation allocation = get_allocation();
    const int width = allocation.get_width();
    const int height = allocation.get_height();
    cr->translate(shift.x, shift.y);
    cr->scale(scale, scale);

    //background
    cr->save();
    cr->set_source_rgb(1.0, 1.0, 1.0);
    cr->paint();
    cr->restore();


    cr -> save();
    // int x_top = width/(4 * scale);
    // int y_top = -height/(4 * scale);
    int x_top = 100;
    int y_top = -100;
    cr->set_line_width(1.0 / scale);
    cr->set_source_rgba(0.0, 0.9, 0.0, 0.5);
    cr->move_to(0, 0);
    cr->line_to(0,y_top);
    cr->line_to(0 + 0.3,y_top + 1);
    cr->move_to(0,y_top);
    cr->line_to(0 - 0.3,y_top + 1);

    cr->move_to(0, 0);
    cr->line_to(x_top, 0);
    cr->line_to(x_top - 1, 0 + 0.3);
    cr->move_to(x_top, 0);
    cr->line_to(x_top - 1, 0 - 0.3);

    cr->set_font_size(15 / scale);
    cr->move_to(x_top, 0);
    cr->show_text("x = 100");
    cr->move_to(0, y_top);
    cr->show_text("y = 100");


    cr->stroke();
    cr -> restore();

    // calling vertices drawing
    for (auto set:sets) {
        cr->set_source_rgba(set.rgb[0], set.rgb[1], set.rgb[2], 0.8);
        for (auto vertex:set.vertices) {
            vertex.draw(cr, 3);
        }
    }

    // drawing tour
    cr->set_source_rgba(1, 0, 0, 1);
    for (auto vertex:tour.vertices) {
        vertex.draw(cr, 1);
    }
    for (auto edge:tour.edges) {
        edge.draw(cr, vertices);
    }

    // drawing best tour
    cr->set_source_rgba(0, 0, 0, 1);
    for (auto vertex:bestTour.vertices) {
        vertex.draw(cr, 1);
    }
    for (auto edge:bestTour.edges) {
        edge.draw(cr, vertices);
    }





    return true;
}

bool Canvas::on_scroll_event(GdkEventScroll *ev) {
    // Update scale according to mouse scroll
    scale -= ev->delta_y / 10.0;
    if (scale < 0.1) { scale = 0.1; }
    queue_draw();
    return true;
}

bool Canvas::on_button_press_event(GdkEventButton *event) {
    if ((event->type == GDK_BUTTON_PRESS) && (event->button == 1)) {
        grab.x = event->x;
        grab.y = event->y;
    }
    return true;
}

bool Canvas::on_button_release_event(GdkEventButton *event) {
    if( (event->type == GDK_BUTTON_RELEASE) && (event->button == 1) ) {
        shift.x += event->x - grab.x;
        shift.y += event->y - grab.y;
        queue_draw();
    }

    return true;
}

bool Canvas::on_motion_notify_event(GdkEventMotion *event) {
    shift.x += event->x - grab.x;
    shift.y += event->y - grab.y;
    grab.x = event->x;
    grab.y = event->y;
    queue_draw();
    // std::cout << "MOVE: " << event->x << " " << event->x << std::endl;
    return true;
}

void Canvas::setTour(Tour tour) {
    this->tour = tour;
}

void Canvas::setBestTour(Tour tour) {
    this->bestTour = tour;
}

void Canvas::notify() {
    // Notify is called from other than drawing thread and activates callback in drawing thread
    Dispatcher.emit();
}

void Canvas::onNotification() {
    // Callback function to activate redrawing in drawing thread
    queue_draw();
}

void Canvas::setData(std::vector<Set> sets, std::vector<Vertex> vertices) {
    this->sets = sets;
    this->vertices = vertices;
}








