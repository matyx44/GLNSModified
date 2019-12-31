#include <iostream>
#include <gtkmm/application.h>
#include <gtkmm/window.h>

#include "canvas.h"

using namespace glns;

int main(int argc, char *argv[]) {
    bool visualize = false;

    for (int i = 0; i < argc; i++) {
        if (argv[i][0] == '-') {
            switch (argv[i][1]) {
                case 'v' :
                    visualize = true;
                    break;
            }
        }
    }

    if (visualize) {
        // Canvas defaults
        float scale = 3;
        Point grab(0,0);
        Point shift(75, 900);

        auto app = Gtk::Application::create("gtkmm.glns");
        Gtk::Window win;
        win.set_title("GLNS");
        win.set_default_size(1200, 1000);
        Canvas canvas(scale, grab, shift, argc, argv); // Planner->run is called in Canvas constructor
        win.add(canvas);
        canvas.show();
        return app->run(win);
    } else {
        Planner planner;
        planner.run(nullptr, argc, argv);
        return 0;
    }
}

