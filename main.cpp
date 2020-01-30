#include <iostream>
#include <gtkmm/application.h>
#include <gtkmm/window.h>
#include "planner.h"


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

    Planner planner;
    planner.run(nullptr, argc, argv);
    return 0;
}

