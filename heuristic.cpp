//
// Created by David Woller on 17.11.18.
//

#include <string>
#include <utility>
#include "heuristic.h"

glns::Heuristic::Heuristic(std::string name, float lambda, float my): name(std::move(name)), lambda(lambda), my(my) {
    scores["early"] = 0;
    scores["mid"] = 0;
    scores["late"] = 0;

    counts["early"] = 0;
    counts["mid"] = 0;
    counts["late"] = 0;

    weights["early"] = 1;
    weights["mid"] = 1;
    weights["late"] = 1;
}
