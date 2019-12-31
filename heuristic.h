//
// Created by David Woller on 23.10.18.
//

#ifndef GLNS_HEURISTIC_H
#define GLNS_HEURISTIC_H

#include <string>
#include <utility>
#include <map>

namespace glns {

    class Heuristic {
    public:
Heuristic() = default;
Heuristic(std::string name, float lambda, float my);

std::string name;
float lambda;
float my;
std::map<std::string, float> scores;
std::map<std::string, int> counts;
std::map<std::string, float> weights;
    };
}

#endif //GLNS_HEURISTIC_H
