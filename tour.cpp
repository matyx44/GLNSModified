//
// Created by David Woller on 5.10.18.
//

#include <iostream>
#include <random>
#include <algorithm>
#include <iterator>
#include <chrono>
#include "tour.h"

using namespace glns;

void Tour::initRandomTour(std::vector<Set> &sets, std::vector<std::vector<Edge> > &edgeMatrix) {
    // clear possible previous random tour
    vertices.clear();
    edges.clear();
    // uniformly randomly shuffle set indices from 0 to noOfSets - 1
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(sets.begin(), sets.end(), g);

    // uniformly randomly select a vertex from each set
    for (auto set:sets) {
        std::uniform_int_distribution<int> dist(0, set.vertices.size() - 1);
        int randIndex = dist(g);
        vertices.push_back(set.vertices[randIndex]);
    }

    // get corresponding edges from edgeMatrix
    for (int i = 0; i < vertices.size(); i++) {
        edges.push_back(edgeMatrix[vertices[i].id][vertices[(i+1) % vertices.size()].id]);
    }

}

