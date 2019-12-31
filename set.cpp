//
// Created by David Woller on 5.10.18.
//


#include "set.h"
#include <utility>
#include <chrono>
#include <random>

using namespace glns;

std::minstd_rand Set::generator = std::minstd_rand();

Set::Set() {
    std::uniform_real_distribution<float> distribution(0.0,1.0);
    rgb = {distribution(generator), distribution(generator), distribution(generator)};
}

Set::Set(std::vector<Vertex> vertices, int id) : vertices(std::move(vertices)), id(id) {
    std::uniform_real_distribution<float> distribution(0.0,1.0);
    rgb = {distribution(generator), distribution(generator), distribution(generator)};
}
