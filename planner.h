//
// Created by David Woller on 6.10.18.
//

#ifndef GLNS_PLANNER_H
#define GLNS_PLANNER_H

#include <map>
#include <random>

#include "tour.h"
#include "heuristic.h"


namespace glns {

class Canvas;

class Planner {
public:
    Planner();
    void run(Canvas *canvas, int argc, char *argv[]);

    void precomputeSetVertexDistances();
    void initHeuristics();

    Tour initRandomTour();
    Tour initRandomInsertionTour();
    Tour initPartialTour(int length);

    Vertex getRandomVertex(std::vector<Vertex> &vertices);
    Tour removeVertexFromTour(Tour tour, Vertex vertex);
    static float getTourWeight(Tour &tour);
    int getRandomNumber(int from, int to);

    int unifiedSetSelection(Tour partialTour, float lambda);
    int cheapestSetSelection(const Tour& partialTour);
    Tour unifiedInsertion(Tour partialTour, float lambda, float my);

    Tour segmentRemoval(Tour tour, int N_r);
    Tour distanceRemoval(Tour tour, int N_r, float lambda);
    Tour worstRemoval(Tour tour, int N_r, float lambda);
    Tour removalFramework(Tour tour, float lambda);
    Tour removeInsert(Tour current, const std::string& phase);
    static void removeEdge(Edge &edge, Tour &tour);

    Heuristic * selectInsertionHeuristic(std::string phase);
    Heuristic * selectRemovalHeuristic(std::string phase);
    void updateHeuristicsWeights(float epsilon);
    void printWeights();

    bool acceptTrial(float trialCost, float currentCost, float temperature);
    bool acceptTrialNoParam(float trialCost, float currentCost, float probAccept);

    Tour reOpt(const Tour& tour);
    Tour moveOpt(Tour tour, int NMove);
    Tour optCycle(Tour tour, int NMove, std::string mode);

    Tour solve(Canvas *canvas, const std::string& mode, float maxTime, float tourBudget);

private:
    std::vector<Vertex> vertices;
    std::vector<Set> sets;
    std::vector<std::vector<float> > edgeMatrix;
    std::vector<std::vector<float> > setVertexDistances; // (set.id, vertex.id) -> minDistance
    std::vector<Heuristic> insertionHeuristics;
    std::vector<Heuristic> removalHeuristics;
    std::minstd_rand generator;

    int shiftSize; // if input data aren't indexed from zero, this value stores the offset
};

}

#endif //GLNS_PLANNER_H
