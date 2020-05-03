//
// Created by David Woller on 6.10.18.
//

#ifndef GLNS_PLANNER_H
#define GLNS_PLANNER_H

#include <map>
#include <random>

#include "tour.h"
#include "heuristic.h"
#include "utils.hpp"
#include "polygon.h"



namespace glns {

class Canvas;

class Planner {
public:

    double finalWeight;
    double finalTime;

    Planner();
    void run(Canvas *canvas, int argc, char *argv[], std::string instance);

    void initHeuristics();
    void precomputePolyToPolyDistances();
    Tour initRandomInsertionTour();

    int getRandomInt(int from, int to);
    static double getTourLength(Tour &tour);
    int getRandomNumber(int from, int to);

    int unifiedSetSelection(Tour partialTour, float lambda);
    int cheapestSetSelection(Tour partialTour);
    Tour unifiedInsertion(Tour partialTour, float lambda, float my);

    bool comparePolygonsRemovalCost(int v1, int v2);
    Tour segmentRemoval(Tour tour, int N_r);
    Tour distanceRemoval(Tour tour, int N_r, float lambda);
    Tour worstRemoval(Tour tour, int N_r, float lambda);
    Tour removalFramework(Tour tour, float lambda);
    Tour removeInsert(Tour current, const std::string& phase);

    Heuristic * selectInsertionHeuristic(std::string phase);
    Heuristic * selectRemovalHeuristic(std::string phase);
    void updateHeuristicsWeights(double epsilon);

    bool acceptTrial(double trialCost, double currentCost, double temperature);
    bool acceptTrialNoParam(double trialCost, double currentCost, double probAccept);

    Tour reOpt(const Tour& tour);
    Tour moveOpt(Tour tour, int NMove);
    Tour optCycle(Tour tour, int NMove, std::string mode);

    Tour solve(Canvas *canvas, const std::string& mode, float maxTime, float tourBudget);

private:
    //std::vector<pmap::geom::FPolygon> polygons;
    std::vector<Polygon> polygons;
    std::vector<Heuristic> insertionHeuristics;
    std::vector<Heuristic> removalHeuristics;
    std::minstd_rand generator;
    std::vector<std::vector<double> > polyToPolyDistances;



};

}

#endif //GLNS_PLANNER_H
