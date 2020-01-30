//
// Created by David Woller on 6.10.18.
//

#include <iostream>
#include <fstream>

#include <cmath>
#include <chrono>
#include <ctime>
#include <cfloat>
#include <random>

#include "planner.h"

#include "utils.hpp"
#include "MapDrawer.hpp"
#include "triangulation.hpp"
#include "simple_intersection.h"
#include "TPP.h"


using namespace glns;

Planner::Planner() {
    generator.seed(static_cast<unsigned long>(std::chrono::system_clock::now().time_since_epoch().count()));
}

void Planner::run(glns::Canvas *canvas, int argc, char *argv[]) {
    std::string input;
    std::string output;
    std::string mode = "default";
    bool outFlag = false;
    bool visualize = false;
    float maxTime = DBL_MAX;
    float tourBudget = 0;

    // Parse parameters
    for (int i = 0; i < argc; i++) {
        if (argv[i][0] == '-') {
            switch (argv[i][1]) {
                case 'v' :
                    visualize = true;
                    break;
                case 'i':
                    input = argv[i + 1];
                    break;
                case 'o':
                    outFlag = true;
                    output = argv[i + 1];
                    break;
                case 't':
                    maxTime = std::stod(argv[i + 1]);;
                    break;
                case 'm':
                    mode = argv[i + 1];
                    break;
                case 'b':
                    tourBudget = std::stod(argv[i + 1]);
                    break;
                default : // Unknown parameter
                    std::cout << "Parameter '" << argv[i] << "' not valid." << std::endl;
                    break;
            }
        }
    }
    input = input + "/GeneratedFiles/";

    std::ofstream file;
    pmap::geom::FMap map;
    pmap::loadMap(input+"potholes.txt", map);

    for (int i = 1; i < map.size(); ++i) {
        std::vector<FPoint> points;
        for (int k = map[i].size() - 1; k >= 0; --k)
            points.emplace_back(map[i][k]);

        Polygon pol(points, i-1);
        polygons.emplace_back(pol);
    }

    std::cout << "Loaded problem " << input << std::endl;
    auto timeStart = std::chrono::high_resolution_clock::now();
    Tour tour = solve(canvas, mode, maxTime, tourBudget);

    //Draw tour
    pmap::draw::MapDrawer md(map);
    md.openPDF(input+"outputTPPPotholes.pdf");
    for (auto &polygon : polygons) {
        for (int i = 0; i < polygon.points.size(); ++i) {
            int j = i == polygon.points.size() - 1 ? 0 : i + 1;
            md.drawLine(polygon.points[i], polygon.points[j], 15.0, PMAP_DRAW_COL_BLACK);
        }
    }
    for (int l = 0; l < tour.points.size(); ++l) {
        int m = l == tour.points.size() - 1 ? 0 : l + 1;
        FPoint p1 = tour.points[l];
        FPoint p2 = tour.points[m];
        md.drawPoint(p1, PMAP_DRAW_COL_BLUE, 10, 1);
        md.drawPoint(p2, PMAP_DRAW_COL_BLUE, 10, 1);
        md.drawLine(p1, p2, 10, PMAP_DRAW_COL_GREEN, 1.0);
    }
    md.closePDF();

}



double min(double a, double b) {
    return (a <= b) ? a : b;
}

double max(double a, double b) {
    return (a >= b) ? a : b;
}

void Planner::initHeuristics() {
    // Adding cheapest insertion heuristic
    Heuristic h = Heuristic("cheapest", -1, 0);
    insertionHeuristics.push_back(h);
    // Adding all variants of unified insertion heuristic
    std::vector<float> insertionLambdas = {0, 1.0/2, static_cast<float>(1/sqrt(2)), 1, static_cast<float>(sqrt(2)), 2, FLT_MAX};
    std::vector<float> mys = {0, 0.25, 0.75};
    for (auto &lambda:insertionLambdas) {
        for (auto &my:mys) {
            insertionHeuristics.emplace_back("unified", lambda, my);
        }
    }

    // add all variants of unified worst removal and distance removal heuristics
    std::vector<float> removalLambdas = {static_cast<float>(1/sqrt(2)), 1, static_cast<float>(sqrt(2)), 2, FLT_MAX};
    std::vector<std::string> names = {"distance", "worst"};
    for (const auto &name:names) {
        for (auto &lambda:removalLambdas) {
            removalHeuristics.emplace_back(name, lambda, -1);
        }
    }
    // add segment removal heuristic
    removalHeuristics.emplace_back("segment", -1, -1);

}

/*
 * Chooses a starting vertex v randomly.
 * Remaining vertices are added using the unified insertion heuristic with lambda = 1 and my = 0.75
 */
Tour Planner::initRandomInsertionTour() {
    Tour tour;
    int indexPolygon1 = getRandomInt(0, polygons.size() - 1);
    int indexPoint1 = getRandomInt(0, polygons[indexPolygon1].points.size() - 1);
    int indexPolygon2 = getRandomInt(0, polygons.size() - 1);
    while(indexPolygon1 == indexPolygon2){
        indexPolygon2 = getRandomInt(0, polygons.size() - 1);
    }
    int indexPoint2 = getRandomInt(0, polygons[indexPolygon2].points.size() - 1);

    tour.polygons.emplace_back(polygons[indexPolygon1]);
    tour.polygons.emplace_back(polygons[indexPolygon2]);
    tour.points.emplace_back(polygons[indexPolygon1].points[indexPoint1]);
    tour.points.emplace_back(polygons[indexPolygon2].points[indexPoint2]);

    float lambda = 1;
    float my = 0.75;

    while(tour.polygons.size() < polygons.size()) {
        tour = unifiedInsertion(tour, lambda, my);
    }

    return tour;
}

int Planner::getRandomInt(int from, int to) {
    std::uniform_int_distribution<int> dist(from, to);
    return dist(generator);
}

double pointToPointDist(FPoint &a, FPoint &b){
    return std::sqrt(imr::geom::CIntersection<FPoint>::squared_distance(a, b));
}

double pointToPolygonDistance(FPoint point, Polygon polygon){
    double minDist = std::numeric_limits<double>::max();
    for (int k = 0; k < polygon.points.size(); ++k) {
        int d = k == polygon.points.size() - 1 ? 0 : k + 1;
        double dist = std::sqrt(imr::geom::CIntersection<FPoint>::point_segment_squared_distance(point, polygon.points[k], polygon.points[d]));
        if(dist < minDist) minDist = dist;
    }
    return minDist;
}

/*
void Planner::precomputePolyToPolyDistances(){
    polyToPolyDistances.reserve(polygons.size());
    for (int i = 0; i < polygons.size(); ++i) {
        polyToPolyDistances[i].reserve(polygons.size());
        for (int j = i+1; j < polygons.size(); ++j) {
            double minDist = std::numeric_limits<double>::max();
            for (auto &point : polygons[i].points) {
                for (int k = 0; k < polygons[j].points.size(); ++k) {
                    int d = k == polygons[j].points.size() - 1 ? 0 : k + 1;
                    double dist = std::sqrt(imr::geom::CIntersection<FPoint>::point_segment_squared_distance(point, polygons[j].points[k], polygons[j].points[d]));
                    if(dist < minDist) minDist = dist;
                }
            }
            polyToPolyDistances[i][j] = minDist;
        }
    }

    for (int i = 0; i < polygons.size(); ++i) {
        for (int j = i + 1; j < polygons.size(); ++j) {
            polyToPolyDistances[j][i] = polyToPolyDistances[i][j];
        }
    }
}*/

double Planner::getTourLength(Tour &tour) {
    double L1 = 0;
    for (int i = 0; i < tour.points.size(); ++i) {
        int j = i == tour.points.size() - 1 ? 0 : i + 1;
        L1 += pointToPointDist(tour.points[i], tour.points[j]);
    }
    return L1;
}

bool compareSetsMinDistV2(std::pair<int, float> &p1, std::pair<int, float> &p2) {
    return (p1.second < p2.second);
}

/*
 * Performs set selection for nearest, random or farthest insertion
 * lambda = 0 ... nearest insertion
 * lambda = 1 ... random insertion
 * lambda = DBL_MAX ... farthest insertion
 */
int Planner::unifiedSetSelection(Tour partialTour, float lambda) {
    // get indices of sets, that are not in partialTour; (P_V \ P_T)
    std::vector<std::pair<int, float>> unusedSetsDists;
    std::vector<bool> isUsed(polygons.size(), false);
    for (auto &v:partialTour.polygons) {
        isUsed[v.id] = true;
    }
    for (int i = 0; i < polygons.size(); i++) {
        if (!isUsed[i]) {
            // for each unused set V_i, define the minimum distance d_i = min dist(V_i, u), where u is from V_T (vertices in partial tour)
            auto minDist = FLT_MAX;
            for(FPoint &p : partialTour.points){
                double dist = pointToPolygonDistance(p, polygons[i]);
                if(dist < minDist) minDist = dist;
            }
            unusedSetsDists.emplace_back(i, minDist);
        }
    }

    // randomly select k = {0...l - 1} according to the unnormalized probability mass function {lambda^0, lambda^1, ... lambda^(l-1)}
    int l = unusedSetsDists.size();
    std::vector<float> weights(l);
    for (int i = 0; i < l; i++) {
        float weight = std::pow(lambda, i);
        if (weight == std::numeric_limits<float>::infinity()) weight = FLT_MAX;
        weights[i] = weight;
    }
    std::discrete_distribution<int> distribution(weights.begin(), weights.end());
    int k = distribution(generator); // generates random number from 0 to l-1, according to weights given

    // sort unused sets according to minDist, take set at index k
    std::sort(unusedSetsDists.begin(), unusedSetsDists.end(), compareSetsMinDistV2);
    int id = unusedSetsDists[k].first;

    return id;
}

int Planner::cheapestSetSelection(Tour& partialTour) {
    std::vector<bool> isUsed(polygons.size(), false);
    for (auto &p : partialTour.polygons) {
        isUsed[p.id] = true;
    }
    double minCost = FLT_MAX;
    int minSetId = -1;
    for (int i = 0; i < polygons.size(); ++i) {
        if(!isUsed[i]){
            double minInsCost = FLT_MAX;
            for (int j = 0; j < partialTour.polygons.size(); ++j) {
                int k = j == partialTour.polygons.size() - 1 ? 0 : j + 1;
                FPoint pointX = partialTour.points[j];
                FPoint pointY = partialTour.points[k];
                Polygon prospectivePolygon = polygons[i];

                double lowerBound = pointToPolygonDistance(pointX, prospectivePolygon)
                        + pointToPolygonDistance(pointY, prospectivePolygon)
                        - pointToPointDist(pointX, pointY);

                if(lowerBound < minCost){
                    FPoint tmpMinPoint = TPP::findOptimalConnectingPointInPolygon(pointX, pointY, prospectivePolygon);
                    double cost = pointToPointDist(pointX, tmpMinPoint)
                                      + pointToPointDist(pointY, tmpMinPoint)
                                      - pointToPointDist(pointX, pointY);
                    if(cost < minCost){
                        minCost = cost;
                        minSetId = i;
                    }
                }
            }
        }
    }
    return minSetId;
}

/*
 * Performs one step of unified insertion heuristics.
 * lambda = 0 ... nearest set insertion
 * lambda = 1 ... random set insertion
 * lambda = -1 ... cheapest insertion
 * lambda = DBL_MAX ... farthest set insertion
 * partialTour given must include at least 2 vertices and at most sets.size() - 1 vertices
 */
Tour Planner::unifiedInsertion(Tour partialTour, float lambda, float my) {
    if (partialTour.polygons.size() < 2) {
        std::cout << "Unified insertion: given tour too short. Returning unchanged tour." << std::endl;
    } else if (partialTour.polygons.size() >= polygons.size()) {
        std::cout << "Unified insertion: given tour too long. Returning unchanged tour." << std::endl;
    } else {
        // Pick a set V_i in P_V \ P_T
        Polygon V_i;
        int i;
        if (lambda == -1)
            i = cheapestSetSelection(partialTour);
        else
            i = unifiedSetSelection(partialTour, lambda);
        V_i = polygons[i];

        double minWeight = FLT_MAX;
        FPoint minPoint;
        int emplaceIndex = -1;
        std::uniform_real_distribution<float> distribution(0, my);

        for (int j = 0; j < partialTour.points.size(); ++j) {
            int k = j == partialTour.points.size() - 1 ? 0 : j + 1;
            FPoint pointX = partialTour.points[j];
            FPoint pointY = partialTour.points[k];

            double lowerBound = pointToPolygonDistance(pointX, V_i)
                    + pointToPolygonDistance(pointY, V_i)
                    - pointToPointDist(pointX, pointY);
            double rand = distribution(generator);

            if(lowerBound < minWeight){
                FPoint tmpMinPoint = TPP::findOptimalConnectingPointInPolygon(pointX, pointY, V_i);
                double weight = (1 + rand) * (pointToPointDist(pointX, tmpMinPoint)
                        + pointToPointDist(pointY, tmpMinPoint)
                        - pointToPointDist(pointX, pointY));

                if(weight < minWeight){
                    minWeight = weight;
                    minPoint = tmpMinPoint;
                    emplaceIndex = k;
                }
            }
        }

        partialTour.polygons.emplace(partialTour.polygons.begin() + emplaceIndex, V_i);
        partialTour.points.emplace(partialTour.points.begin() + emplaceIndex, minPoint);

    }
    return partialTour;
}

/*
 * Removes a continuous segment of the tour of length N_r.
 */
Tour Planner::segmentRemoval(Tour tour, int N_r) {
    int length = tour.polygons.size();
    if (length < 3) {
        std::cout << "segmentRemoval: given tour is too short (<3 vertices). Returning unchanged tour." << std::endl;
    } else if (N_r > (length - 2)) {
        std::cout << "segmentRemoval: N_r given is too large. Returning unchanged tour." << std::endl;
    } else {
        // Uniformly randomly select a vertex
        std::uniform_int_distribution<int> dist(0, tour.polygons.size() - 1);
        int randIndex = dist(generator);
        for (int j = 0; j < N_r; ++j) {
            int index = randIndex < tour.polygons.size() ? randIndex : 0;
            tour.polygons.erase(tour.polygons.begin() + index);
            tour.points.erase(tour.points.begin() + index);
        }
    }

    return tour;
}

Tour Planner::distanceRemoval(Tour tour, int N_r, float lambda) {
    int length = tour.polygons.size();
    if (length < 3) {
        std::cout << "distanceRemoval: given tour is too short (<3 vertices). Returning unchanged tour." << std::endl;
    } else if (N_r > (length - 2)) {
        std::cout << "distanceRemoval: N_r given is too large. Returning unchanged tour." << std::endl;
    } else {
        std::vector<FPoint> V_removed;
        // Randomly remove a vertex from T, add it to V_removed
        int index = getRandomInt(0, length - 1);
        FPoint erasedPoint = tour.points[index];
        tour.polygons.erase(tour.polygons.begin() + index);
        tour.points.erase(tour.points.begin() + index);
        V_removed.push_back(erasedPoint);

        for (int i = 1; i < N_r; ++i) {
            int index = getRandomInt(0, V_removed.size() - 1);
            FPoint seed = V_removed[index];
            for (int j = 0; j < tour.polygons.size(); ++j) {
                tour.polygons[j].removalCost = pointToPointDist(tour.points[j], seed);
            }
            tour = removalFramework(tour, lambda);
        }
    }
    return tour;
}

Tour Planner::worstRemoval(Tour tour, int N_r, float lambda) {
    int length = tour.polygons.size();
    if (length < 3) {
        std::cout << "worstRemoval: given tour is too short (<3 vertices). Returning unchanged tour." << std::endl;
    } else if (N_r > (length - 2)) {
        std::cout << "worstRemoval: N_r given is too large. Returning unchanged tour." << std::endl;
    } else {
        for (int j = 0; j < tour.polygons.size(); ++j) {
            int k = j == tour.polygons.size() - 1 ? 0 : j + 1;
            int l = k == tour.polygons.size() - 1 ? 0 : k + 1;

            tour.polygons[j].removalCost = pointToPointDist(tour.points[j], tour.points[k])
                                         + pointToPointDist(tour.points[k], tour.points[l])
                                         - pointToPointDist(tour.points[j], tour.points[l]);
        }

        for (int i = 0; i < N_r; i++) {
            tour = removalFramework(tour, lambda);
        }
    }
    return tour;
}

bool comparePolygonsRemovalCost(Polygon v1, Polygon v2) {
    return (v1.removalCost < v2.removalCost);
}

Tour Planner::removalFramework(Tour tour, float lambda) {
    // Randomly select k = {0...l - 1} according to the unnormalized probability mass function {lambda^0, lambda^1, ... lambda^(l-1)}
    // Initialize weights
    int l = tour.polygons.size();
    std::vector<float> weights(l);
    for (int i = 0; i < l; i++) {
        float weight = std::pow(lambda, i);
        if (weight == std::numeric_limits<float>::infinity()) weight = DBL_MAX;
        weights[i] = weight;
    }
    // Generate random k
    std::discrete_distribution<int> distribution(weights.begin(), weights.end());
    int k = distribution(generator); // generates random number from 0 to l-1, according to weights given
    // sort vertices according to removalCost, take vertex at index k
    std::vector<Polygon> tmpPolygons(tour.polygons);
    std::sort(tmpPolygons.begin(), tmpPolygons.end(), comparePolygonsRemovalCost);
    int id = tmpPolygons[k].id;
    // Pick the vertex v_j from V_T  with the kth smallest value r_j
    for (int j = 0; j < tour.polygons.size(); ++j) {
        // Remove v_j from tour, remove corresponding edges from and to v_j, add edge between disconnected vertices
        if(tour.polygons[j].id == id){
            tour.polygons.erase(tour.polygons.begin() + j);
            tour.points.erase(tour.points.begin() + j);
            break;
        }
    }
    return tour;
}

/*
 * Selects a random number uniformly randomly from range <from, to>
 */
int Planner::getRandomNumber(int from, int to) {
    std::uniform_int_distribution<int> dist(from, to);
    return dist(generator);
}

Tour Planner::removeInsert(Tour current, const std::string& phase) {
    // Select a removal heuristic R and insertion heuristic I
    Heuristic * R = selectRemovalHeuristic(phase);
    Heuristic * I = selectInsertionHeuristic(phase);

    // Select the number of vertices to remove, N_r, uniformly randomly from {1,...,N_max}
    unsigned long N_max = polygons.size() - 2;
    int N_r = getRandomNumber(1, N_max);

    // Create a copy of current tour
    Tour T_new(current.polygons, current.points);

    // Remove N_r vertices from T_new using R
    if (R->name == "distance") {
        T_new = distanceRemoval(T_new, N_r, R->lambda);
    } else if (R->name == "worst") {
        T_new = worstRemoval(T_new, N_r, R->lambda);
    } else if (R->name == "segment") {
        T_new = segmentRemoval(T_new, N_r);
    }

    // std::cout << "Removed " << N_r << " vertices, " << R.name << " removal heuristic, lambda = " << R.lambda << std::endl;

    // Insert N_r vertices into T_new using I
    while (T_new.polygons.size() < polygons.size()) {
        T_new = unifiedInsertion(T_new, I->lambda, I->my);
    }

    // Update scores for insertion and removal heuristics
    double score = 100 * max(getTourLength(current) - getTourLength(T_new), 0)/getTourLength(current);
    I->scores[phase] += score;
    I->counts[phase] += 1;
    R->scores[phase] += score;
    R->counts[phase] += 1;

    // std::cout << "Inserted " << N_r << " vertices, " << I.name << " insertion heuristic, lambda = " << I.lambda << ", my = " << I.my << std::endl;
    return T_new;
}

/*
 * Returns an insertion heuristic from the insertion heuristics bank.
 * Heuristic is chosen randomly according to a standard roulette wheel mechanism.
 */
Heuristic * Planner::selectInsertionHeuristic(std::string phase) {
    std::vector<float> weights;
    for (auto &h:insertionHeuristics) {
        weights.emplace_back(h.weights[phase]);
    }
    std::discrete_distribution<int> distribution(weights.begin(), weights.end());
    int index = distribution(generator); // generates random number from 0 to l-1, according to weights given
    return &insertionHeuristics[index];
}

/*
 * Returns a removal heuristic from the removal heuristics bank.
 * Heuristic is chosen randomly according to a standard roulette wheel mechanism.
 */
Heuristic * Planner::selectRemovalHeuristic(std::string phase) {
    std::vector<float> weights;
    for (auto &h:removalHeuristics) {
        weights.emplace_back(h.weights[phase]);
    }
    std::discrete_distribution<int> distribution(weights.begin(), weights.end());
    int index = distribution(generator); // generates random number from 0 to l-1, according to weights given
    return &removalHeuristics[index];
}

void Planner::updateHeuristicsWeights(double epsilon) {
    std::string phasesArray[] = {"early", "mid", "late"};
    std::vector<std::string> phases(phasesArray, phasesArray + sizeof(phasesArray)/ sizeof(std::string));

    for (const auto &phase:phases) {
        for (auto &h:insertionHeuristics) {
            if (h.counts[phase] > 0) {
                h.weights[phase] = epsilon * h.scores[phase]/h.counts[phase] + (1 - epsilon) * h.weights[phase];
            }
            h.scores[phase] = 0;
            h.counts[phase] = 0;
        }
        for (auto &h:removalHeuristics) {
            if (h.counts[phase] > 0) {
                h.weights[phase] = epsilon * h.scores[phase]/h.counts[phase] + (1 - epsilon) * h.weights[phase];
            }
            h.scores[phase] = 0;
            h.counts[phase] = 0;
        }
    }

}

bool Planner::acceptTrial(double trialCost, double currentCost, double temperature) {
    double prob1 = exp((currentCost - trialCost)/temperature);
    double prob2 = 1;
    double prob = min(prob1, prob2);

    std::vector<double> weights;
    weights.emplace_back(1-prob); // prob of not accepting at position 0
    weights.emplace_back(prob); // prob of accepting at position 1

    std::discrete_distribution<int> distribution(weights.begin(), weights.end());
    int index = distribution(generator);

    return (bool)index;
}

bool Planner::acceptTrialNoParam(double trialCost, double currentCost, double probAccept) {
    if (trialCost < currentCost) return true;

    std::vector<double> weights;
    weights.emplace_back(1-probAccept); // prob of not accepting at position 0
    weights.emplace_back(probAccept); // prob of accepting at position 1

    std::discrete_distribution<int> distribution(weights.begin(), weights.end());
    int index = distribution(generator);

    return (bool)index;
}

/*
 * Optimizes the tour given, while keeping the set ordering fixed.
 */
Tour Planner::reOpt(const Tour& tour) {
    TPP tpp(tour.polygons, 0.01f);
    tpp.run();
    std::vector<FPoint> points = tpp.outputPoints;
    //The last point is the same as first one.
    points.pop_back();
    Tour bestTour(tour.polygons, points);
    return bestTour;
}

/*
 * Optimizes the tour given by randomly changing set ordering.
 */
Tour Planner::moveOpt(Tour tour, int NMove) {
    Tour bestTour(tour.polygons, tour.points);
    double bestTourLength = getTourLength(bestTour);

    for (int i = 0; i < NMove; i++) {
        //randomly remove a polygon from tour
        int index = getRandomInt(0, tour.polygons.size() - 1);
        Polygon removedPolygon = tour.polygons[index];
        tour.polygons.erase(tour.polygons.begin() + index);
        tour.points.erase(tour.points.begin() + index);

        //find new place for the removed polygon in partial tour
        double minDist = std::numeric_limits<int>::max();
        int minIndex = 0;
        FPoint minPoint;
        for (int j = 0; j < tour.polygons.size(); ++j) {
            int k = j == tour.polygons.size() - 1 ? 0 : j + 1;
            FPoint tmpOptimalPoint = TPP::findOptimalConnectingPointInPolygon(tour.points[j], tour.points[k], removedPolygon);
            double tmpDist = pointToPointDist(tour.points[j], tmpOptimalPoint) + pointToPointDist(tour.points[k], tmpOptimalPoint);
            if(tmpDist < minDist){
                minDist = tmpDist;
                minIndex = j;
                minPoint = tmpOptimalPoint;
            }
        }

        //reinsert polygon at minIndex
        tour.polygons.insert(tour.polygons.begin() + minIndex + 1, 1, removedPolygon);
        tour.points.insert(tour.points.begin() + minIndex + 1, 1, minPoint);

        double newTourLength = getTourLength(tour);
        if(newTourLength < bestTourLength){
            bestTour = tour;
            bestTourLength = newTourLength;
        }
    }

    return bestTour;
}

/*
 * Repeatedly performs moveOpt and reOpt, until there is no improvement.
 */
Tour Planner::optCycle(Tour tour, int NMove, std::string mode) {
    double previousWeight = getTourLength(tour);
    double newWeight = 0;
    while (newWeight < previousWeight) {
        previousWeight = getTourLength(tour);

        if (mode != "fast") {
            tour = reOpt(tour);
        }
        tour = moveOpt(tour, NMove);
        newWeight = getTourLength(tour);
        // std::cout << "previous weight: " << previousWeight << std::endl;
        // std::cout << "new weight     : " << newWeight << std::endl;
    }
    return tour;
}

/*
 * Solves GTSP and returns final tour, with indices shifted so that they are consistent with input data.
 */
Tour Planner::solve(Canvas *canvas, const std::string& mode, float maxTime, float tourBudget) {
    bool visualize = nullptr != canvas;
    std::cout << "Planning..." << std::endl;

    //precomputePolyToPolyDistances();

    // Common parameters
    auto numSets = polygons.size();
    double acceptPercentage = 0.05;
    double epsilon = 0.5;
    __useconds_t uDelay = 0; // delay after finding a better tour in warm trial; in useconds
    // Mode-specific parameters
    int coldTrials;
    int warmTrials;
    int numIterations;
    // if best tour wasn't improved for latestImprovement consecutive iterations, leave initial descent (mid phase)
    int latestImprovement;
    // if best tour wasn't improved for firstImprovement consecutive iterations and there was not an initial improvement, leave warm restart (late phase)
    int firstImprovement;
    double probAccept;
    int NMax;
    int NMove;
    if (mode == "fast") {
        coldTrials = 3;
        warmTrials = 2;
        numIterations = 60 * numSets;
        latestImprovement = numIterations / 4;
        firstImprovement = numIterations / 6;
        probAccept = 10.0 / numIterations;
        NMax = static_cast<int>(min(20, 0.1 * numSets));
        NMove = NMax;
    } else if (mode == "default") {
        coldTrials = 5;
        warmTrials = 3;
        numIterations = 60 * numSets;
        latestImprovement = numIterations / 2;
        firstImprovement = numIterations / 4;
        probAccept = 10.0 / numIterations;
        NMax = static_cast<int>(min(100, 0.3 * numSets));
        NMove = NMax;
    } else if (mode == "slow") {
        coldTrials = 10;
        warmTrials = 5;
        numIterations = 150 * numSets;
        latestImprovement = numIterations / 3;
        firstImprovement = numIterations / 6;
        probAccept = 10.0 / numIterations;
        NMax = static_cast<int>(0.4 * numSets);
        NMove = NMax;
    } else {
        std::cout << "Invalid mode: " << mode << std::endl;
        return Tour();
    }

    // Init planner counters, flags, variables
    int latestImprovementCnt = 1;
    bool firstImprovementFlag = false;
    int warmTrialsCnt = 0;
    int coldTrialsCnt = 1;
    int totalIterCnt = 0;

    auto timeStart = std::chrono::high_resolution_clock::now();
    Tour lowestT; // best tour found overall
    
    // Cold trials loop
    while (coldTrialsCnt <= coldTrials) {
        // build tour from scratch on a cold restart
        Tour bestT = initRandomInsertionTour(); // best tour in this trial
        if (lowestT.points.empty()) {
            lowestT = bestT;
        } else {
            if (getTourLength(lowestT) > getTourLength(bestT)) lowestT = bestT;
        }

        std::string phase = "early";

        // Update selection weights
        if (coldTrialsCnt == 1) {
            initHeuristics();
        } else {
            updateHeuristicsWeights(epsilon);
            // printWeights();
        }

        // Warm restarts loop
        while (warmTrialsCnt <= warmTrials) {
            int iterCount = 1;
            Tour currentT = bestT;
            double temperature = 1.442 * acceptPercentage * getTourLength(bestT);
            double cooling_rate = pow(((0.0005 * getTourLength(lowestT))/(acceptPercentage * getTourLength(currentT))), 1.0/numIterations);

            // If warm restart, use lower temperature
            if (warmTrialsCnt > 0) {
                temperature *= pow(cooling_rate, numIterations/2.0);
                phase = "late";
            }

            //
            while(latestImprovementCnt <= (firstImprovementFlag ? latestImprovement : firstImprovement)) {
                // Move to mid phase after half iterations
                if ((iterCount > numIterations/2) && (phase == "early")) {
                    phase = "mid";
                }

                Tour trial = removeInsert(currentT, phase);

                // Decide whether or not to accept trial
                if (acceptTrialNoParam(getTourLength(trial), getTourLength(currentT), probAccept) || acceptTrial(getTourLength(trial), getTourLength(currentT), temperature)) {
                    if (mode == "slow") trial = optCycle(trial, NMove, mode);
                    currentT = trial;
                }



                if(getTourLength(currentT) < getTourLength(bestT)) {
                    latestImprovementCnt = 1;
                    firstImprovementFlag = true;
                    if ((coldTrialsCnt > 1) && (warmTrialsCnt > 1)) {
                        warmTrialsCnt++;
                    }
                    currentT = optCycle(currentT, NMove, mode); // Locally reoptimize current tour

                    bestT = currentT;
                } else {
                    latestImprovementCnt++;
                }

                // time limit and budget limit check
                auto t_now = std::chrono::high_resolution_clock::now();
                double timeFromStart = std::chrono::duration<double, std::milli>(t_now - timeStart).count();
                if ((timeFromStart/1000 > maxTime) || (getTourLength(bestT) < tourBudget)) {
                    if (timeFromStart/1000 > maxTime) std::cout << "Max time exceeded" << std::endl;
                    if (getTourLength(bestT) < tourBudget) std::cout << "Tour better than budget found" << std::endl;
                    if (getTourLength(lowestT) > getTourLength(bestT)) lowestT = bestT;
                    std::cout << "lowest weight: " << getTourLength(lowestT) << std::endl;
                    return lowestT;
                }

//                std::cout << "timeFromStart: " << timeFromStart << " ms  coldTrialsCnt: " << coldTrialsCnt << "    warmTrialsCnt: " << warmTrialsCnt << "  phase: " << phase << "  iterCount: " << iterCount << "     temperature: " << temperature << " best weight: " << getTourWeight(bestT) << std::endl;

                // cool the temperature
                temperature *=cooling_rate;
                iterCount++;
                totalIterCnt++;

            }

            warmTrialsCnt++;
            latestImprovementCnt = 1;
            firstImprovementFlag = false;
        } // Warm trials loop

        if (getTourLength(lowestT) > getTourLength(bestT)){
            lowestT = bestT;
        }
        warmTrialsCnt = 0;
        coldTrialsCnt++;
    } // Cold trials loop

    std::cout << "Best tour found: ";
    for (auto &v:lowestT.points) {
        std::cout << v.x << " " << v.y << std::endl;
    }
    std::cout << std::endl;
    std::cout << "Weight: " << getTourLength(lowestT) << std::endl;
    // for (auto v:lowest.vertices) std::cout << v.id << " ";
    // std::cout << std::endl;
    auto t_now = std::chrono::high_resolution_clock::now();
    double timeFromStart = std::chrono::duration<double, std::milli>(t_now - timeStart).count();
    std::cout << "Time: " << timeFromStart << " ms" << std::endl;

    return lowestT;
}


































