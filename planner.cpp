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
#include "canvas.h"
#include "parser.h"

#include "utils.hpp"
#include "MapDrawer.hpp"
#include "triangulation.hpp"
#include "TPP.h"


using namespace glns;

Planner::Planner() {
    generator.seed(static_cast<unsigned long>(std::chrono::system_clock::now().time_since_epoch().count()));
}

///NEW CODE
void writeDataTSPIntoFile(std::string path, int totalPoints, int totalSets, std::vector<pmap::geom::FPoints> &pointsOfPolygons){
    std::ofstream file;
    file.open (path);
    file << "Name : RNG Data\n";
    file << "TYPE : TSP\n";
    file << "DIMENSION : " << totalPoints << "\n";
    file << "GTSP_SETS : " << totalSets << "\n";
    file << "EDGE_WEIGHT_TYPE : EUC_2D\n";
    file << "NODE_COORD_SECTION\n";
    int pointCounter = 1;
    for (int l = 0; l < pointsOfPolygons.size(); ++l) {
        for(auto &point : pointsOfPolygons[l]){
            file << pointCounter++ << " " << point.x << " " << point.y << "\n";
        }
    }
    pointCounter = 1;
    int setCounter = 1;
    file << "GTSP_SET_SECTION\n";
    for (int j = 0; j < pointsOfPolygons.size(); ++j) {
        file << setCounter++ << " ";
        for (int i = 0; i < pointsOfPolygons[j].size(); ++i) {
            file << pointCounter++ << " ";
        }
        file << -1 << "\n";
    }

    file.close();
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

    ///ADDED CODE
    std::ofstream file;
    pmap::geom::FMap map;
    pmap::loadMap(input+"dataPolygons.txt", map);
    //pmap::loadMap("/home/h/CLionProjects/GLNSModified/GeneratedFiles/dataPolygons.txt", map);

    std::vector<pmap::geom::FPoints> pointsCenter;
    std::vector<pmap::geom::FPolygons> triangles;
    std::vector<pmap::geom::FPolygon> polygons;
    int totalNumOfPoints = 0;

    for (int i = 1; i < map.size(); ++i) {
        pmap::geom::FPolygon pol;
        for (int k = map[i].size() - 1; k >= 0; --k) {
            pol.emplace_back(map[i][k]);
        }

        polygons.emplace_back(pol);

        pmap::geom::FPoints tmpPointsCenter;
        pmap::geom::FPolygons tmpTriangles;
        pmap::triangulation::generateTriangularMeshFromPolygon(pol, 20.0, tmpPointsCenter, tmpTriangles);
        totalNumOfPoints+=tmpPointsCenter.size();
        triangles.emplace_back(tmpTriangles);
        pointsCenter.emplace_back(tmpPointsCenter);
    }
    //TODO end of added code



    //std::string filename = argv[1];
    //input = "../GeneratedFiles/dataTSP.txt";
    //input = "/home/h/CLionProjects/GLNSModified/GeneratedFiles/dataTSP.txt";
    writeDataTSPIntoFile(input+"dataTSP.txt", totalNumOfPoints, polygons.size(), pointsCenter);

    TPP tpp(polygons, 0.00001);
    tpp.run();


    pmap::draw::MapDrawer md(map);
    md.openPDF(input+"outputTPP.pdf");
    //md.openPDF("/home/h/CLionProjects/GLNSModified/GeneratedFiles/pic3.pdf");
    md.drawMap();
    for (auto &tmpTriangles : triangles) {
        md.drawPolygons(tmpTriangles, 1.0, PMAP_DRAW_COL_RED);
    }
    for (auto &tmpPoints : pointsCenter) {
        md.drawPoints(tmpPoints, PMAP_DRAW_COL_BLUE, 15.0);
    }
    for (int i = 0; i < tpp.outputPoints.size()-1; i++) {
        pmap::geom::FPoint p1(tpp.outputPoints[i].x, tpp.outputPoints[i].y);
        pmap::geom::FPoint p2(tpp.outputPoints[i+1].x, tpp.outputPoints[i+1].y);
        md.drawPoint(p1, PMAP_DRAW_COL_YELLOW, 10, 1);
        md.drawPoint(p2, PMAP_DRAW_COL_YELLOW, 10, 1);
        md.drawLine(p1, p2, 10, PMAP_DRAW_COL_GREEN, 1.0);
    }
    md.closePDF();
    std::cout << "the end";
    ///ADDED CODE





    // Load GTSP problem instance
    Parser parser;
    shiftSize = parser.parse2dGtspInstance(input+"dataTSP.txt", vertices, sets, edgeMatrix);


    std::cout << "\n"  << "Sets " << sets.size() << " Vertices: " << vertices.size() << std::endl;



    std::cout << "Loaded problem " << input << std::endl;
    if (visualize) {
        canvas->setData(sets, vertices);
        canvas->notify();
    }

    auto timeStart = std::chrono::high_resolution_clock::now();

    Tour tour = solve(canvas, mode, maxTime, tourBudget);

    std::cout<<"tour: ";
    for (Vertex v : tour.vertices) {
        std::cout<< v.x << " "<<v.y;
    }

    auto t_now = std::chrono::high_resolution_clock::now();
    float time = std::chrono::duration<float, std::milli>(t_now - timeStart).count();

    if (outFlag) {
        std::ofstream outputFile;
        std::ifstream f(output);
        if (!f.good()) {
            outputFile.open (output, std::ofstream::app);
            outputFile << "problem" << " mode" << " maxTime(s)" << " tourBudget" << " time(s)" << " m" << " n"<< " weight\n";
            outputFile << input << " "<< mode << " " << maxTime << " " << tourBudget << " " << time/1000 << " " << sets.size() << " " << vertices.size() << " " << getTourWeight(tour) << std::endl;
        } else {
            outputFile.open (output, std::ofstream::app);
            outputFile << input << " "<< mode << " " << maxTime << " " << tourBudget << " " << time/1000 << " " << sets.size() << " " << vertices.size() << " " << getTourWeight(tour) << std::endl;
        }
        outputFile.close();
    }
}



float min(float a, float b) {
    return (a <= b) ? a : b;
}

float max(float a, float b) {
    return (a >= b) ? a : b;
}

void Planner::precomputeSetVertexDistances() {
    int maxVId = -1;
    int maxSetId = -1;
    for (auto &vertex:vertices) {
        if (vertex.id > maxVId) maxVId = vertex.id;
        if (vertex.setId > maxSetId) maxSetId = vertex.setId;
    }
    setVertexDistances = std::vector<std::vector<float> >(maxSetId + 1, std::vector<float>(maxVId + 1));

    for (Vertex &u:vertices) {
        for (Set &set:sets) {
            if (u.setId != set.id) {
                // dist(set, u) = min{min{w(u, v), w(v, u)}}, where v is from set
                float minDistance = FLT_MAX;
                for (Vertex &v:set.vertices) {
                    float dist1 = edgeMatrix[u.id][v.id];
                    float dist2 = edgeMatrix[v.id][u.id];
                    float smaller = min(dist1, dist2);
                    minDistance = min(minDistance, smaller);
                }
                setVertexDistances[set.id][u.id] = minDistance;
            }
        }
    }
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



Tour Planner::initRandomTour() {
    std::vector<Set> tmpSets; // we don't want to shuffle sets stored in Planner
    tmpSets = sets;
    Tour tour;
    // uniformly randomly shuffle set indices from 0 to noOfSets - 1
    std::shuffle(tmpSets.begin(), tmpSets.end(), generator);

    // uniformly randomly select a vertex from each set
    for (auto &set:tmpSets) {
        std::uniform_int_distribution<int> dist(0, set.vertices.size() - 1);
        int randIndex = dist(generator);
        tour.vertices.push_back(set.vertices[randIndex]);
    }

    // get corresponding edges from edgeMatrix
    for (int i = 0; i < tour.vertices.size(); i++) {
        int vFromId = tour.vertices[i].id;
        int vToId = tour.vertices[(i + 1) % tour.vertices.size()].id;
        Edge e(vFromId, vToId, edgeMatrix[vFromId][vToId]);
        tour.edges.push_back(e);
    }

    return tour;
}

/*
 * Chooses a starting vertex v randomly.
 * Remaining vertices are added using the unified insertion heuristic with lambda = 1 and my = 0.75
 */
Tour Planner::initRandomInsertionTour() {
    Tour tour;
    Vertex v1 = getRandomVertex(vertices);
    tour.vertices.push_back(v1);
    Vertex v2 = getRandomVertex(vertices);
    while (v1.setId == v2.setId) {
        v2 = getRandomVertex(vertices);
    }
    tour.vertices.push_back(v2);
    tour.edges.emplace_back(v1.id, v2.id, edgeMatrix[v1.id][v2.id]);
    tour.edges.emplace_back(v2.id, v1.id, edgeMatrix[v2.id][v1.id]);

    float lambda = 1;
    float my = 0.75;

    while(tour.vertices.size() < sets.size()) {
        tour = unifiedInsertion(tour, lambda, my);
    }

    return tour;
}

/*
 * Returns closed partial tour of given length.
 * If length given is too large, returns tour of random length instead.
 * Randomly generated partial tours have length from 1 to sets.size()
 */
Tour Planner::initPartialTour(int length) {
    Tour partialTour;
    std::vector<Set> tmpSets = sets; // we don't want to shuffle sets stored in Planner
    // uniformly randomly shuffle set indices from 0 to noOfSets - 1
    std::shuffle(tmpSets.begin(), tmpSets.end(), generator);

    // get a random index from 1 to number of sets - 1
    if (length > tmpSets.size()) {
        std::cout << "Partial tour length given too large (>" << tmpSets.size() << "). Returning randomly long tour."
                  << std::endl;
        std::uniform_int_distribution<int> dist(1, tmpSets.size());
        length = dist(generator);
    }

    // get a subset of tmpSets
    std::vector<Set> tmpSetsSubset(tmpSets.begin(), tmpSets.begin() + length);
    // uniformly randomly select a vertex from each set
    for (auto &set:tmpSetsSubset) {
        std::uniform_int_distribution<int> dist(0, set.vertices.size() - 1);
        int randIndex = dist(generator);
        partialTour.vertices.push_back(set.vertices[randIndex]);
    }
    // get corresponding edges from edgeMatrix
    if (partialTour.vertices.size() > 1) {
        for (int i = 0; i < partialTour.vertices.size(); i++) {
            int vFromId = partialTour.vertices[i].id;
            int vToId = partialTour.vertices[(i + 1) % partialTour.vertices.size()].id;
            partialTour.edges.emplace_back(vFromId, vToId, edgeMatrix[vFromId][vToId]);
        }
    }
    return partialTour;
}



Vertex Planner::getRandomVertex(std::vector<Vertex> &vertices) {
    std::uniform_int_distribution<int> dist(0, vertices.size() - 1);
    return vertices[dist(generator)];
}

/*
 * Removes vertex v_i and edges (v_i-1, v_i), (v_i, v_i+1)
 * Adds edge (v_i-1.from, v_i+1.to)
 */
Tour Planner::removeVertexFromTour(Tour tour, Vertex vertex) {
    std::vector<Vertex>::iterator vertexIt;
    vertexIt = std::find_if(tour.vertices.begin(), tour.vertices.end(), [&vertex](Vertex const &v) {
        return v.id == vertex.id;
    });
    tour.vertices.erase(vertexIt);
    // remove edges from and to erased vertex
    std::vector<Edge>::iterator edgeIt;
    edgeIt = std::find_if(tour.edges.begin(), tour.edges.end(), [&vertex](Edge const &e) {
        return e.vFromId == vertex.id;
    });
    int toId = edgeIt->vToId;
    tour.edges.erase(edgeIt);
    edgeIt = std::find_if(tour.edges.begin(), tour.edges.end(), [&vertex](Edge const &e) {
        return e.vToId == vertex.id;
    });
    int fromId = edgeIt->vFromId;
    tour.edges.erase(edgeIt);
    // insert edge connecting disconnected vertices
    tour.edges.emplace_back(fromId, toId, edgeMatrix[fromId][toId]);
    return tour;
}

float Planner::getTourWeight(Tour &tour) {
    float weight = 0;
    for (auto &e:tour.edges) {
        weight += e.weight;
    }
    return weight;
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
    std::vector<bool> isUsed(sets.size(), false);
    for (auto &v:partialTour.vertices) {
        isUsed[v.setId] = true;
    }
    for (int i = 0; i < sets.size(); i++) {
        if (!isUsed[i]) {
            // for each unused set V_i, define the minimum distance d_i = min dist(V_i, u), where u is from V_T (vertices in partial tour)
            auto minDist = FLT_MAX;
            for (Vertex &v:partialTour.vertices) {
                float dist = setVertexDistances[i][v.id];
                if (dist < minDist) minDist = dist;
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

int Planner::cheapestSetSelection(const Tour& partialTour) {
    // get sets, that are not in partialTour (P_V \ P_T)
    std::vector<bool> isUsed(sets.size(), false);
    for (auto &v:partialTour.vertices) {
        isUsed[v.setId] = true;
    }

    float minCost = FLT_MAX;
    int minSetId;

    for (int i = 0; i < sets.size(); i++) {
        if (!isUsed[i]) {
            // pick the set V_i that contains the vertex v that minimizes the insertion cost w(v) + w(x,v) + w(v,y) - w(x,y)
            float minInsCost = FLT_MAX;
            for (auto &e:partialTour.edges) {
                float lower_bound = setVertexDistances[sets[i].id][e.vFromId] +
                                    setVertexDistances[sets[i].id][e.vToId] - e.weight;

                if (lower_bound < minCost) {
                    for (auto &v:sets[i].vertices) {
                        float cost = edgeMatrix[e.vFromId][v.id] + edgeMatrix[v.id][e.vToId] - e.weight;
                        if (cost < minInsCost) minInsCost = cost;
                    }
                    if (minInsCost < minCost) {
                        minCost = minInsCost;
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
    if (partialTour.vertices.size() < 2 || partialTour.edges.empty()) {
        std::cout << "Unified insertion: given tour too short. Returning unchanged tour." << std::endl;
    } else if (partialTour.vertices.size() >= sets.size()) {
        std::cout << "Unified insertion: given tour too long. Returning unchanged tour." << std::endl;
    } else {
        // Pick a set V_i in P_V \ P_T
        Set *V_i;
        int i;
        if (lambda == -1) {
            i = cheapestSetSelection(partialTour);
            V_i = &sets[i];
        } else {
            i = unifiedSetSelection(partialTour, lambda);
            V_i = &sets[i];
        }
        // Find an edge (x, y) from E_T and vertex v from V_i that minimizes (1 + rand)(w(x,v) + w(v,y) - w(x,y))
        // rand = uniform random number from [0, my]
        float minWeight = FLT_MAX;
        Edge minEdge;
        Vertex minVertex;

        std::uniform_real_distribution<float> distribution(0, my);

        for (auto &edge:partialTour.edges) {
            int x_id = edge.vFromId;
            int y_id = edge.vToId;

            float lowerBound = setVertexDistances[(*V_i).id][x_id] +
                               setVertexDistances[(*V_i).id][y_id] - edge.weight;

            if (lowerBound < minWeight) {
                for (auto &v:(*V_i).vertices) {
                    float rand = distribution(generator);
                    float weight =
                            (1 + rand) * (edgeMatrix[x_id][v.id] + edgeMatrix[v.id][y_id] - edge.weight);

                    if (weight < minWeight) {
                        minWeight = weight;
                        minEdge = edge;
                        minVertex = v;
                    }
                }
            }
        }

        // Delete the edge (x,y) from E_T
        removeEdge(minEdge, partialTour);

        // add the edges (x,v), (v,y) to E_T
        partialTour.edges.emplace_back(minEdge.vFromId, minVertex.id, edgeMatrix[minEdge.vFromId][minVertex.id]);
        partialTour.edges.emplace_back(minVertex.id, minEdge.vToId, edgeMatrix[minVertex.id][minEdge.vToId]);
        // Add v to V_T
        partialTour.vertices.push_back(minVertex);
        // return T
    }
    return partialTour;
}

void Planner::removeEdge(Edge &edge, Tour &tour) {
    auto it = std::find_if(tour.edges.begin(), tour.edges.end(), [&edge](Edge const &e) {
        return (e.vFromId == edge.vFromId && e.vToId == edge.vToId);
    });
    tour.edges.erase(it);

}

/*
 * Removes a continuous segment of the tour of length N_r.
 */
Tour Planner::segmentRemoval(Tour tour, int N_r) {
    int length = tour.vertices.size();
    if (length < 3) {
        std::cout << "segmentRemoval: given tour is too short (<3 vertices). Returning unchanged tour." << std::endl;
    } else if (N_r > (length - 2)) {
        std::cout << "segmentRemoval: N_r given is too large. Returning unchanged tour." << std::endl;
    } else {
        // Uniformly randomly select a vertex
        std::uniform_int_distribution<int> dist(0, tour.vertices.size() - 1);
        int randIndex = dist(generator);
        Vertex firstVertex = tour.vertices[randIndex];
        // remove currentEdge and nextVertex N_r times
        std::vector<Edge>::iterator edgeIt;
        std::vector<Vertex>::iterator vertexIt;
        Vertex previousVertex = firstVertex;
        Vertex nextVertex;
        Edge currentEdge;
        for (int i = 0; i < N_r; i++) {
            // find edge from previousVertex
            edgeIt = std::find_if(tour.edges.begin(), tour.edges.end(), [&previousVertex](Edge const &e) {
                return e.vFromId == previousVertex.id;
            });
            currentEdge = *edgeIt;
            // find nextVertex
            nextVertex = vertices[currentEdge.vToId];
            vertexIt = std::find_if(tour.vertices.begin(), tour.vertices.end(), [&nextVertex](Vertex const &v) {
                return v.id == nextVertex.id;
            });
            // erase both
            tour.edges.erase(edgeIt);
            tour.vertices.erase(vertexIt);
            previousVertex = nextVertex;
        }
        // erase one remaining edge from the segment
        edgeIt = std::find_if(tour.edges.begin(), tour.edges.end(), [&previousVertex](Edge const &e) {
            return e.vFromId == previousVertex.id;
        });
        nextVertex = vertices[edgeIt->vToId];
        tour.edges.erase(edgeIt);
        // make tour closed again
        tour.edges.emplace_back(firstVertex.id, nextVertex.id, edgeMatrix[firstVertex.id][nextVertex.id]);
    }

    return tour;
}

Tour Planner::distanceRemoval(Tour tour, int N_r, float lambda) {
    int length = tour.vertices.size();
    if (length < 3) {
        std::cout << "distanceRemoval: given tour is too short (<3 vertices). Returning unchanged tour." << std::endl;
    } else if (N_r > (length - 2)) {
        std::cout << "distanceRemoval: N_r given is too large. Returning unchanged tour." << std::endl;
    } else {
        std::vector<Vertex> V_removed;
        // Randomly remove a vertex from T, add it to V_removed
        Vertex vertex = getRandomVertex(tour.vertices);
        tour = removeVertexFromTour(tour, vertex);
        V_removed.push_back(vertex);
        // Perform remaining N_r - 1 removals
        for (int i = 1; i < N_r; i++) {
            // Uniformly randomly select v_seed from V_removed
            Vertex v_seed = getRandomVertex(V_removed);
            // for each v_j from V_T, compute r_j as r_j = min{w(v_seed, v_j), w(v_j, v_seed)}
            for (auto &v:tour.vertices) {
                v.removalCost = min(edgeMatrix[v.id][v_seed.id], edgeMatrix[v_seed.id][v.id]);
            }
            tour = removalFramework(tour, lambda);
        }
    }
    return tour;
}

Tour Planner::worstRemoval(Tour tour, int N_r, float lambda) {
    int length = tour.vertices.size();
    if (length < 3) {
        std::cout << "worstRemoval: given tour is too short (<3 vertices). Returning unchanged tour." << std::endl;
    } else if (N_r > (length - 2)) {
        std::cout << "worstRemoval: N_r given is too large. Returning unchanged tour." << std::endl;
    } else {
        // Calculate removal cost for all vertices
        std::vector<Edge>::iterator edgeIt;
        float prevId, nextId; // vertices ids
        for (auto &v:tour.vertices) {
            v.removalCost = 0;
            // Find edge from v
            edgeIt = std::find_if(tour.edges.begin(), tour.edges.end(), [&v](Edge const &e) {
                return e.vFromId == v.id;
            });
            nextId = edgeIt->vToId;
            v.removalCost += edgeIt->weight;
            // find edge to v
            edgeIt = std::find_if(tour.edges.begin(), tour.edges.end(), [&v](Edge const &e) {
                return e.vToId == v.id;
            });
            prevId = edgeIt->vFromId;
            v.removalCost += edgeIt->weight;
            v.removalCost -= edgeMatrix[prevId][nextId];
        }
        for (int i = 0; i < N_r; i++) {
            tour = removalFramework(tour, lambda);
        }
    }
    return tour;
}

bool compareVerticesRemovalCost(Vertex v1, Vertex v2) {
    return (v1.removalCost < v2.removalCost);
}

Tour Planner::removalFramework(Tour tour, float lambda) {
    // Randomly select k = {0...l - 1} according to the unnormalized probability mass function {lambda^0, lambda^1, ... lambda^(l-1)}
    // Initialize weights
    int l = tour.vertices.size();
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
    std::sort(tour.vertices.begin(), tour.vertices.end(), compareVerticesRemovalCost);
    // Pick the vertex v_j from V_T  with the kth smallest value r_j
    Vertex v_j = tour.vertices[k];
    // Remove v_j from tour, remove corresponding edges from and to v_j, add edge between disconnected vertices
    tour = removeVertexFromTour(tour, v_j);
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
    unsigned long N_max = sets.size() - 2;
    int N_r = getRandomNumber(1, N_max);

    // Create a copy of current tour
    Tour T_new = current;

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
    while (T_new.vertices.size() < sets.size()) {
        T_new = unifiedInsertion(T_new, I->lambda, I->my);
    }

    // Update scores for insertion and removal heuristics
    float score = 100 * max(getTourWeight(current) - getTourWeight(T_new), 0)/getTourWeight(current);
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

void Planner::updateHeuristicsWeights(float epsilon) {
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

void Planner::printWeights() {
    std::cout << std::endl << "Insertion heuristics" << std::endl;
    for (auto h:insertionHeuristics) {
        std::cout << h.name << " lambda:" << h.lambda << " my:" << h.my << " " << std::endl << "    ";
        for (auto w:h.weights) {
            std::cout << w.first << ": " << w.second << "   ";
        }
        std::cout << std::endl;
    }

    std::cout << std::endl << "Removal heuristics" << std::endl;
    for (auto h:removalHeuristics) {
        std::cout << h.name << " lambda:" << h.lambda << " my:" << h.my << " " << std::endl << "    ";
        for (auto w:h.weights) {
            std::cout << w.first << ": " << w.second << "   ";
        }
        std::cout << std::endl;
    }

}

bool Planner::acceptTrial(float trialCost, float currentCost, float temperature) {
    float prob1 = exp((currentCost - trialCost)/temperature);
    float prob2 = 1;
    float prob = min(prob1, prob2);

    std::vector<float> weights;
    weights.emplace_back(1-prob); // prob of not accepting at position 0
    weights.emplace_back(prob); // prob of accepting at position 1

    std::discrete_distribution<int> distribution(weights.begin(), weights.end());
    int index = distribution(generator);

    return (bool)index;
}

bool Planner::acceptTrialNoParam(float trialCost, float currentCost, float probAccept) {
    if (trialCost < currentCost) return true;

    std::vector<float> weights;
    weights.emplace_back(1-probAccept); // prob of not accepting at position 0
    weights.emplace_back(probAccept); // prob of accepting at position 1

    std::discrete_distribution<int> distribution(weights.begin(), weights.end());
    int index = distribution(generator);

    return (bool)index;
}


/*
 * Optimizes the tour given, while keeping the set ordering fixed.
 * Optimization is achieved by performing BFS search.
 */
Tour Planner::reOpt(const Tour& tour) {
    // find smallest set, to start with
    int minSetId = 0;
    unsigned long minSetSize = LONG_MAX;
    for (const auto& set:sets) {
        if (set.vertices.size() < minSetSize) {
            minSetSize = set.vertices.size();
            minSetId = set.id;
        }
    }

    // reconstruct sets order from tour.edges
    int firstSetId = minSetId;
    int currentSetId = firstSetId;
    std::vector<int> setsOrdering;
    for (int i = 0; i < sets.size(); i++) {
        for (auto &e:tour.edges) {
            if (vertices[e.vFromId].setId == currentSetId) {
                setsOrdering.push_back(currentSetId);
                currentSetId = vertices[e.vToId].setId;
                break;
            }
        }
    }

    for (auto &v:sets[firstSetId].vertices) v.BFSWeight = 0;

    Tour bestTour = tour;
    for (auto &vStart:sets[firstSetId].vertices) {
        int nextSetId;
        for (int i = 0; i < sets.size() - 1; i++) {
            if (i == 0) { // fill BFSWeights from start vertex to vertices in first set
                nextSetId = setsOrdering[i+1];
                for (auto &v:sets[nextSetId].vertices) {
                    v.BFSWeight = edgeMatrix[vStart.id][v.id];
                    v.BFSPrevId = vStart.id;
                }
            } else { // fill BFSWeights from all vertices in i-th set to all vertices in i+1-th set
                currentSetId = setsOrdering[i];
                nextSetId = setsOrdering[i+1];
                for (auto &v:sets[nextSetId].vertices) v.BFSWeight = DBL_MAX;
                for (auto &vFrom:sets[currentSetId].vertices) {
                    for (auto &vTo:sets[nextSetId].vertices) {
                        float newWeight = vFrom.BFSWeight + edgeMatrix[vFrom.id][vTo.id];
                        if (newWeight < vTo.BFSWeight) {
                            vTo.BFSWeight = newWeight;
                            vTo.BFSPrevId = vFrom.id;
                        }
                    }
                }
            }
        }
        // Add weight of edge from vertices in last set to first element
        for (auto &v:sets[setsOrdering[sets.size()-1]].vertices) {
            v.BFSWeight += edgeMatrix[v.id][vStart.id];
        }
        // Reconstruct tour from last set to first
        Tour newTour;
        float minWeight = FLT_MAX;
        Vertex minVertex;
        for (auto &v:sets[setsOrdering[sets.size()-1]].vertices) {
            if(v.BFSWeight < minWeight) {
                minWeight = v.BFSWeight;
                minVertex = v;
            }
        }
        newTour.vertices.insert(newTour.vertices.begin(), minVertex);
        Vertex nextVertex = minVertex;
        for (int i = sets.size() - 2; i >= 0; i--) {
            currentSetId = setsOrdering[i];
            Vertex currentVertex;
            for (auto &v:sets[currentSetId].vertices) {
                if (v.id == nextVertex.BFSPrevId) {
                    currentVertex = v;
                }
            }
            newTour.vertices.insert(newTour.vertices.begin(), currentVertex);
            nextVertex = currentVertex;
        }
        // add edges to newTour
        for (int i = 0; i < newTour.vertices.size(); i++) {
            int vFromId = newTour.vertices[i].id;
            int vToId = newTour.vertices[(i + 1) % newTour.vertices.size()].id;
            newTour.edges.emplace_back(vFromId, vToId, edgeMatrix[vFromId][vToId]);
        }

        if (getTourWeight(newTour) < getTourWeight(bestTour)) bestTour = newTour;
    }

    return bestTour;
}

/*
 * Optimizes the tour given by randomly changing set ordering.
 */
Tour Planner::moveOpt(Tour tour, int NMove) {
    Tour bestTour = tour;
    for (int i = 0; i < NMove; i++) {
        // Randomly select vertex v in tour
        Vertex randV = getRandomVertex(tour.vertices);
        // remove v, remove edges from and to v, add edge (from, to)
        tour = removeVertexFromTour(tour, randV);

        int setId = randV.setId;
        float minWeight = DBL_MAX;
        Edge eToRemove;
        Vertex uMin;
        for (auto &u:sets[setId].vertices) {
            for (auto &e:tour.edges) {
                float weight = edgeMatrix[e.vFromId][u.id] + edgeMatrix[u.id][e.vToId] - e.weight;
                if (weight < minWeight) {
                    minWeight = weight;
                    eToRemove = e;
                    uMin = u;
                }
            }
        }
        // Remove eToRemove
        std::vector<Edge>::iterator edgeIt;
        edgeIt = std::find_if(tour.edges.begin(), tour.edges.end(), [&eToRemove](Edge const &e) {
            return (e.vFromId == eToRemove.vFromId) && (e.vToId == eToRemove.vToId);
        });
        tour.edges.erase(edgeIt);
        // Insert uMin and appropriate edges
        tour.vertices.push_back(uMin);
        tour.edges.emplace_back(eToRemove.vFromId,uMin.id, edgeMatrix[eToRemove.vFromId][uMin.id]);
        tour.edges.emplace_back(uMin.id, eToRemove.vToId, edgeMatrix[uMin.id][eToRemove.vToId]);

        if (getTourWeight(tour) < getTourWeight(bestTour)) bestTour = tour;
    }

    return bestTour;
}

/*
 * Repeatedly performs moveOpt and reOpt, until there is no improvement.
 */
Tour Planner::optCycle(Tour tour, int NMove, std::string mode) {
    float previousWeight = getTourWeight(tour);
    float newWeight = 0;
    while (newWeight < previousWeight) {
        previousWeight = getTourWeight(tour);
        if (mode != "fast") {
            tour = reOpt(tour);
        }        tour = moveOpt(tour, NMove);
        newWeight = getTourWeight(tour);
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

    // Common parameters
    auto numSets = (int) sets.size();
    float acceptPercentage = 0.05;
    float epsilon = 0.5;
    __useconds_t uDelay = 0; // delay after finding a better tour in warm trial; in useconds
    // Mode-specific parameters
    int coldTrials;
    int warmTrials;
    int numIterations;
    // if best tour wasn't improved for latestImprovement consecutive iterations, leave initial descent (mid phase)
    int latestImprovement;
    // if best tour wasn't improved for firstImprovement consecutive iterations and there was not an initial improvement, leave warm restart (late phase)
    int firstImprovement;
    float probAccept;
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

    // Precompute all necessary stuff
    precomputeSetVertexDistances();
    
    // Cold trials loop
    while (coldTrialsCnt <= coldTrials) {
        // build tour from scratch on a cold restart
        Tour bestT = initRandomInsertionTour(); // best tour in this trial
        if (lowestT.vertices.empty()) {
            lowestT = bestT;
        } else {
            if (getTourWeight(lowestT) > getTourWeight(bestT)) lowestT = bestT;
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
            float temperature = 1.442 * acceptPercentage * getTourWeight(bestT);
            float cooling_rate = pow(((0.0005 * getTourWeight(lowestT))/(acceptPercentage * getTourWeight(currentT))), 1.0/numIterations);

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
                if (acceptTrialNoParam(getTourWeight(trial), getTourWeight(currentT), probAccept) || acceptTrial(getTourWeight(trial), getTourWeight(currentT), temperature)) {
                    if (mode == "slow") trial = optCycle(trial, NMove, mode);
                    currentT = trial;
                }



                if(getTourWeight(currentT) < getTourWeight(bestT)) {
                    latestImprovementCnt = 1;
                    firstImprovementFlag = true;
                    if ((coldTrialsCnt > 1) && (warmTrialsCnt > 1)) {
                        warmTrialsCnt++;
                    }
                    currentT = optCycle(currentT, NMove, mode); // Locally reoptimize current tour

                    bestT = currentT;
                    if (visualize) {
                        canvas->setTour(bestT);
                        canvas->notify();
                        usleep(uDelay);
                    }
                } else {
                    latestImprovementCnt++;
                }

                // time limit and budget limit check
                auto t_now = std::chrono::high_resolution_clock::now();
                float timeFromStart = std::chrono::duration<float, std::milli>(t_now - timeStart).count();
                if ((timeFromStart/1000 > maxTime) || (getTourWeight(bestT) < tourBudget)) {
                    if (timeFromStart/1000 > maxTime) std::cout << "Max time exceeded" << std::endl;
                    if (getTourWeight(bestT) < tourBudget) std::cout << "Tour better than budget found" << std::endl;
                    if (getTourWeight(lowestT) > getTourWeight(bestT)) lowestT = bestT;
                    std::cout << "lowest weight: " << getTourWeight(lowestT) << std::endl;
                    if (visualize) {
                        canvas->setBestTour(lowestT);
                        Tour emptyTour;
                        canvas->setTour(emptyTour);
                        canvas->notify();
                    }
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

        if (getTourWeight(lowestT) > getTourWeight(bestT)){
            lowestT = bestT;
            if (visualize) {
                canvas->setBestTour(lowestT);
                canvas->notify();
            }
        }
        warmTrialsCnt = 0;
        coldTrialsCnt++;
    } // Cold trials loop

    std::cout << "Best tour found: ";
    for (auto &v:lowestT.vertices) {
        v.id += shiftSize;
        std::cout << v.id << " ";
    }
    std::cout << std::endl;
    std::cout << "Weight: " << getTourWeight(lowestT) << std::endl;
    Tour emptyTour;
    if (visualize) {
        canvas->setTour(emptyTour);
        canvas->notify();
    }
    // for (auto v:lowest.vertices) std::cout << v.id << " ";
    // std::cout << std::endl;
    auto t_now = std::chrono::high_resolution_clock::now();
    float timeFromStart = std::chrono::duration<float, std::milli>(t_now - timeStart).count();
    std::cout << "Time: " << timeFromStart << " ms" << std::endl;

    return lowestT;
}


































