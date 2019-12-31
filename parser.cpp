//
// Created by David Woller on 18.9.18.
//

#include "parser.h"
#include "vertex.h"
#include "edge.h"
#include "set.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <utility>

using namespace glns;

std::vector<std::string> Parser::getLines(std::string filename) {
    std::vector<std::string> lines;
    std::string line;
    std::ifstream myfile(filename);

    if (myfile.is_open()) {
        while (getline(myfile, line)) {
            // std::cout << line << '\n';
            lines.push_back(line);
        }
        myfile.close();
    } else std::cout << "Unable to open file: " << filename << std::endl;
    return lines;
}

std::vector<std::string> Parser::lineToWords(std::string line, char delim) {
    std::vector<std::string> words;
    std::stringstream str_strm(line);
    std::string tmp;
    while (std::getline(str_strm, tmp, delim)) {
        words.push_back(tmp);
    }
    return words;
}


int Parser::parse2dGtspInstance(std::string filename, std::vector<Vertex> &vertices,
                                 std::vector<Set> &sets, std::vector<std::vector<float> > &edgeMatrix) {
/*
 * Assuming file format:
 * NAME : ...
 * COMMENT : Rattled grid (Pulleyblank)
 * TYPE : ...
 * DIMENSION : ...
 * GTSP_SETS : ...
 * EDGE_WEIGHT_TYPE : EUC_2D
 * NODE_COORD_SECTION
 * ...
 * EOF
 */
    int noOfVertices = -1;
    int noOfSets = -1;
    // std::vector<Vertex> vertices;
    std::vector<std::vector<int>> setLists;

    std::vector<std::string> lines = getLines(std::move(filename));
    std::vector<std::string> words;
    float x, y;
    int id;
    int shiftSize = 0;
    bool shift = false;
    bool coordsReached = false;
    bool setsReached = false;
    bool eofReached = false;

    for (const auto &line:lines) {
        if (line.find("DIMENSION") != std::string::npos) {
            words = lineToWords(line, ':');
            noOfVertices = std::stoi(words.back());
        }
        if (line.find("GTSP_SETS") != std::string::npos) {
            words = lineToWords(line, ':');
            noOfSets = std::stoi(words.back());
        }
        if (line.find("NODE_COORD_SECTION") != std::string::npos) coordsReached = true;
        if (line.find("GTSP_SET_SECTION") != std::string::npos) setsReached = true;
        if (line.find("EOF") != std::string::npos) eofReached = true;

        // loading vertices from coordinates data
        if (coordsReached && !setsReached && !eofReached && line.find("NODE_COORD_SECTION") == std::string::npos) {
            words = lineToWords(line, ' ');
            y = std::stoi(words.back());
            words.pop_back();
            x = std::stoi(words.back());
            words.pop_back();
            id = std::stoi(words.back());;
            if (!shift) {
                shift = true;
                shiftSize = id;
            }
            id = id - shiftSize;
            Vertex vertex(x, y, id);
            vertices.push_back(vertex);
        }

        // loading list of elements in each set
        if (coordsReached && setsReached && !eofReached && line.find("GTSP_SET_SECTION") == std::string::npos) {
            std::vector<int> setList;
            words = lineToWords(line, ' ');
            for (int i = 1; i < words.size() - 1; i++) {
                setList.push_back(std::stoi(words[i]) - shiftSize);
            }
            setLists.push_back(setList);
        }
    }


    // creating sets according to set lists
    int setId = 0;
    for (auto setList:setLists) {
        Set set;
        for (int curId:setList) {
            for (auto &vertex:vertices) {
                if (vertex.id == curId) {
                    vertex.setId = setId;
                    set.vertices.push_back(vertex);
                }
            }
        }
        set.id = setId++;
        sets.push_back(set);
    }

    // creating matrix of weights
    int maxId = -1;
    int edgeId = 0;
    for (auto vertex:vertices) {
        if (vertex.id > maxId) maxId = vertex.id;
    }
    edgeMatrix = std::vector<std::vector<float> >(maxId + 1, std::vector<float>(
            maxId + 1));
    for (Vertex from:vertices) {
        for (Vertex to:vertices) {
            if (from.id != to.id) {
                Edge edge(from.id, to.id, from.getDistanceFrom(to));
                edgeMatrix[edge.vFromId][edge.vToId] = edge.weight;
            }
        }
    }

    return shiftSize;
}




