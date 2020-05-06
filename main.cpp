#include <iostream>
#include <gtkmm/application.h>
#include <gtkmm/window.h>
#include "planner.h"
#include <iostream>
#include <fstream>


using namespace glns;

void calculateConfidenceForFile(double executions, double iterations, std::string instance, int argc, char *argv[], std::string mode){
    double t = 2.262f;

    std::vector<double> avgWeights;
    std::vector<double> avgTimes;

    for(int j = 0; j < executions; j++){
        double tmpWeight = 0;
        double tmpTime = 0;

        std::ofstream fileWeight;
        std::ofstream fileTime;
        fileWeight.open("/home/h/CLionProjects/GLNSModified/Executions/"+ instance+"_OutputWeight_" +"_execution"+std::to_string(j));
        fileTime.open("/home/h/CLionProjects/GLNSModified/Executions/"+ instance+"_OutputTime_" + "_execution"+std::to_string(j));

        for(int i = 0; i < iterations; i++){

            Planner planner;
            planner.run(nullptr, argc, argv, instance+".txt", mode);
            fileWeight << planner.finalWeight << "\n";
            fileTime << planner.finalTime << "\n";

            tmpWeight += planner.finalWeight;
            tmpTime += planner.finalTime;

        }

        avgWeights.emplace_back(tmpWeight / iterations);
        avgTimes.emplace_back(tmpTime / iterations);

        std::cout<<"average time: " << tmpTime / iterations << std::endl;
        std::cout<<"average weight: " << tmpWeight / iterations << std::endl;


        fileTime.close();
        fileWeight.close();
    }

    std::ofstream file;
    file.open("/home/h/CLionProjects/GLNSModified/Executions/"+instance+"_OutputConfidence");

    double upsilonTime = 0;
    double upsilonWeight = 0;
    for(double d : avgTimes){
        upsilonTime += d;
    }
    for(double d : avgWeights){
        upsilonWeight += d;
    }
    upsilonTime = upsilonTime / executions;
    upsilonWeight = upsilonWeight / executions;
    double tmpSumTime = 0;
    for(double d : avgTimes){
        tmpSumTime += pow( (d-upsilonTime),2 );
    }
    double tmpSumWeight = 0;
    for(double d : avgWeights){
        tmpSumWeight += pow( (d-upsilonWeight),2 );
    }

    double confidenceTime = t*sqrt( (1.0/(executions*(executions-1)))*tmpSumTime);
    double confidenceWeight = t*sqrt( (1.0/(executions*(executions-1)))*tmpSumWeight);


    file << "Time confidence: " << upsilonTime << "+-" << confidenceTime << "\n";
    file << "Weight confidence: " << upsilonWeight << "+-" << confidenceWeight << "\n";

    file.close();

}

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
    std::string ngonsintersect50 = "50_ngons_intersect";
    std::string ngonsintersect100 = "100_ngons_intersect";
    std::string ngonsintersect150 = "150_ngons_intersect";
    std::string ngonsintersect200 = "200_ngons_intersect";
    std::string ngonsintersect400 = "400_ngons_intersect";

    std::string gons3 = "50_3gons";
    std::string gons4 = "50_4gons";
    std::string gons5 = "50_5gons";
    std::string gons6 = "50_6gons";
    std::string gons7 = "50_7gons";
    std::string gons8 = "50_8gons";
    std::string gons9 = "50_9gons";
    std::string gons10 = "50_10gons";
    std::string gons20 = "50_20gons";
    std::string gons30 = "50_30gons";
    std::string gons40 = "50_40gons";
    std::string gons80 = "50_80gons";
    std::string gons160 = "50_160gons";

    std::string density40 = "density_40";
    std::string density80 = "density_80";
    std::string density120 = "density_120";
    std::string density160 = "density_160";
    std::string density200 = "density_200";
    std::string density400 = "density_400";

    std::string potholes2 = "potholes-2";
    std::string potholes3 = "potholes-3";
    std::string potholes4 = "potholes-4";
    std::string potholes6 = "potholes-6";
    std::string potholes8 = "potholes-8";
    std::string potholesInf = "potholes-inf";

    std::vector<std::string> instances;
    //instances.emplace_back(potholes2);
    //instances.emplace_back(potholes3);
    //instances.emplace_back(density120);
    //instances.emplace_back(density160);
    //instances.emplace_back(density200);
    //instances.emplace_back(ngonsintersect200);
    //instances.emplace_back(ngonsintersect200);
    //instances.emplace_back(density40);
    //instances.emplace_back(density400);


    //instances.emplace_back(ngonsintersect400);

    //instances.emplace_back(gons3);
    //instances.emplace_back(gons4);
    //instances.emplace_back(gons5);
    //instances.emplace_back(gons6);
    //instances.emplace_back(gons7);
    //instances.emplace_back(gons8);
    //instances.emplace_back(gons9);
    //instances.emplace_back(gons10);
    //instances.emplace_back(gons20);
    //instances.emplace_back(gons30);
    //instances.emplace_back(gons40);
    //instances.emplace_back(gons80);
    //instances.emplace_back(gons160);

    instances.emplace_back(potholesInf);
    //instances.emplace_back(potholes8);
    //instances.emplace_back(potholes6);
    //instances.emplace_back(density80);
    //instances.emplace_back(ngonsintersect50);
    //instances.emplace_back(potholes4);
    //instances.emplace_back(ngonsintersect100);

    //instances.emplace_back(potholesInf);


    double executions = 5;
    double iterations = 10;

    for(std::string instance : instances){
        calculateConfidenceForFile(executions,iterations,instance,argc,argv, "fast");
    }





    return 0;
}



