#include <GL/freeglut.h>
#include <cmath>
#include <vector>
#include <iostream>
#include <chrono>
#include <ctime>
#include <unordered_map>
#include <fstream>


#include <bso/spatial_design/ms_building.hpp>
#include <bso/spatial_design/cf_building.hpp>


bso::spatial_design::ms_building MS1("Villa");
bso::spatial_design::cf_building CF1(MS1);

void printPrisms2(std::vector<bso::spatial_design::conformal::cf_vertex*> points,
                 std::ofstream& outfile, int design, int zone) {
    outfile << zone << ",";
    for(auto point : points) {
        outfile << point->x() << ",";
        outfile << point->y() << ",";
    }

    double z_base = points[0]->z();
    double z_difference = points[1]->z();

    for(auto point : points) {
        if (point->z() != z_base) {
            z_difference = point->z();
            std::cout << "Different z";
            break;
        }
    }

    outfile << z_base << "," << z_difference << std::endl;
}


void printCF(bso::spatial_design::cf_building CF, int design) {
    std::ofstream outFile("comparison" + std::to_string(design) + ".txt");

    
    std::cout << "CF Model" << std::endl;
    std::cout << "Spaces: " << CF.cfSpaces().size() << std::endl;
    int k = 0;
    for (auto vertex : CF.cfVertices()) {
        std::vector<bso::spatial_design::conformal::cf_vertex*> vertices;
        // std::cout << "Space " << k << std::endl;
        // std::cout << "Vertices: " << space->cfVertices().size() << std::endl;
        vertices.push_back(vertex);
        printPrisms2(vertices, outFile, design, k);
        k++;
    }
}

std::string exec(const char* cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}

int main() {
    std::cout << "Models constructed" << std::endl;
    std::cout << "Zonings made" << std::endl;
    printCF(CF1, 1);
    printCF(CF1, 2);
    sleep(10);

    std::cout << "Post sleep\n";

    std::cout << exec("source ../env/bin/activate && python3 dissimilarity.py");

    std::cout << "Executed python\n";


    return 0;
}