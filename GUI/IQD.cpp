#include <GL/freeglut.h>
#include <cmath>
#include <vector>
#include <iostream>
#include <chrono>
#include <ctime>
#include <unordered_map>
#include <fstream>

#include <bso/spatial_design/ms_building.hpp>



bso::spatial_design::ms_building MS1("Villa");



std::vector<bso::spatial_design::Geometry::Vertex*> boundaryVertices(std::vector<BSO::Spatial_Design::Geometry::Vertex*> vertices) {
    std::unordered_map<BSO::Spatial_Design::Geometry::Vertex*, int> vertexCount;

    // Count each vertex's appearances
    for (auto vertex : vertices) {
        vertexCount[vertex]++;
    }

    std::vector<BSO::Spatial_Design::Geometry::Vertex*> uniqueVertices;
    // Add vertices that appear exactly once to the result
    for (const auto& pair : vertexCount) {
        if (pair.second == 1) {
            uniqueVertices.push_back(pair.first);
        }
    }

    return uniqueVertices;
}

std::pair<BSO::Spatial_Design::Geometry::Vertex*, BSO::Spatial_Design::Geometry::Vertex*> getDiagonal(std::vector<BSO::Spatial_Design::Geometry::Vertex*> points) {
    // Based on the provided 8 points for a 3D figure, we want to get the diagonal in the 2D projection
    // The diagonal is the line that connects the two points that are farthest from each other
    BSO::Spatial_Design::Geometry::Vertex* v1 = points[0];
    BSO::Spatial_Design::Geometry::Vertex* v2 = points[1];
    for(auto point : points) {
        double p2x = v2->get_coords()[0];
        double p2y = v2->get_coords()[1];
        double p1x = v1->get_coords()[0];
        double p1y = v1->get_coords()[1];

        double p3x = point->get_coords()[0];
        double p3y = point->get_coords()[1];

        if ((p1x - p2x) * (p1x - p2x) + (p1y - p2y) * (p1y - p2y) < (p1x - p3x) * (p1x - p3x) + (p1y - p3y) * (p1y - p3y)) {
            v2 = point;
        }
    }
    return std::make_pair(v1, v2);
}

void printPrisms(std::vector<BSO::Spatial_Design::Geometry::Vertex*> points, std::pair<BSO::Spatial_Design::Geometry::Vertex*, BSO::Spatial_Design::Geometry::Vertex*> diag, std::ofstream& outfile, int design, int zone) {
    // There will be 2 prisms, split by the diagonal points
    double z_difference;
    for(auto point : points) {
        z_difference = point->get_coords()[2] - diag.first->get_coords()[2];
        if (z_difference > 0) {
            break;
        } 
    }

    BSO::Spatial_Design::Geometry::Vertex* p3;
    BSO::Spatial_Design::Geometry::Vertex* p4;
    bool p3set = false;
    bool p4set = false;

    for(auto point : points) {
        if (point->get_coords()[0] != diag.first->get_coords()[0] && point->get_coords()[1] != diag.first->get_coords()[1] && 
            point->get_coords()[0] == diag.second->get_coords()[0] && point->get_coords()[1] == diag.second->get_coords()[1] &&
            !p3set) {
            p3 = point;
            continue;
        }
        if (point->get_coords()[0] != diag.first->get_coords()[0] && point->get_coords()[1] != diag.first->get_coords()[1] && 
            point->get_coords()[0] == diag.second->get_coords()[0] && point->get_coords()[1] == diag.second->get_coords()[1] &&
            !p4set) {
            p4 = point;
            continue;
        }
    }
    outfile << zone << "," << diag.first->get_coords()[0] << "," << diag.first->get_coords()[1] << "," <<  
            diag.first->get_coords()[2] << "," << diag.second->get_coords()[0] << "," << diag.second->get_coords()[1] << "," <<  diag.first->get_coords()[2] <<
            "," << p3->get_coords()[0] << "," << p3->get_coords()[1] << "," <<  diag.first->get_coords()[2]
            << "," << diag.first->get_coords()[0] << "," << diag.first->get_coords()[1] << "," <<  
            diag.first->get_coords()[2] + z_difference << "," << diag.second->get_coords()[0] << "," << diag.second->get_coords()[1] << "," <<  diag.first->get_coords()[2] + z_difference <<
            "," << p3->get_coords()[0] << "," << p3->get_coords()[1] << "," <<  diag.first->get_coords()[2] + z_difference << std::endl;

    outfile << zone << "," << diag.first->get_coords()[0] << "," << diag.first->get_coords()[1] << "," <<  
            diag.first->get_coords()[2] << "," << diag.second->get_coords()[0] << "," << diag.second->get_coords()[1] << "," <<  diag.first->get_coords()[2] <<
            "," << p4->get_coords()[0] << "," << p4->get_coords()[1] << "," <<  diag.first->get_coords()[2]
            << "," << diag.first->get_coords()[0] << "," << diag.first->get_coords()[1] << "," <<  
            diag.first->get_coords()[2] + z_difference << "," << diag.second->get_coords()[0] << "," << diag.second->get_coords()[1] << "," <<  diag.first->get_coords()[2] + z_difference <<
            "," << p4->get_coords()[0] << "," << p4->get_coords()[1] << "," <<  diag.first->get_coords()[2] + z_difference << std::endl;
}

void printZonedDesign(bso::spatial_design::ms_building MS, int design) {
    std::ofstream outFile("comparison" + std::to_string(design) + ".txt");

    std::cout << "Zoned Design" << std::endl;
    std::cout << "Zones: " << MS.get_space_ptrs().size() << std::endl;
    int k = 0;
    for (auto space : CF.get_space_ptrs()) {
        std::cout << "Space" << std::endl;
        std::vector<BSO::Spatial_Design::Geometry::Vertex*> vertices;
        for (auto vertex : space->get_coordinates()) {
            vertices.push_back(vertex);
        }
        std::vector<BSO::Spatial_Design::Geometry::Vertex*> bound = boundaryVertices(vertices);
        std::cout << "Zone vertices: " << bound.size() << std::endl;
        std::pair<BSO::Spatial_Design::Geometry::Vertex*, BSO::Spatial_Design::Geometry::Vertex*> diag = getDiagonal(bound);
        printPrisms(bound, diag, outFile, design, k);
        k++;
    }
}

int main() {
    std::cout << "Models constructed" << std::endl;
    std::cout << "Zonings made" << std::endl;
    printZonedDesign(MS, 1);

    return 0;
}