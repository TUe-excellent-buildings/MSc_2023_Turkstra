#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <iomanip>
#include <algorithm>

#include <cstdlib>
#include <stdexcept>

#include <boost/algorithm/string.hpp>

#include <bso/spatial_design/ms_building.hpp>
#include <bso/spatial_design/cf_building.hpp>
#include <bso/structural_design/sd_model.hpp>
#include <bso/grammar/grammar.hpp>
#include <bso/grammar/sd_grammars/design_horizontal.cpp>
#include <bso/visualization/visualization.hpp>

// Function to generate all permutations of {1, 2, 3}
void generate_permutations(std::vector<std::string>& permutations) {
    std::string values = "123";
    do {
        permutations.push_back(values);
    } while (std::next_permutation(values.begin(), values.end()));
}

int main(int argc, char* argv[]) {
    // Define ranges for parameters
    bool penalty = false;
	double penalty_strain = 0;
    double penalty_vol = 0;
    double etaNoise = 0.025;
	int etaConverge = 1;
	bool visualise = false;
    std::vector<std::string> args(argv+1, argv+argc);
	std::string input_file = "villa_v3.txt"; // default value
    std::vector<double> etaShear_values = {0.0, 0.2, 0.4, 0.6, 0.8, 1.0};
    std::vector<double> etaAx_values = {0.0, 0.2, 0.4, 0.6, 0.8, 1.0};
    std::vector<double> etaBend_values = {0.0, 0.2, 0.4, 0.6, 0.8, 1.0};
    std::vector<std::string> checkingOrder_permutations;
    generate_permutations(checkingOrder_permutations);

    // Open CSV file for writing
    std::ofstream csv_file("parameter_study_results.csv");
    if (!csv_file.is_open()) {
        std::cerr << "Error opening CSV file for writing!" << std::endl;
        return 1;
    }

    // Write header row
    csv_file << "EtaShear,EtaAx,EtaBend,EtaConverge,EtaNoise,CheckingOrder,StrainEnergy,StructuralVolume" << std::endl;

    // Iterate over each combination of parameters
    for (double etaShear : etaShear_values) {
        for (double etaAx : etaAx_values) {
            for (double etaBend : etaBend_values) {
                for (const auto& checkingOrder : checkingOrder_permutations) {
                            bso::spatial_design::ms_building MS(input_file);
                            bso::spatial_design::cf_building CF(MS);

                            bso::grammar::grammar gram(CF);

                            bso::structural_design::component::structure trussStructure("truss",{{"A",2250},{"E",3e4}});
                            bso::structural_design::component::structure beamStructure("beam",{{"width",150},{"height",150},{"poisson",0.3},{"E",3e4}});
                            bso::structural_design::component::structure flatShellStructure("flat_shell",{{"thickness",150},{"poisson",0.3},{"E",3e4}});
                            bso::structural_design::component::structure substituteStructure("flat_shell",{{"thickness",150},{"poisson",0.3},{"E",3e-2}});

                            //bso::structural_design::sd_model SD = gram.sd_grammar<bso::grammar::DEFAULT_SD_GRAMMAR>(std::string("settings/sd_settings.txt"));
                            bso::structural_design::sd_model SD = gram.sd_grammar<bso::grammar::DESIGN_HORIZONTAL>(std::string("settings/sd_settings.txt"),etaBend,etaAx,etaShear,etaNoise,etaConverge,checkingOrder,trussStructure,beamStructure,flatShellStructure,substituteStructure);

                            // Calculate strain energy and structural volume
                            double strain_energy, structural_volume;
                            SD.analyze();
                            strain_energy = SD.getTotalResults().mTotalStrainEnergy;
                            structural_volume = SD.getTotalResults().mTotalStructuralVolume;

                            // Write results to CSV file
                            csv_file << std::fixed << std::setprecision(6) << etaShear << ","
                                    << std::fixed << std::setprecision(6) << etaAx << ","
                                    << std::fixed << std::setprecision(6) << etaBend << ","
                                    << etaConverge << ","
                                    << std::fixed << std::setprecision(6) << etaNoise << ","
                                    << checkingOrder << ","
                                    << std::scientific << strain_energy << ","
                                    << std::scientific << structural_volume << std::endl;
                }
            }
        }
    }

    // Close CSV file
    csv_file.close();

    return 0;
}
