#include <iostream>

#include <bso/spatial_design/ms_building.hpp>
#include <bso/spatial_design/cf_building.hpp>
#include <bso/utilities/geometry.hpp>
#include <bso/utilities/data_point.hpp>
#include <bso/structural_design/sd_model.hpp>
#include <bso/building_physics/bp_model.hpp>
#include <bso/grammar/grammar.hpp>
#include <bso/visualization/visualization.hpp>
#include <bso/grammar/sd_grammars/design_response_grammar.cpp>
#include <boost/algorithm/string.hpp>

#include <cstdlib>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <map>
#include <algorithm>

std::vector<std::string> VIS_OPTIONS = {"ms","sc","rc","cb","sd","fe","bp"};
std::vector<std::string> OUT_OPTIONS = {"result_line", "verbose", "ms_files","best","all"};

int end, begin = 0;
template<class T>
void out(const T& t, const bool& e = false,
				 const bool& i = false, const bool& verbose = false){
	if (!verbose) return;
	std::cout << t;
	end = clock();
	if (i) std::cout << " (" << 1000*(end-begin)/CLOCKS_PER_SEC << " ms)";
	if (e) std::cout << std::endl;
	begin = end;
} // out()

using namespace bso;

int main(int argc, char* argv[])
{
	std::string inputFile = "v.txt";
	std::string designName = "";
	std::string inputLine = "";
	std::vector<std::string> visualizations;
	bool result_line = false;
	bool ms_files = false;
	bool verbose = false;
	bool outputAll = false;
	bool outputBest = false;
	double etaBend = 0.2;
	double etaAx = 0.2;
	double etaShear = 0.2;
	double etaNoise = 0.025;
	int etaConverge = 1;
	int nSpacesDelete = 2;
	std::string checkingOrder = "123";
	int iterations = 5;

	std::vector<std::string> args(argv+1, argv+argc);
	auto arg = args.begin();
	if (arg == args.end())
	{
		std::stringstream errorMessage;
		errorMessage << "\nError, expected arguments, received nothing.\n"
								 << "use -h or --help for help." << std::endl;
		throw std::invalid_argument(errorMessage.str());
	}
	while (arg != args.end())
	{
		if (*arg == "-v" || *arg == "--visualize")
		{
			while (++arg != args.end() && arg->operator[](0) != '-')
			{
				std::string visSpec = *arg;
				boost::trim(visSpec);
				if (std::find(VIS_OPTIONS.begin(),VIS_OPTIONS.end(),visSpec) == VIS_OPTIONS.end())
				{
					std::stringstream errorMessage;
					errorMessage << "\nError, did not recognize: \"" << visSpec << "\"\n"
											 << "as an option for -v or --visualization as a specification\n"
											 << "of a model to visualize, use -h or --help for help\n";
					throw std::invalid_argument(errorMessage.str());
				}
				visualizations.push_back(visSpec);
			}
			if (visualizations.size() == 0)
			{
				std::stringstream errorMessage;
				errorMessage << "\nError, expected at least one string after\n"
										 << "-v or --visualize, received nothing. Use -h or\n"
										 << "--help for help\n";
				throw std::invalid_argument(errorMessage.str());
			}
		}
		else
		{
			std::stringstream errorMessage;
			errorMessage << "\nError, did not recognize argument: " << *arg << std::endl;
			throw std::invalid_argument(errorMessage.str());
		}
	}
	// if (!visualizations.empty()) visualization::initVisualization(argc,argv);
	// out("Initialized visualization", true, true, verbose);

	std::vector<spatial_design::ms_building> msDesigns;
	try
	{
		if (!inputFile.empty()) msDesigns.push_back(spatial_design::ms_building(inputFile));
	}
	catch (std::exception& e)
	{
		std::stringstream errorMessage;
		errorMessage << "Error, could not initialize MS building design using\n";
		if (!inputFile.empty())
			errorMessage << "the following MS input file: " << inputFile << "\n";
		else
			errorMessage << "an SC model initialized by the following line:\n" << inputLine;
		errorMessage << "Received the following error:\n" << e.what() << std::endl;

		throw std::invalid_argument(errorMessage.str());
	}
	out("Initialized initial MS building model", true, true, verbose);

	unsigned int nSpaces = msDesigns.back().getSpacePtrs().size();
	double floorArea = msDesigns.back().getFloorArea();
	std::vector<spatial_design::sc_building> scDesigns;
	std::vector<structural_design::sd_model> sdModels;
	std::vector<building_physics::bp_model> bpModels;


	bso::structural_design::component::structure trussStructure("truss",{{"A",2250},{"E",3e4}});
	bso::structural_design::component::structure beamStructure("beam",{{"width",150},{"height",150},{"poisson",0.3},{"E",3e4}});
	bso::structural_design::component::structure flatShellStructure("flat_shell",{{"thickness",150},{"poisson",0.3},{"E",3e4}});
	bso::structural_design::component::structure substituteStructure("flat_shell",{{"thickness",150},{"poisson",0.3},{"E",3e-2}});

	// LOOP
	for (unsigned int i = 0; i <= iterations; ++i)
	{
		out("Iteration: ",false,false,verbose); out(i,true,false,verbose);
		// if (std::find(visualizations.begin(),visualizations.end(),"sc") != visualizations.end() ||
		// 		result_line)
		// {
		// 	scDesigns.push_back(spatial_design::sc_building(msDesigns.back()));
		// 	out("initialized sc building",true,true,verbose);
		// }

		// step 1, generate SD and BP models and evaluate them.
		spatial_design::cf_building cf(msDesigns.back(),1e-6);
		out("Initialized conformal model",true,true,verbose);

		grammar::grammar grm(cf);
		out("Initialized grammar",true,true,verbose);		

		sdModels.push_back(grm.sd_grammar<grammar::DESIGN_RESPONSE>(std::string("settings/sd_settings.txt"),etaBend,etaAx,etaShear,etaNoise,etaConverge,checkingOrder,trussStructure,beamStructure,flatShellStructure,substituteStructure));
		out("Generated structural model",true,true,verbose);

		sdModels.back().analyze();
		out("Evaluated structural model: ",false,false,verbose);
		double sdResult = sdModels.back().getTotalResults().mTotalStrainEnergy;
		out(sdResult,true,true,verbose);

		out("Sent models to visualization",true,true,verbose);

		// create a new MS model
		spatial_design::ms_building newMS = msDesigns.back();

        // Get performances per space and store in a vector
        std::vector<std::pair<double, std::shared_ptr<spatial_design::ms_space>>> spacePerformances;

        for (const auto& j : newMS.getSpacePtrs()) {
            auto spaceGeom = j->getGeometry();
            auto id = j->getID();
            double performance = sdModels.back().getPartialResults(&spaceGeom).mTotalStrainEnergy / j->getFloorArea();
            spacePerformances.emplace_back(performance, j);
        }

        // Sort the spacePerformances vector based on the performance values in descending order
        std::sort(spacePerformances.rbegin(), spacePerformances.rend());

        // Output the ranked performances
        std::cout << "Ranked Performances:\n";
        for (const auto& performance : spacePerformances) {
            auto id = performance.second->getID();
            std::cout << "ID: " << id << ", Performance: " << performance.first << std::endl;
        }
		
        // Delete the nToDelete worst-performing spaces
        for (unsigned int i = 0; i < nSpacesDelete && i < spacePerformances.size(); ++i) {
            auto spaceToDelete = spacePerformances[i].second;
            newMS.deleteSpace(*spaceToDelete);
        }
		std::cout << newMS.getSpacePtrs().size() << std::endl;

        // newMS.setZZero();

        // std::vector<spatial_design::ms_space*> floatingSpaces;
        // if (newMS.hasFloatingSpaces(floatingSpaces)) 
		// {
        //     for (auto& i : floatingSpaces) newMS.deleteSpace(i);
        // }
		// out("Removed worst performing spaces",true,true,verbose);
		msDesigns.push_back(newMS);
		
        // Step to scale and split spaces to recover initial conditions
        // Scale the dimensions in x and y direction
        double scaleFactor = sqrt(floorArea / newMS.getFloorArea());
        newMS.scale({{0,scaleFactor},{1,scaleFactor}});
		newMS.snapOn({{0,1},{1,1}});

        // newMS.setZZero();
        out("Scaled design to initial volume", true, true, verbose);

        msDesigns.push_back(newMS);
        out("Modified building spatial design into a new one", true, true, verbose);
	}
    out("finished SCDP process",true,true,verbose);

	if (!visualizations.empty()) visualization::initVisualization(argc,argv);
	for(int i = 0; i < msDesigns.size(); i++)
	{
		visualization::visualize(msDesigns[i],"","ms_building",4.0);
	}
	visualization::endVisualization();

	return 0;
}