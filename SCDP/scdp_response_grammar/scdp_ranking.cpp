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
	std::string inputFile = "";
	std::string designName = "";
	std::string inputLine = "";
	std::vector<std::string> visualizations;
	bool result_line = false;
	bool ms_files = false;
	bool verbose = false;
	bool outputAll = false;
	bool outputBest = false;
	double etaBend = 0;
	double etaAx = 0;
	double etaShear = 0;
	double etaNoise = 0;
	int etaConverge = 0;
	int nSpacesDelete = 0;
	std::string checkingOrder = "123";
	unsigned int iterations = 0;

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
		if (*arg == "-h" || *arg == "--help")
		{
			std::cout << "This program ...." << std::endl;
			std::cout << std::endl;
			std::cout << "-i\t" << "or --input, to specify an input file,\n"
								<<   "\t" << "expects a string containing the file name\n"
								<<   "\t" << "of the input file. The input file should be\n"
								<<   "\t" << "in \'Movable Sizable\' format.\n";
			std::cout << "-l\t" << "or --loops, to specify the number of loops\n"
								<<   "\t" << "that will be applied. Expects only a positive integer\n";
			std::cout << "-e\t" << "or --export, to specify the diferent outputs:\n"
								<<   "\t" << "\'result_line\' prints a line with the input\n"
								<<   "\t" << "parameters and the results; \'verbose\' prints\n"
								<<   "\t" << "a descriptive outline of the process;\n"
								<<   "\t" << "\'ms_files\' will write the ms designs to files\n"
								<<   "\t" << "\'all\' will write all the designs to the specified format\n"
								<<   "\t" << "\'best\' (default) will only write the best all the designs to\n"
								<<   "\t" << "the specified format. More than one option can be given,\n"
								<<   "\t" << "but \'all\' and \'best\' cannot be given simultaneously.\n";
			std::cout << "-v\t" << "or --visualization, to specify which models are\n"
								<<   "\t" << "visualized: \'ms\' for \'Movable Sizable\',\n"
								<<   "\t" << "\'sc\' for \'Super Cube\', \'rc\' for \'conformal model with\n"
								<<   "\t" << "ReCtangles\', \'cb\' for \'conformal model with CuBoids\'\n"
								<<   "\t" << "\'sd\' for'\'Structural Design\', \'fe\' for \'Finite\n"
								<<   "\t" << "Elements of the structural design model\',\n"
								<<   "\t" << "and \'bp\' for the \'Building Physics model\'.\n"
								<<   "\t" << "More than one can be specified.\n";
            std::cout << "-s\t"	<< "or --shear, expects a real value x where 0 <= x <= 1\n";
            std::cout << "-a\t" << "or --axial, expects a real value x where 0 <= x <= 1\n";
            std::cout << "-b\t" << "or --bending, expects a real value x where 0 <= x <= 1\n";
            std::cout << "-c\t" << "or --convergence, expects a positive integer larger than 0.\n";
            std::cout << "-x\t" << "or --nSpacesDelete, expects a positive integer larger than 0.\n";
			std::cout << "-n\t" << "or --noise, expects a real value x where 0 <= x <= 1\n";
            std::cout << "-o\t" << "or --order, expects a string of three integers with value 1, 2, or 3.\n";
			std::cout << "\n\n" << "Press ENTER to continue..." << std::endl;
			std::cin.get();
			return 0;
		}
		else if (*arg == "-i" || *arg == "--input")
		{
			if (++arg == args.end() || arg->operator[](0) == '-')
			{
				std::stringstream errorMessage;
				errorMessage << "\nError, expected the name of the input file after -i or --input\n"
										 << "Use -h or --help for help" << std::endl;
				throw std::invalid_argument(errorMessage.str());
			}
			inputFile = *arg;
			boost::trim(inputFile);
			designName = inputFile;
			++arg;
		}
		else if (*arg == "-l" || *arg == "--loops")
		{
			if (++arg == args.end() || arg->operator[](0) == '-')
			{
				std::stringstream errorMessage;
				errorMessage << "\nError, expected a positive integer to be\n"
										 << "specified after -l or --loops. Use -h or\n"
										 << "--help for help\n";
				throw std::invalid_argument(errorMessage.str());
			}
			try
			{
				iterations = bso::utilities::trim_and_cast_uint(*arg);
			}
			catch (std::exception& e)
			{
				std::stringstream errorMessage;
				errorMessage << "\nError, could not parse argument specified after\n"
										 << "-l or --loops. received the following error:\n"
										 << e.what()
										 << "Use -h or -- help for help\n";
				throw std::invalid_argument(errorMessage.str());
			}

			if (++arg != args.end() && arg->operator[](0) != '-')
			{
				std::stringstream errorMessage;
				errorMessage << "\nError, only one argument can be specified after\n"
										 << "-l or --loops. Use -h or -- help for help\n";
				throw std::invalid_argument(errorMessage.str());
			}
		}
        else if (*arg == "-s" || *arg == "--shear")
        {
            if (++arg == args.end() || (*arg)[0] == '-') throw std::domain_error("Expected a value after -s or --shear");
            double value = bso::utilities::trim_and_cast_double(*arg);
            if (value < 0 || value > 1) throw std::domain_error("expected a value between 0 and 1 for eta_shear");
            etaShear = value;
            arg++;
        }
        else if (*arg == "-a" || *arg == "--axial")
        {
            if (++arg == args.end() || (*arg)[0] == '-') throw std::domain_error("Expected a value afyer -a or --axial");
            double value = bso::utilities::trim_and_cast_double(*arg);
            if (value < 0 || value > 1) throw std::domain_error("expected a value between 0 and 1 for eta_ax");
            etaAx = value;
            arg++;
        }
        else if (*arg == "-b" || *arg == "--bend")
        {
            if (++arg == args.end() || (*arg)[0] == '-') throw std::domain_error("Expected a balue after -b or --bend");
            double value = bso::utilities::trim_and_cast_double(*arg);
            if (value < 0 || value > 1) throw std::domain_error("expected a value between 0 and 1 for eta_bend");
            etaBend = value;
            arg++;
        }
		else if (*arg == "-n" || *arg == "--noise")
        {
            if (++arg == args.end() || (*arg)[0] == '-') throw std::domain_error("Expected a value after -n or --noise");
            double value = bso::utilities::trim_and_cast_double(*arg);
            if (value < 0 || value > 1) throw std::domain_error("expected a value between 0 and 1 for eta_noise");
            etaNoise = value;
            arg++;
        }
        else if (*arg == "-c" || *arg == "--convergence")
        {
            if (++arg == args.end() || (*arg)[0] == '-') throw std::domain_error("Expected a value after -c or --convergence");
            int value = bso::utilities::trim_and_cast_int(*arg);
            if (value < 1) throw std::domain_error("Expected a positive int larger than zero for eta_convergence");
            etaConverge = value;
            arg++;
        }
		else if (*arg == "-x" || *arg == "--nSpacesDelete")
        {
            if (++arg == args.end() || (*arg)[0] == '-') throw std::domain_error("Expected a value after -x or --nSpacesDelete");
            int value = bso::utilities::trim_and_cast_int(*arg);
            if (value < 1) throw std::domain_error("Expected a positive int larger than zero for nSpacesDelete");
            nSpacesDelete = value;
            arg++;
        }
        else if (*arg == "-o" || *arg == "--order")
        {
            if (++arg == args.end() || (*arg)[0] == '-') throw std::domain_error("Expected a value after -o or --order");
            std::string value = *arg;
            if (value.size() != 3) throw std::domain_error("Expected a string of three ints with value 1, 2, or 3 for checking order");
            for (auto c : value) if (c < '1' || c > '3') throw std::domain_error("Expected a string of three ints with value 1, 2, or 3 for checking order");
            checkingOrder = value; // add the four at the back for no structure
            arg++;
        }
		else if (*arg == "-e" || *arg == "--export")
		{
			while (++arg != args.end() && arg->operator[](0) != '-')
			{
				std::string outSpec = *arg;
				boost::trim(outSpec);
				if 			(outSpec == "result_line") result_line = true;
				else if (outSpec == "verbose") verbose = true;
				else if (outSpec == "ms_files") ms_files = true;
				else if (outSpec == "all") outputAll = true;
				else if (outSpec == "best") outputBest = true;
				else
				{
					std::stringstream errorMessage;
					errorMessage << "\nError, did not recognize: " << outSpec << " as an\n"
											 << "argument for -e or --export. Use -h or\n"
											 << "--help for help\n";
					throw std::invalid_argument(errorMessage.str());
				}
			}
		}
		else if (*arg == "-v" || *arg == "--visualize")
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
	if (inputFile == "" && inputLine == "")
	{
		std::stringstream errorMessage;
		errorMessage << "\nError, expected an input file or line to be specified.\n"
								 << "use -h or --help for help\n";
		throw std::invalid_argument(errorMessage.str());
	}
	if ((ms_files || result_line) && (outputAll && outputBest))
	{
		std::stringstream errorMessage;
		errorMessage << "Cannot specify both \'all\'  and \'best\' for output.\n"
								 << "use -h or --help for help\n";
		throw std::invalid_argument(errorMessage.str());
	}
	if (!outputAll && !outputBest) outputBest = true;
	out("Parsed program arguments", true, true, verbose);
	if (!visualizations.empty()) visualization::initVisualization(argc,argv);
	out("Initialized visualization", true, true, verbose);

	std::vector<spatial_design::ms_building> msDesigns;
	std::vector<spatial_design::ms_building> msDesignsTemp;
	try
	{
		if (!inputFile.empty()) msDesigns.push_back(spatial_design::ms_building(inputFile));
		else
		{
			spatial_design::sc_building sc(inputLine);
			msDesigns.push_back(spatial_design::ms_building(sc));
		}
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
	for (unsigned int i = 0; i <= iterations; ++i)
	{
		out("Iteration: ",false,false,verbose); out(i,true,false,verbose);
		if (std::find(visualizations.begin(),visualizations.end(),"sc") != visualizations.end() ||
				result_line)
		{
			scDesigns.push_back(spatial_design::sc_building(msDesigns.back()));
			out("initialized sc building",true,true,verbose);
		}

		// step 1, generate SD and BP models and evaluate them.
		spatial_design::cf_building cf(msDesigns.back(),1e-6);
		out("Initialized conformal model",true,true,verbose);

		grammar::grammar grm(cf);
		out("Initialized grammar",true,true,verbose);

		bso::structural_design::component::structure trussStructure("truss",{{"A",2250},{"E",3e4}});
		bso::structural_design::component::structure beamStructure("beam",{{"width",150},{"height",150},{"poisson",0.3},{"E",3e4}});
		bso::structural_design::component::structure flatShellStructure("flat_shell",{{"thickness",150},{"poisson",0.3},{"E",3e4}});
		bso::structural_design::component::structure substituteStructure("flat_shell",{{"thickness",150},{"poisson",0.3},{"E",3e-2}});

		sdModels.push_back(grm.sd_grammar<grammar::DESIGN_RESPONSE>(std::string("settings/sd_settings.txt"),etaBend,etaAx,etaShear,etaNoise,etaConverge,checkingOrder,trussStructure,beamStructure,flatShellStructure,substituteStructure));
		out("Generated structural model",true,true,verbose);

		sdModels.back().analyze();
		out("Evaluated structural model: ",false,false,verbose);
		double sdResult = sdModels.back().getTotalResults().mTotalStrainEnergy;
		out(sdResult,true,true,verbose);

		if (!visualizations.empty()) for (const auto& visOption : visualizations)
		{
			if (visOption == "ms" && i != 0) visualization::visualize(msDesignsTemp.back(),"","ms_building",4.0);
			if (visOption == "ms") visualization::visualize(msDesigns.back(),"","ms_building",4.0);
			if (visOption == "sc") visualization::visualize(scDesigns.back());
			if (visOption == "rc") visualization::visualize(cf,"rectangle");
			if (visOption == "cb") visualization::visualize(cf,"cuboid");
			if (visOption == "sd") visualization::visualize(sdModels.back());
			if (visOption == "fe") visualization::visualize(sdModels.back(), "element");
			if (visOption == "fe") visualization::visualize(sdModels.back(), "strain_energy");
		}
		out("Sent models to visualization",true,true,verbose);
		if (i >= iterations) break;

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

        newMS.setZZero();
        std::vector<spatial_design::ms_space*> floatingSpaces;
        if (newMS.hasFloatingSpaces(floatingSpaces)) 
		{
            for (auto& i : floatingSpaces) newMS.deleteSpace(i);
        }
		out("Removed worst performing spaces",true,true,verbose);
		msDesignsTemp.push_back(newMS);
		
        // Step to scale and split spaces to recover initial conditions
        // Scale the dimensions in x and y direction
        double scaleFactor = sqrt(floorArea / newMS.getFloorArea());
        newMS.scale({{0,scaleFactor},{1,scaleFactor}});
		newMS.snapOn({{0,1},{1,1}});

        newMS.setZZero();
        out("Scaled design to initial volume", true, true, verbose);

        msDesigns.push_back(newMS);
        out("Modified building spatial design into a new one", true, true, verbose);

	}
    out("finished SCDP process",true,true,verbose);

	if (!visualizations.empty()) visualization::endVisualization();

	return 0;
}


