#ifndef BSO_GRAMMAR_DESIGN_INPUT_CPP
#define BSO_GRAMMAR_DESIGN_INPUT_CPP

#include <bso/utilities/clustering.hpp>
#include <stdexcept>
#include <sstream>

namespace bso { namespace grammar {
class DESIGN_INPUT;

template <>
bso::structural_design::sd_model grammar::sd_grammar<DESIGN_INPUT>(
	const std::string& fileName,
	const double& etaBend,
	const double& etaAx,
	const double& etaShear,
	const double& etaNoise,
	const int& etaConverge,
	const std::string& checkingOrder,
	const bso::structural_design::component::structure& trussStructure,
	const bso::structural_design::component::structure& beamStructure,
	const bso::structural_design::component::structure& flatShellStructure,
	const bso::structural_design::component::structure& substituteStructure,
	const std::vector<bso::structural_design::component::structure>& structureAssignments)
{
	// initialize a new SD model
	mSDModel = bso::structural_design::sd_model();
	
	//variable to store initial mean of the total design response
	double initMeanTot;

	// read the structural design settings
	this->mReadSDSettings(fileName);

	// assign substitute type to rectangles
	for (const auto& i : mRectangleProperties)
	{
		if (i.second->isSpaceSeparating())
			{
				// create a rectangle rule set
				auto sdRectangleRuleSet = new rule_set::sd_rectangle_rule(i.second);
				sdRectangleRuleSet->assignStructure(substituteStructure);
				sdRectangleRuleSet->assignLoadPanel(mLoadPanel);
				sdRectangleRuleSet->addLoads(&mLoads);
				mSDRectangleRules[i.first] = sdRectangleRuleSet;
			}
	}
	
	auto subRectangles = mSDRectangleRules;
	int initSubNumber = subRectangles.size();
	
	// find each line that is associated to a rectangle that has a rectangle rule set
	for (const auto& i : mSDRectangleRules)
	{ // for each sd rectangle rule
		for (const auto& j : i.first->cfLines())
		{ // for each line associated to the rectangle associated to the rectangle rule
			if (mSDLineRules.find(j) == mSDLineRules.end());
			{ // if it has not yet been added to the vector, add it
				mSDLineRules[j] = new rule_set::sd_line_rule(mLineProperties[j]);
			}
		}
	}
	
	// initialize vertex rules
	for (const auto& i : mVertexProperties)
	{
		if (i.second->isSpaceCorner())
		{
			mSDVertexRules[i.first] = new rule_set::sd_vertex_rule(i.second);
		}
	}

	for (int i = 0; i < etaConverge + 1; ++i) {
		// Execute rectangle and line rules
		for (const auto& j : mSDVertexRules) j.second->apply(mSDModel);
		for (const auto& j : mSDLineRules) j.second->apply(mSDModel);
		for (const auto& j : mSDRectangleRules) j.second->apply(mSDModel);
		mSDModel.setMeshSize(mMeshSize);

		// Check if the grammar has finished, if so, further evaluation is not needed
		if (i == etaConverge) break;

		// Evaluate SD model
		mSDModel.analyze();

		// Check if a rectangle is horizontal and exclude it from substitution
		for (const auto& rectangle : mRectangleProperties) {
			if (rectangle.second->isSpaceSeparating() && rectangle.second->isFloor()) { // Check if the rectangle is a floor
				// Assign flat shell type to horizontal floor rectangle
				mSDRectangleRules[rectangle.first]->assignStructure(flatShellStructure);
				// Remove the horizontal floor rectangle from subRectangles
				subRectangles.erase(rectangle.first);
			}
		}
		
		std::map<bso::spatial_design::conformal::cf_rectangle*, bso::structural_design::sd_results> subResults;
		std::vector<bso::utilities::data_point> resultData;
		
		// obtain design responses of substitute rectangles
		int count = 0;
		for (auto j : subRectangles)
		{
			auto sdResult = mSDModel.getPartialResults(j.first);
			subResults[j.first] = sdResult;
			resultData.push_back(bso::utilities::data_point({sdResult.mTotalStrainEnergy}));
		}
		
		if (i == 0)
		{
			for (auto j : subResults)
			{
				initMeanTot += (j.second.mTotalStrainEnergy / j.second.mTotalStructuralVolume);
			}
			initMeanTot /= subResults.size();
		}
		
		// check substitution convergence
		int targetSubNumber;
		bool initCheck = true;
		do 
		{ // compute the target number of substitute rectangles, and if already reached, skip to next iteration
			if (initCheck) initCheck = false;
			else ++i;
			targetSubNumber = std::max(0,initSubNumber - (int)std::ceil((i+1)*initSubNumber/(double)etaConverge));

		} while (targetSubNumber > subRectangles.size());
		
		// substitute the substitute rectangles
		while (targetSubNumber < subRectangles.size())
		{
			// inventory the rectangles that must be substituted
			std::vector<bso::spatial_design::conformal::cf_rectangle*> mustSubstitute;
			{ // if no clustering has been carried out
				// add all rectangles to the list of rectangles that must be substituted.
				for (const auto& j : subRectangles)
				{
					mustSubstitute.push_back(j.first);
				}
			}
			
			for (size_t index = 0; index < mustSubstitute.size(); ++index) {
				auto key = mustSubstitute[index];  // Assuming mustSubstitute holds keys from subRectangles
				if (index < structureAssignments.size()) {
					// Check if the key exists in subRectangles to prevent accessing non-existing keys
					if (subRectangles.find(key) != subRectangles.end()) {
						subRectangles[key]->assignStructure(structureAssignments[index]);
					}
				}
			}
			
			// remove the rectangles that have been substituted from subRectangles and subResults
			for (const auto& j : mustSubstitute)
			{
				subRectangles.erase(j);
				subResults.erase(j);
			}
			mustSubstitute.clear();

		}
		
		// check consequences of substitution for line rules
		for (auto& i : mSDLineRules)
		{ // for each sd line rule
			// find the potential structural types from all the possible sd rectangle rules this
			// line rule may be associated with
			std::vector<structural_design::component::structure> potentialStructures;
			for (const auto& j : i.first->cfRectangles())
			{ // for each rectangle that this line is associated with
				auto SDRectangleRuleSearch = mSDRectangleRules.find(j);
				if (SDRectangleRuleSearch != mSDRectangleRules.end())
				{ // if that rectangle is associated with an sd rectangle rule, store its structural type
					potentialStructures.push_back(SDRectangleRuleSearch->second->getStructure());
				}
			}

			// pass the potential structural types to the sd line rule
			i.second->assignStructure(potentialStructures);
		}
		
		// check convergence
		if (i != etaConverge) 
		{ // if there is still an iteration coming, store the model as intermediate
			mIntermediateSDModels.push_back(mSDModel);
			mSDModel = bso::structural_design::sd_model();
		}
	}
	
	// delete the rules
	for (auto& i : mSDVertexRules) delete i.second;
	for (auto& i : mSDLineRules) delete i.second;
	for (auto& i : mSDRectangleRules) delete i.second;
	
	return mSDModel;
} // default_sd_grammar

} // namespace grammar
} // namespace bso

#endif // BSO_GRAMMAR_DESIGN_INPUT_CPP