#ifndef BSO_GRAMMAR_DESIGN_HUMAN_CPP
#define BSO_GRAMMAR_DESIGN_HUMAN_CPP

#include <bso/utilities/clustering.hpp>
#include <stdexcept>
#include <sstream>

namespace bso { namespace grammar {
class DESIGN_HUMAN;

template <>
bso::structural_design::sd_model grammar::sd_grammar<DESIGN_HUMAN>(
    const std::string& fileName,
    const bso::structural_design::component::structure& flatShellStructure,
	const bso::structural_design::component::structure& substituteStructure)
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
		for (const auto& j : i.first->cfLines()) {
			if (mSDLineRules.find(j) == mSDLineRules.end()) {  // Removed the semicolon
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

	// Execute rectangle and line rules
	for (const auto& j : mSDVertexRules) j.second->apply(mSDModel);
	for (const auto& j : mSDLineRules) j.second->apply(mSDModel);
	for (const auto& j : mSDRectangleRules) j.second->apply(mSDModel);
	mSDModel.setMeshSize(mMeshSize);

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


	std::cout << "GRM nr subrec: " << subRectangles.size() << std::endl;
	// inventory the rectangles that must be substituted
	std::vector<bso::spatial_design::conformal::cf_rectangle*> mustSubstitute;

	// add all rectangles to the list of rectangles that must be substituted.
	for (const auto& j : subRectangles)
	{
		mustSubstitute.push_back(j.first);
	}

	// Now assign mustSubstitute to the SD model
	for (const auto& rectangle : mustSubstitute) {
		mSDModel.addSubRectangle(rectangle);  // Add each rectangle to the SD model
	}

	//unsigned int nSubRectangles = subRectangles.size();
    //std::cout << "Mustsubstitue size: " << mustSubstitute.size() << std::endl;
	std::cout << "GRM nr subrecsize: " << mSDModel.getSubRectangles().size() << std::endl;
	std::cout << "GRM Strain:" << mSDModel.getTotalResults().mTotalStrainEnergy << std::endl;
    std::cout << "GRM Volume:" << mSDModel.getTotalResults().mTotalStructuralVolume << std::endl;


	// delete the rules
	for (auto& i : mSDVertexRules) delete i.second;
	for (auto& i : mSDLineRules) delete i.second;
	for (auto& i : mSDRectangleRules) delete i.second;

	return mSDModel;
} // default_sd_grammar

} // namespace grammar
} // namespace bso

#endif // BSO_GRAMMAR_DESIGN_HUMAN_CPP
