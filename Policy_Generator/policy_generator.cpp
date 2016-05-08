#include "policy_generator.h"
#include "obstacle_district_manager.h"
#include "obstacle_connections.h"

PolicyGenerator::PolicyGenerator()
{

}

int PolicyGenerator::CalculatePolicy(const OGM_MAP &OGMmap, PolicyData &DecisionData)
{
	// Get average probability
	const OGM_TYPE threshold = this->CalculateAverageProbability(OGMmap);

	// Find obstacles given this threshold

}
