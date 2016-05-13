#include "policy_generator.h"
#include "obstacle_district_manager.h"
#include "obstacle_map.h"

PolicyGenerator::PolicyGenerator()
{

}

int PolicyGenerator::CalculatePolicy(const OGM_MAP &OGMmap, PolicyData &DecisionData)
{
	// Get average probability
	const OGM_TYPE threshold = OccupancyGridMap::CalculateAverageProbability(OGMmap);

	// Find obstacles using this threshold as decision
	ObstacleMap obstacles;
	obstacles.FindAllObstacles(OGMmap, threshold);

	ObstacleDistrictManager districts;

	// Find minimum distances between obstacles to use as dividers between districts
	districts.CalculateDistricts(obstacles);

	// TODO: Create a graph between districts and dividers

	return -1;
}
