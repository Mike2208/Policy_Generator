#include "obstacle_district_manager.h"

ObstacleDistrictManager::ObstacleDistrictManager()
{

}

int ObstacleDistrictManager::CalculateDistricts(const OGM_MAP &OGMmap, const OGM_TYPE Threshold)
{
	// Find obstacles using Threshold as decision
	ObstacleMap obstacles;
	obstacles.FindAllObstacles(OGMmap, Threshold);

	// Find all districts
	return this->CalculateDistricts(obstacles);
}

int ObstacleDistrictManager::CalculateDistricts(const ObstacleMap &OGMmap)
{

}
