#include "obstacle_district_manager.h"
#include "obstacle_connection_manager.h"
#include "obstacle_district_map.h"

ObstacleDistrictManager::ObstacleDistrictManager()
{

}

void ObstacleDistrictManager::Reset()
{
	this->_DistrictMap.ResetMap(OBSTACLE_DISTRICT_MAP::EmptyID);
	this->_Districts.clear();
}

int ObstacleDistrictManager::CalculateDistricts(const OGM_MAP &OGMmap, const OGM_TYPE Threshold)
{
	// Data is reset later in other CalculateDistricts function

	// Find obstacles using Threshold as decision
	ObstacleMap obstacles;
	obstacles.FindAllObstacles(OGMmap, Threshold);

	// Find all districts
	return this->CalculateDistricts(obstacles);
}

int ObstacleDistrictManager::CalculateDistricts(const ObstacleMap &ObstacleMap)
{
	// Reset data
	this->Reset();

	// Get all connections
	ObstacleConnectionManager connectionData;
	connectionData.CalculateAllConnections(ObstacleMap);

	// Divide free area into districts
	ObstacleDistrictMap districtMap;
	districtMap.CalculateAllDistrictAreas(ObstacleMap, connectionData);

	// TODO: Go through district map and create districts for all IDs

	return -1;
}
