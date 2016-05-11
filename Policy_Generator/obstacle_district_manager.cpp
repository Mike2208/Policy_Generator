#include "obstacle_district_manager.h"
#include "obstacle_connection_manager.h"

ObstacleDistrictManager::ObstacleDistrictManager()
{

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

	// Divide map into districts that contain both the obstacles and the separate free areas
	unsigned int numDistricts = ObstacleMap.GetNumObstacles();

	// Calculate number of separated free areas in map
	numDistricts += this->CalculateNumSeparateDistricts(connectionData);

	// Reserve enough room for data
	this->_Districts.resize(numDistricts);

	for(unsigned int i=0; i<numDistricts; i++)
	{

	}
}

unsigned int ObstacleDistrictManager::CalculateNumSeparateDistricts(const ObstacleConnectionManager &Connections)
{

}
