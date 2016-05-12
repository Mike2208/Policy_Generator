#ifndef OBSTACLEDISTRICTMANAGER_H
#define OBSTACLEDISTRICTMANAGER_H

#include "map_standards.h"
#include "map.h"
#include "occupancy_grid_map.h"
#include "obstacle_map.h"
#include "obstacle_district.h"
#include "obstacle_connection_manager.h"

#include <vector>

class ObstacleDistrictManager
{
	public:
		ObstacleDistrictManager();

		void Reset();

		int CalculateDistricts(const OGM_MAP &OGMmap, const OGM_TYPE Threshold);		// Calculate all districts given a map
		int CalculateDistricts(const ObstacleMap &ObstacleMap);							// Calculate all districts given obstacle map
		int DivideDistrict(const OBSTACLE_ID &DistrictID, const OGM_TYPE &Threshold);	// Divides a district into smaller districts


	private:

		Map_ID						_DistrictMap;			// Map of all districts
		std::vector<ObstacleDistrict> _Districts;			// Separate districts

		unsigned int CalculateNumSeparateDistricts(const ObstacleConnectionManager &Connections);		// returns number of districts Connections separates the free space into
};

#endif // OBSTACLEDISTRICTMANAGER_H
