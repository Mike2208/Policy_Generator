#ifndef OBSTACLEDISTRICTMANAGER_H
#define OBSTACLEDISTRICTMANAGER_H

#include "map_standards.h"
#include "map.h"
#include "occupancy_grid_map.h"
#include "obstacle_map.h"
#include "obstacle_district.h"

#include <vector>

typedef std::vector<> DISTRICT_CONNECTIONS;

class ObstacleDistrictManager
{
	public:
		ObstacleDistrictManager();

		int CalculateDistricts(const OGM_MAP &OGMmap, const OGM_TYPE Threshold);
		int CalculateDistricts(const ObstacleMap &OGMmap);
		int DivideDistrict(const OBSTACLE_ID &DistrictID, const OGM_TYPE &Threshold);



	private:


		std::vector<ObstacleDistrict> _Districts;

};

#endif // OBSTACLEDISTRICTMANAGER_H
