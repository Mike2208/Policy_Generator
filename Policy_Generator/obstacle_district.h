#ifndef OBSTACLE_DISTRICT_H
#define OBSTACLE_DISTRICT_H

/*	class ObstacleDistricts
 *		divides a map with obstacles into separate districts
 *
 */

#include "map_standards.h"
#include "map.h"
#include "occupancy_grid_map.h"

namespace OBSTACLE_DISTRICT
{
	typedef Map<OCCUPANCYGRID_DISCRETE_TYPE> DISTRICT_MAP;
}

typedef std::vector<POS_2D>		DISTRICT_DATA;		// Data t

class ObstacleDistrict
{
	public:
		ObstacleDistrict();

		void ExtractDataFromMap(const Map_ID &MapData, const OBSTACLE_ID &DistrictID);		// Extracts individual district data from a map
		void ExtractDataFromMap(const Map_ID &MapData, const OBSTACLE_ID &DistrictID, const POS_2D &StartSearchPos);		// Gets district data from a map (StartSearchPos should be set to a position with DistrictID)

		bool IsPositioninDistrict(const POS_2D &Position);		// Checks if position is in district

	private:

		POS_2D							_MapPosition;		// Position of bottom left corner of map
		OBSTACLE_DISTRICT::DISTRICT_MAP	_Map;				// Map data stating where a district is located
		OBSTACLE_ID						_ID;				// ID of current district

};

#endif // OBSTACLE_DISTRICT_H
