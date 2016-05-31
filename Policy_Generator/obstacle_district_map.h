#ifndef OBSTACLE_DISTRICT_MAP_H
#define OBSTACLE_DISTRICT_MAP_H

/*	class ObstacleDistrictMap
 *		stores map of all districts
 */

#include "map_standards.h"
#include "map.h"

#include "obstacle_map.h"
#include "obstacle_connection_manager.h"

namespace OBSTACLE_DISTRICT_MAP
{
	const OBSTACLE_ID DistrictID = OBSTACLE_ID_RESERVED;
	const OBSTACLE_ID EmptyID = OBSTACLE_ID_EMPTY;
}

class ObstacleDistrictMap
{
	public:

		ObstacleDistrictMap();
		ObstacleDistrictMap(const ObstacleDistrictMap &S);

		void Reset();

		int CalculateAllDistrictAreas(const ObstacleMap &ObstacleData, const ObstacleConnectionManager &Connections);	// Calculates and stores all districts given a obstacleMap and connection

	private:

		Map_ID				_IDMap;					// Map of all ditricts with their IDs
		std::vector<POS_2D>	_IDPositions;			// Start Positions of all IDs

		int AddObstacleToMap(const ObstacleMap &ObstacleData, const OBSTACLE_ID &ObstacleID);			// Add all obstacles to _IDMap
		int AddDistrictDividers(const ObstacleMap &ObstacleData, const ObstacleConnectionManager &Connections, const OBSTACLE_ID &DividerID);		// Adds dividers between districts

		void SetPathToID(const POS_2D &StartPos, const POS_2D &DestPos, const OBSTACLE_ID &ID, Map_ID &Map);
		void SetDistrictToID(const POS_2D &StartPos, const OBSTACLE_ID &IDtoReplace, const OBSTACLE_ID &DividerID, const OBSTACLE_ID &NewID, Map_ID &Map);		// Sets all positions around StartPos with same ID to NewID
};

#endif // OBSTACLE_DISTRICT_MAP_H
