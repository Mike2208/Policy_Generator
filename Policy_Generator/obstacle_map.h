#ifndef OBSTACLE_MAP_H
#define OBSTACLE_MAP_H

/*	class ObstacleMap
 *		a map of all obstacles
 *
 */

#include "map.h"
#include "map_standards.h"
#include "obstacle_path_finder.h"
#include <vector>
#include <queue>

namespace OBSTACLE_MAP
{
	typedef std::vector<POS_2D>	OBSTACLE_POSITIONS;
	typedef Map_IntType			INTERNAL_OBSTACLE_MAP;		// Enough for Obstacle IDs
	typedef std::queue<POS_2D>	POS_TO_CHECK_TYPE;			// type used to store future positions to check

	const unsigned int EmptyID = MAP_CONST_EDGE_IDS::EmptyID;
}

class ObstacleMap
{
	public:

		ObstacleMap();

		void ResetMap();
		void ResetMap(const Map<OCCUPANCYGRID_DISCRETE_TYPE> &NewObstacleMap);		// Copies the obstacle map and locates all obstacles

		int AddObstaclePos(const POS_2D &NewPosition);				// Adds an obstacle at the given position (if it is connected to another obstacle, no new obstacle is created, the old one is just updated)
		int RemoveObstaclePos(const POS_2D &Position);				// Removes obstacle data

		unsigned int GetNumObstacles() const;		// Returns number of obstacles

	private:

		OBSTACLE_MAP::INTERNAL_OBSTACLE_MAP	_ObstacleMap;			// Map of all obstacles ()
		OBSTACLE_MAP::OBSTACLE_POSITIONS	_ObstaclePositions;		// Positions of all obstacles

		void CombineTwoIDs(const unsigned int &OriginalID, const unsigned int &IDToCombine);		// Combines both IDs (IDToCombine is removed, and all its positions are replaced with OriginalID)
		inline void CombineTwoIDs_Step(const unsigned int &OriginalID, const unsigned int &IDToCombine, OBSTACLE_MAP::POS_TO_CHECK_TYPE &PosToCheck);		// Step in combination
};

#endif // OBSTACLE_MAP_H
