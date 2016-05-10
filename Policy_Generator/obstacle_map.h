#ifndef OBSTACLE_MAP_H
#define OBSTACLE_MAP_H

/*	class ObstacleMap
 *		a map of all obstacles
 *
 */

#include "map.h"
#include "map_standards.h"
#include "occupancy_grid_map.h"

#include <queue>

namespace OBSTACLE_MAP
{
	typedef std::queue<POS_2D> POS_TO_CHECK_TYPE;
}

class ObstacleMap
{
	public:

		ObstacleMap();

		void Reset();

		void FindAllObstacles(const OGM_MAP &MapData, const OGM_TYPE &Threshold);		// Finds all obstacles above a certain value

		int AddObstaclePos(const POS_2D &NewPosition);				// Adds an obstacle at the given position (if it is connected to another obstacle, no new obstacle is created, the old one is just updated)
		int RemoveObstaclePos(const POS_2D &Position);				// Removes obstacle data

		unsigned int GetNumObstacles() const;		// Returns number of obstacles

		int GetPosID(const POS_2D &Position, OBSTACLE_ID &ID) const;		// Returns ID at Position

	private:

		Map_ID					_ObstacleMap;			// Map of all obstacles ()
		std::vector<POS_2D>		_ObstaclePositions;		// Positions of all obstacles

		void CombineTwoIDs(const OBSTACLE_ID &OriginalID, const OBSTACLE_ID &IDToCombine);		// Combines both IDs (IDToCombine is removed, and all its positions are replaced with OriginalID)
		inline void CombineTwoIDs_Step(const OBSTACLE_ID &OriginalID, const OBSTACLE_ID &IDToCombine, OBSTACLE_MAP::POS_TO_CHECK_TYPE &PosToCheck);		// Step in combination
};

#endif // OBSTACLE_MAP_H
