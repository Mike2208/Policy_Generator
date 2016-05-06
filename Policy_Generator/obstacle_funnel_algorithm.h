#ifndef OBSTACLEFUNNELALGORITHM_H
#define OBSTACLEFUNNELALGORITHM_H

/* class ObstacleFunnelAlgorithm
 *	calculates optimal path from start to finish through polygon set
 *	here, it is used to calculate the best route from start to destination while navigating around obstacles
 */

#include "map_standards.h"
#include "map.h"
#include "map_path.h"
#include "obstacle_connections.h"
#include "obstacle_path_finder.h"		// for OBSTACLE_PATH_FINDER::MAP_ID_DIST

#include <array>

namespace OBSTACLE_FUNNEL_ALGORITHM
{
	struct SINGLE_VERTICE_PATH_DATA
	{
			unsigned int ObstacleID;			// obstacle to rotate around
			unsigned int ConnectedObstacleID;	// connected obstacle
	};

	struct CONNECTION_POSITIONS
	{
			std::array<unsigned int,2>		IDs;
			std::array<POS_2D,2>	Positions;
	};

	struct SINGLE_VERTICE_POSITIONS
	{
			std::array<POS_2D,3>	Positions;
	};
}

typedef std::vector<OBSTACLE_FUNNEL_ALGORITHM::SINGLE_VERTICE_PATH_DATA> VERTICE_PATH_DATA;

class ObstacleFunnelAlgorithm
{
	public:
		ObstacleFunnelAlgorithm();


		int CalculateAllRoutes(const ObstacleConnections &ObstacleData, const POS_2D &StartPos, const POS_2D &Destination);			// Calculates all routes that do not circle an obstacle more than once

		int CalculateOptimalRoute(const ObstacleConnections &ObstacleData, const Map<OBSTACLE_PATH_FINDER::MAP_ID_DIST> &IdDistMap, const VERTICE_PATH_DATA &PathData, const POS_2D &StartPos, const POS_2D &Destination);		// Calculates optimal path through obstacles given desired route, start and destination

	private:


};
#endif // OBSTACLEFUNNELALGORITHM_H
