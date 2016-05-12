#ifndef OBSTACLE_CONNECTION_MANAGER_H
#define OBSTACLE_CONNECTION_MANAGER_H

/*	class ObstacleConnectionManager
 *		manages all connections of a given map
 *
 */

#include "obstacle_map.h"
#include "obstacle_connection.h"

#include <vector>
#include <queue>

namespace OBSTACLE_CONNECTION_MANAGER
{
	typedef OBSTACLE_CONNECTION::DIST_TYPE DIST_TYPE;
	typedef Map<DIST_TYPE> DIST_MAP;

	typedef std::queue<POS_2D>	POS_TO_CHECK;
	typedef std::vector<ObstacleConnection> OBSTACLE_CONNECTIONS;

	//const OBSTACLE_ID TopID_Sub = 1;		// Subtract this from array size to get top ID
	//const OBSTACLE_ID BottomID_Sub = 2;		// Subtract this from array size to get bottom ID
	//const OBSTACLE_ID LeftID_Sub = 3;		// Subtract this from array size to get left ID
	//const OBSTACLE_ID RightID_Sub = 4;		// Subtract this from array size to get right ID
};

class ObstacleConnectionManager
{
	public:
		ObstacleConnectionManager();

		void Reset();		// Erases data

		int CalculateAllConnections(const ObstacleMap &MapData);		// Gets all connections between obstacles (only one connection between two obstacles)

		unsigned int GetNumConnectionsToObstacle(const OBSTACLE_ID &ObstacleID)const;		// Returns number of connections to obstacle
		int GetConnectionData(const OBSTACLE_ID &ObstacleID, const unsigned int &ConnectionNum, CONNECTION_DATA &Data)const;

	private:

		OBSTACLE_CONNECTION_MANAGER::OBSTACLE_CONNECTIONS	_ConnectionData;		// Contains all obstacles and their connections between each other

		int EnlargeObstaclesUntilCollision(const ObstacleMap &MapData, Map_ID &IDMap, OBSTACLE_CONNECTION_MANAGER::DIST_MAP &DistMap, OBSTACLE_CONNECTION_MANAGER::OBSTACLE_CONNECTIONS &Connections);			// Performs the work to find obstacle connections
		void EnlargeObstaclesUntilCollision_Step(const ObstacleMap &MapData, const POS_2D &CurPos, Map_ID &IDMap, OBSTACLE_CONNECTION_MANAGER::DIST_MAP &DistMap, OBSTACLE_CONNECTION_MANAGER::POS_TO_CHECK &PosToCheck, OBSTACLE_CONNECTION_MANAGER::OBSTACLE_CONNECTIONS &Connections);			// One step to find obstacle collisions
};

#endif // OBSTACLE_CONNECTION_MANAGER_H
