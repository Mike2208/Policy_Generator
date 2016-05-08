#ifndef OBSTACLECONNECTIONMANAGER_H
#define OBSTACLECONNECTIONMANAGER_H

/*	class ObstacleConnectionManager
 *		manages all connections of a given map
 *
 */

#include "obstacle_map.h"
#include "obstacle_connection.h"

#include <vector>

class ObstacleConnectionManager
{
	public:
		ObstacleConnectionManager();

		int CalculateAllConnections(const ObstacleMap &MapData);

	private:

		std::vector<ObstacleConnection>	_ConnectionData;		// Contains all obstacles and their connections between each other
};

#endif // OBSTACLECONNECTIONMANAGER_H
