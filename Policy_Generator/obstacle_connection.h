#ifndef OBSTACLECONNECTION_H
#define OBSTACLECONNECTION_H

/*	class ObstacleConnection
 *		stores connections of one obstacle with it's neighbors
 */

#include "map_standards.h"
#include "map.h"
#include "obstacle_map.h"

#include <vector>
#include <array>

namespace OBSTACLE_CONNECTION
{
}

// Information of one connection
struct CONNECTION_DATA
{
	OBSTACLE_ID ConnectedID;		// ID of connected obstacle
	POS_2D		MinCenterPos;		// Position between both obstacles that is at halfway point between them
	POS_2D		MinStartPos;		// Position on current obstacle that is closest to other obstacle
	POS_2D		MinDestPos;			// Position on connected obstacle that is closest to this obstacle
};

class ObstacleConnection
{
	public:
		ObstacleConnection();
		~ObstacleConnection();

		void Reset();		// Remove all data

		int AddConnection(const CONNECTION_DATA &NewData);			// Add one connection, this also sets connections as unsorted
		int SortConnections(const ObstacleMap &Obstacles);			// Obstacle data

	private:

		OBSTACLE_ID _ID;													// ID of this obstacle
		std::vector<CONNECTION_DATA> _Connections;							// Connections to other obstacles
		bool		_ConnectionsSorted;										// Are the saved connections sorted?

		POS_2D SortGetNextObstacleEdge(const ObstacleMap &Obstacles, const OBSTACLE_ID &ObstacleID, const POS_2D &CurPos);		// Gets next position along edge of current obstacle
};

#endif // OBSTACLECONNECTION_H
