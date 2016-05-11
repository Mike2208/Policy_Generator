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
	typedef unsigned int DIST_TYPE;

	const DIST_TYPE MAX_DIST = UINT_MAX;
}

// Information of one connection
struct CONNECTION_DATA
{
	OBSTACLE_ID ConnectedID;					// ID of connected obstacle
	POS_2D		MinCenterPos;					// Position between both obstacles that is at halfway point between them
	POS_2D		MinStartPos;					// Position on current obstacle that is closest to other obstacle
	POS_2D		MinDestPos;						// Position on connected obstacle that is closest to this obstacle
	OBSTACLE_CONNECTION::DIST_TYPE	Distance;	// Distance from center to obstacle
};

class ObstacleConnection
{
	public:
		ObstacleConnection();
		~ObstacleConnection();

		void Reset();		// Remove all data

		int AddConnection(const CONNECTION_DATA &NewData);			// Add one connection, this also sets connections as unsorted
		int UpdateConnection(const OBSTACLE_ID &ConnectedObstacle, const POS_2D &ConnectionPos, const OBSTACLE_CONNECTION::DIST_TYPE Distance);		// Replaces/adds one connection only if new distance is smaller

		int RecalculateAllData(const ObstacleMap &Obstacles, const Map<OBSTACLE_CONNECTION::DIST_TYPE> &DistMap, const Map_ID &IDMap);				// Recalculates minimum position on obstacle and sorts connections in a counter-clockwise fashion

		int GetConnection(const OBSTACLE_ID &ConnectedObstacleID, CONNECTION_DATA &Data)const;		// Returns connection data between two obstacles if available

	private:

		OBSTACLE_ID _ID;													// ID of this obstacle
		std::vector<CONNECTION_DATA> _Connections;							// Connections to other obstacles
		bool		_ConnectionsSorted;										// Are the saved connections sorted?
		bool		_ConnectionsCalculated;									// Does connection data need to be recalculated?

		int SortConnections(const ObstacleMap &Obstacles);			// Sort connection data to circle around obstacle CCW
		int RecalculateConnectionData(const Map_ID &IDMap, const Map<OBSTACLE_CONNECTION::DIST_TYPE> &DistMap);		// Recalculate Minimum distance positions

		POS_2D SortGetNextObstacleEdge(const ObstacleMap &Obstacles, const OBSTACLE_ID &ObstacleID, const POS_2D &CurPos);		// Gets next position along edge of current obstacle
};

#endif // OBSTACLECONNECTION_H
