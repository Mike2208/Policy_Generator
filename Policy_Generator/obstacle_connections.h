#ifndef OBSTACLECONNECTIONS_H
#define OBSTACLECONNECTIONS_H

/*	class ObstacleConnections
 *		stores which obstacles are close to each other
 */

#include "map_standards.h"
#include "map.h"
#include "obstacle_map.h"

#include <vector>
#include <array>

namespace OBSTACLE_CONNECTIONS
{
	struct CONNECTION_DATA
	{
		OBSTACLE_ID		ConnectedObstacleID;		// ID of connected obstacle
		POS_2D			MinDistPosition;			// Position of minimum distance between obstacles
		std::array<POS_2D,2>	MinDistOnElement;	// Positions on elements that are closest to each other
		ROT_ANGLE_TYPE	Angle;						// Angle of connection
	};

	struct OBSTACLE_DATA
	{
			std::vector<CONNECTION_DATA>	ConnectedObstacles;
			POS_2D							Position;
	};
}

typedef std::vector<OBSTACLE_CONNECTIONS::OBSTACLE_DATA>	MULTIPLE_OBSTACLES;

class ObstacleConnections
{
	public:
		ObstacleConnections();
		~ObstacleConnections();

		int CalculateAllConnections(const ObstacleMap &Obstacles);

		int CreateNewEmptyObstacles(const unsigned int &NumNewObstacles);			// Delete old data and set new size

		unsigned int GetNumObstacles() const;		// Return number of obstacles, map edges, start and end position
		int GetObstaclePos(const OBSTACLE_ID &ObstacleID, POS_2D &Position) const;		// Returns obstacle position
		unsigned int GetNumObstacleConnections(const OBSTACLE_ID &ObstacleID) const;			// Returns number of adjacent obstacles

		int GetConnectionPos(const OBSTACLE_ID &ObstacleID, const OBSTACLE_ID &ConnectedObstacleID, POS_2D &ConnectionPos) const;		// Returns point where two obstacles are connected

		OBSTACLE_ID GetNextObstacleInRotDir(const OBSTACLE_ID &ObstacleID, const OBSTACLE_ID &CurConnectedObstacleID, const bool &RotationDirection) const;		// Returns ID of next connected obstacle that is along the given rotational direction (1: CCW, 0:CW)
		OBSTACLE_ID GetNextObstacleInRotDir(const OBSTACLE_ID &ObstacleID, const ROT_ANGLE_TYPE &Angle, const bool &RotationDirection) const;		// Returns ID of next connected obstacle that is along the given rotational direction (1: CCW, 0:CW))

		int SetObstaclePos(const OBSTACLE_ID &ObstacleID, const POS_2D &Position);		// Set position of obstacle
		int AddConnectedObstacle(const OBSTACLE_ID &ObstacleID, const OBSTACLE_ID &ConnectedObstacleID, const POS_2D &ConnectionPos);			// Add obstacle to list and calculate angle

		int RemoveConnectedObstacle(const OBSTACLE_ID &ObstacleID, const OBSTACLE_ID &ConnectedObstacleID);		// Remove obstacle from list

		// Bracket operator to access obstacle data
		const OBSTACLE_CONNECTIONS::OBSTACLE_DATA &operator[](OBSTACLE_ID ObstacleID) const { return _Obstacles[ObstacleID]; }

	private:

		MULTIPLE_OBSTACLES _Obstacles;

		unsigned int AddConnectedObstacle_OneSide(const OBSTACLE_ID &ObstacleID, const OBSTACLE_ID &ConnectedObstacleID, const POS_2D &ConnectionPos);		// Function to connect one object to another. Call it for both IDs to register the connections for both objects

		int CalculateMinDistancePositions(const void * const IdDistMap);		// Calculate Minimum distance positions between obstacles
		int CalculateSingleMinDistPosition(const OBSTACLE_ID &ObstacleID, const void * const IdDistMap, OBSTACLE_CONNECTIONS::CONNECTION_DATA &ConnectionData);		// Calculates min dist position of two obstacles (requires obstacle ID and position of second obstacle in vector)
};

#endif // OBSTACLECONNECTIONS_H
