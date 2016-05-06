#ifndef OBSTACLECONNECTIONS_H
#define OBSTACLECONNECTIONS_H

/*	class ObstacleConnections
 *		stores which obstacles are close to each other
 */

#include "map_standards.h"
#include "map.h"

#include <vector>
#include <array>

namespace OBSTACLE_CONNECTIONS
{
	struct CONNECTION_DATA
	{
		unsigned int	ConnectedObstacleID;		// ID of connected obstacle
		POS_2D			MinDistPosition;			// Position of minimum distance between obstacles
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

		int CreateNewEmptyObstacles(const unsigned int &NumNewObstacles);			// Delete old data and set new size
		//void DeleteObstacle_DataOnly(const unsigned int &ObstacleID);				// Remove all connected obstacles and set position to (0,0)
		//void DeleteObstacle(const unsigned int &ObstacleID);						// Remove obstacle comlpetely

		//unsigned int GetStartID() const;			// Returns ID of start position
		//unsigned int GetDestinationID() const;		// Returns ID of destination

		unsigned int GetNumObstacles() const;		// Return number of obstacles, map edges, start and end position
		int GetObstaclePos(const unsigned int &ObstacleID, POS_2D &Position) const;		// Returns obstacle position
		unsigned int GetNumObstacleConnections(const unsigned int &ObstacleID) const;			// Returns number of adjacent obstacles

		int GetConnectionPos(const unsigned int &ObstacleID, const unsigned int &ConnectedObstacleID, POS_2D &ConnectionPos) const;		// Returns point where two obstacles are connected

		unsigned int GetNextObstacleInRotDir(const unsigned int &ObstacleID, const unsigned int &CurConnectedObstacleID, const bool &RotationDirection) const;		// Returns ID of next connected obstacle that is along the given rotational direction (1: CCW, 0:CW)
		unsigned int GetNextObstacleInRotDir(const unsigned int &ObstacleID, const ROT_ANGLE_TYPE &Angle, const bool &RotationDirection) const;		// Returns ID of next connected obstacle that is along the given rotational direction (1: CCW, 0:CW))

		int SetObstaclePos(const unsigned int &ObstacleID, const POS_2D &Position);		// Set position of obstacle
		int AddConnectedObstacle(const unsigned int &ObstacleID, const unsigned int &ConnectedObstacleID, const POS_2D &ConnectionPos);			// Add obstacle to list and calculate angle

		int RemoveConnectedObstacle(const unsigned int &ObstacleID, const unsigned int &ConnectedObstacleID);		// Remove obstacle from list

		// Bracket operator to access obstacle data
		const OBSTACLE_CONNECTIONS::OBSTACLE_DATA &operator[](unsigned int ObstacleID) const { return _Obstacles[ObstacleID]; }

	private:

		MULTIPLE_OBSTACLES _Obstacles;

		void AddConnectedObstacle_OneSide(const unsigned int &ObstacleID, const unsigned int &ConnectedObstacleID, const POS_2D &ConnectionPos);		// Function to connect one object to another. Call it for both IDs to register the connections for both objects

		int CalculateMinDistancePositions();		// Calculate Minimum distance positions between obstacles
		int CalculateSingleMinDistPosition(const unsigned int &ObstacleID, const OBSTACLE_CONNECTIONS::CONNECTION_DATA &ConnectionData, const void * const IdDistMap);		// Calculates min dist position of two obstacles (requires obstacle ID and position of second obstacle in vector)
};

#endif // OBSTACLECONNECTIONS_H
