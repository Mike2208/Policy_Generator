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

		int AddConnection(const CONNECTION_DATA &NewData);

	private:

		OBSTACLE_ID _ID;													// ID of this obstacle
		POS_2D		_CenterPos;												// Center of this current obstacle
		std::vector<CONNECTION_DATA> _Connections;							// Connections to other obstacles

};

#endif // OBSTACLECONNECTION_H
