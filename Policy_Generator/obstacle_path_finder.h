#ifndef OBSTACLE_PATH_FINDER_H
#define OBSTACLE_PATH_FINDER_H

/* class ObstaclePathFinder
 *	used to find all possible paths around obstacles
 *
 */

#include "map.h"
#include "map_standards.h"
#include "graph_class.h"
#include "obstacle_connections.h"

#include <vector>

namespace OBSTACLE_PATH_FINDER
{
	struct OBSTACLE_CONNECTION
	{
			unsigned int ObstacleID;
			unsigned int ConnectedObstacleID;
	};

	struct AVERAGE_OBSTACLE_POS
	{
			POS_2D AveragePos;
			unsigned int NumObstaclesInAverage;

			AVERAGE_OBSTACLE_POS() : AveragePos(0,0), NumObstaclesInAverage(0) {}
			AVERAGE_OBSTACLE_POS(const POS_2D &_AveragePos, const unsigned int _NumObstaclesInAverage) : AveragePos(_AveragePos), NumObstaclesInAverage(_NumObstaclesInAverage) {}
	};

	struct MAP_ID_DIST
	{
		unsigned int ID;			// ID associated with current position
		unsigned int Distance;		// Distance from obstacle

		MAP_ID_DIST() {}
		MAP_ID_DIST(const unsigned int &_ID, const unsigned int &_Distance) : ID(_ID),Distance(_Distance) {}

		// Comparators for distacne (mainly important for debugging, as it makes creating an image easier)
		bool operator>(const MAP_ID_DIST &Val) {return (this->Distance>Val.Distance);}
		bool operator<(const MAP_ID_DIST &Val) {return (this->Distance<Val.Distance);}
		bool operator==(const MAP_ID_DIST &Val) {return (this->Distance==Val.Distance);}

		bool operator==(const unsigned int &Val) {return (this->Distance==Val);}

		operator unsigned int() const { return this->Distance; }		// Return distance if asked for conversion (only used for debugging)
	};

	struct MAP_ID_DIST_POS : public MAP_ID_DIST			// Store Connected ID and Position
	{
		// Connected ID and distance to Position stored in inherited MAP_ID_DIST

		POS_2D Position;		// Position of minimum distance

		MAP_ID_DIST_POS() : MAP_ID_DIST() {}
		MAP_ID_DIST_POS(const unsigned int &_ID, const unsigned int &_Distance, const POS_2D &_Position) : MAP_ID_DIST::MAP_ID_DIST(_ID, _Distance), Position(_Position) {}
	};

	typedef std::vector<POS_2D> POSTOCHECK;
	typedef std::vector<std::vector<MAP_ID_DIST_POS>> CONNECTED_IDS;
}

namespace MAP_CONST_EDGE_IDS
{
	const unsigned int EmptyID = UINT_MAX;
	const unsigned int EmptyDist = UINT_MAX;

	// Number to add to Obstacle size() for other IDs
	const unsigned int TopID_Add = 0;
	const unsigned int LeftID_Add = 1;
	const unsigned int BottomID_Add = 2;
	const unsigned int RightID_Add = 3;
	const unsigned int StartID_Add = 4;
	const unsigned int DestID_Add = 5;


	const unsigned int NumExtraIDs = 6;
}

class ObstaclePathFinder
{
	public:
		ObstaclePathFinder();

		static int FindAllPaths(const Map_BoolType &ObstacleMap, const std::vector<POS_2D> &Obstacles, const POS_2D &Start, const POS_2D &Destination);
		static int FindAdjoiningObstacles(const Map_BoolType &ObstacleMap, const std::vector<POS_2D> &Obstacles, const POS_2D &Start, const POS_2D &Destination, ObstacleConnections &AdjoiningObstacles);

	private:

		// Helper functions to get all adjoining obstacles
		static inline void CompareOneAdjacentPosition(const Map_BoolType &ObstacleMap, Map<OBSTACLE_PATH_FINDER::MAP_ID_DIST> &IdDistMap, const OBSTACLE_PATH_FINDER::MAP_ID_DIST &CurIdDist, const OBSTACLE_PATH_FINDER::MAP_ID_DIST &AdjacentIdDist, const POS_2D &AdjacentPos, OBSTACLE_PATH_FINDER::CONNECTED_IDS &ConnectedIds, OBSTACLE_PATH_FINDER::POSTOCHECK &PosToCheck, std::vector<OBSTACLE_PATH_FINDER::AVERAGE_OBSTACLE_POS> &AverageObstaclePos);
		static inline void CompareAllAdjacentPositions(const Map_BoolType &ObstacleMap, Map<OBSTACLE_PATH_FINDER::MAP_ID_DIST> &DistMap, const POS_2D &CurPos, OBSTACLE_PATH_FINDER::CONNECTED_IDS &ConnectedIds, OBSTACLE_PATH_FINDER::POSTOCHECK &PosToCheck, std::vector<OBSTACLE_PATH_FINDER::AVERAGE_OBSTACLE_POS> &AverageObstaclePos);
		static inline void AddAdjacentBorder(const unsigned int &CurrentID, const OBSTACLE_PATH_FINDER::MAP_ID_DIST &AdjacentIdDist, const POS_2D &AdjacentPos, OBSTACLE_PATH_FINDER::CONNECTED_IDS &ConnectedIds);
		static inline void AddAdjacentEdge(const OBSTACLE_PATH_FINDER::MAP_ID_DIST &CurrentIdDist, const POS_2D &CurPos, const unsigned int &EdgeID, OBSTACLE_PATH_FINDER::CONNECTED_IDS &ConnectedIDs);

		// Helper function to calculate all paths from class ObstacleConnections
		static void FindNextPathStep(const ObstacleConnections &Connections, const bool CCW, const unsigned int &CurPathNum, std::vector<std::vector<OBSTACLE_PATH_FINDER::OBSTACLE_CONNECTION>> &CurPaths);
};

#endif // OBSTACLE_PATH_FINDER_H
