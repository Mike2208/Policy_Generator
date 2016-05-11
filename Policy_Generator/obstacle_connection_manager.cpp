#include "obstacle_connection_manager.h"
#include "robot_navigation.h"

ObstacleConnectionManager::ObstacleConnectionManager()
{

}

void ObstacleConnectionManager::Reset()
{
	this->_ConnectionData.clear();
}

int ObstacleConnectionManager::CalculateAllConnections(const ObstacleMap &MapData)
{
	// Erase previous data
	this->Reset();

	Map_ID idMap;				// Map to manage IDs
	Map<OBSTACLE_CONNECTION::DIST_TYPE> distMap;	// Map to store distances from obstacles

	// Enlarge obstacle until they collide
	return this->EnlargeObstaclesUntilCollision(MapData, idMap, distMap, this->_ConnectionData);
}

int ObstacleConnectionManager::EnlargeObstaclesUntilCollision(const ObstacleMap &MapData, Map_ID &IDMap, OBSTACLE_CONNECTION_MANAGER::DIST_MAP &DistMap, std::vector<ObstacleConnection> &Connections)
{
	// Initialization
	IDMap.ResetMap(MapData.GetMapHeight(), MapData.GetMapWidth(), OBSTACLE_ID_EMPTY);		// Set idMap to initial value
	DistMap.ResetMap(MapData.GetMapHeight(), MapData.GetMapWidth(), OBSTACLE_CONNECTION::MAX_DIST);				// Set distance to maximum
	Connections.clear();

	// Reserve connections for all obstacles + Movement options of robot for edges of map
	Connections.resize(MapData.GetNumObstacles()+RobotNavigation::GetNumNextMovementPositions());

	// Queue for all positions to check
	OBSTACLE_CONNECTION_MANAGER::POS_TO_CHECK posToCheck;

	// Add all obstacle points to this position
	for(OBSTACLE_ID i=0; i<MapData.GetNumObstacles(); i++)
	{
		POS_2D tmpPos;
		MapData.GetObstaclePosition(i, tmpPos);
		posToCheck.push(tmpPos);
	}

	// Go through queue until all positions have been checked
	while(posToCheck.size() > 0)
	{
		this->EnlargeObstaclesUntilCollision_Step(MapData, posToCheck.front(), IDMap, DistMap, posToCheck, this->_ConnectionData);

		posToCheck.pop();
	}

	return 1;
}

void ObstacleConnectionManager::EnlargeObstaclesUntilCollision_Step(const ObstacleMap &MapData, const POS_2D &CurPos, Map_ID &IDMap, OBSTACLE_CONNECTION_MANAGER::DIST_MAP &DistMap, OBSTACLE_CONNECTION_MANAGER::POS_TO_CHECK &PosToCheck, OBSTACLE_CONNECTION_MANAGER::OBSTACLE_CONNECTIONS &Connections)
{

	OBSTACLE_ID adjacentID, curID;
	OBSTACLE_CONNECTION::DIST_TYPE adjacentDist, curDist;

	// Get data of current position
	IDMap.GetPixel(CurPos, curID);
	curDist = DistMap.GetPixel(CurPos);

	// Go through all adjacent positions
	for(unsigned int i=0; i<RobotNavigation::GetNumNextMovementPositions(); i++)
	{
		// Get adjacent position
		const POS_2D adjacentPos = RobotNavigation::GetNextMovementPosition(CurPos, i);

		DistMap.GetPixel(adjacentPos, adjacentDist);

		// Get OBSTACLEMAP adjacent ID to check whether adjacentPos is in map
		if(MapData.GetIDatPosition(adjacentPos, adjacentID) < 0)
		{
			// If adjacent ID couldn't be found, add current position to edge of map (only way for this error to occur is when leaving map)
			Connections[curID].UpdateConnection(Connections.size()-1-i, CurPos, curDist);

			return;
		}
		else if(curDist == 0 && adjacentID == curID && adjacentDist == OBSTACLE_CONNECTION::MAX_DIST)
		{
			// If a position was found that should be in an obstacle and hasn't been checked yet, set its distance to zero
			IDMap.SetPixel(adjacentPos, curID);
			DistMap.SetPixel(adjacentPos, 0);

			// Check this position immediately to prevent errors later on (e.g. another obstacle reaching this position first)
			this->EnlargeObstaclesUntilCollision_Step(MapData, adjacentPos, IDMap, DistMap, PosToCheck, Connections);

			return;
		}

		// Get ID that is associated with this position in IDMap
		IDMap.GetPixel(adjacentPos, adjacentID);

		// If adjacent distance is larger than current distance+1, a faster path was found
		if(adjacentDist > curDist + 1)
		{
			// Set both adjacent ID and distance to current one and add this position to be checked
			IDMap.SetPixel(adjacentPos, curID);
			DistMap.SetPixel(adjacentPos, curDist+1);

			PosToCheck.push(adjacentPos);
		}
		else if(adjacentID != curID && (adjacentDist==curDist || adjacentDist == curDist+1))
		{
			// Add this position to connection data
			if(Connections[curID].UpdateConnection(adjacentID, adjacentPos, adjacentDist) > 0)
			{
				// if an update was necessary, also add it to opposite ID
				Connections[adjacentID].UpdateConnection(curID, adjacentPos, adjacentDist);
			}
		}
	}

	return;
}
