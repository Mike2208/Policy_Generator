#include "obstacle_connection.h"
#include "obstacle_path_finder.h"
#include "map_height_map.h"
#include "robot_navigation.h"

#include <cmath>

ObstacleConnection::ObstacleConnection() : _ConnectionsSorted(true)
{
}

ObstacleConnection::~ObstacleConnection()
{

}

void ObstacleConnection::Reset()
{
	this->_ID = OBSTACLE_ID_EMPTY;
	this->_Connections.clear();
	this->_ConnectionsSorted = true;
}

int ObstacleConnection::SortConnections(const ObstacleMap &Obstacles)
{
	this->_ConnectionsSorted = false;
	std::vector<CONNECTION_DATA> SortedData;
	std::vector<CONNECTION_DATA> tmpData = this->_Connections;
	SortedData.push_back(this->_Connections[0]);
	tmpData.erase(tmpData.begin());

	ROT_ANGLE_TYPE rotation = 0;		// keeps track of rotation (if it is -2PI, reverse vector, else keep it)

	// Set start position as first connection position
	const POS_2D startPos = this->_Connections[0].MinStartPos;
	POS_2D curPos = this->SortGetNextObstacleEdge(Obstacles, this->_ID, startPos);
	POS_2D prevPos = startPos;

	while(curPos != startPos)
	{
		if(prevPos == curPos)
			return -1;		// sorting error, couldnt find complete edge

		// check if next position is part of tmpData
		for(unsigned int i=0; i<tmpData.size(); i++)
		{
			if(tmpData[i].MinStartPos == curPos)
			{
				SortedData.push_back(tmpData[i]);
				break;
			}
		}


	}
}

POS_2D ObstacleConnection::SortGetNextObstacleEdge(const ObstacleMap &Obstacles, const OBSTACLE_ID &ObstacleID, const POS_2D &CurPos)
{
	OBSTACLE_ID curID;

	// Get all adjacent positions
	for(unsigned int i=0; i<RobotNavigation::GetNumNextMovementPositions(); i++)
	{
		const POS_2D adjacentPos = RobotNavigation::GetNextMovementPosition(CurPos, i);

		if(Obstacles.GetPosID(adjacentPos, curID) < 0)
			break;
		if(curID != ObstacleID)
			break;

		// Check for a free position next to this one to see whether adjacent position is
		for(unsigned int j=0; j<RobotNavigation::GetNumNextMovementPositions(); j++)
		{
			const POS_2D freePos = RobotNavigation::GetNextMovementPosition(adjacentPos, i);
			if(Obstacles.GetPosID(freePos, curID) >= 0)
			{
				// Check whether adjacentPos is an edge
				if(curID != ObstacleID)
					return adjacentPos;
			}
		}
	}

	// return CurPos to state that no new positions where found
	return CurPos;
}
