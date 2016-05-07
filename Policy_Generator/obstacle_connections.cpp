#include "obstacle_connections.h"
#include "obstacle_path_finder.h"
#include "map_height_map.h"
#include "robot_navigation.h"

#include <cmath>

ObstacleConnections::ObstacleConnections()
{
}

ObstacleConnections::~ObstacleConnections()
{

}

int ObstacleConnections::CreateNewEmptyObstacles(const unsigned int &NumNewObstacles)
{
	// Check that either both or no start/end pos are given
	//if((StartPos == 0 && DestinationPos != 0) ||
	//   (StartPos != 0 && DestinationPos == 0))
	//	return -1;

	// Empty previous data
	this->_Obstacles.clear();

	// Calculate how many obstacles are here in total
	unsigned int numTotalObstacles = NumNewObstacles;
	//if(StartPos != 0 && DestinationPos != 0)
	//	numTotalObstacles +=2;

	// Make enough room for new obstacles
	this->_Obstacles.resize(numTotalObstacles);

	/*if(StartPos != 0 && DestinationPos != 0)
	{
		// Set start and end positions
		this->StartPosID = this->Obstacles.size()-2;
		this->DestPosID = this->Obstacles.size()-1;

		this->SetObstaclePos(this->StartPosID, *StartPos);
		this->SetObstaclePos(this->DestPosID, *DestinationPos);
	}
	else
	{
		// Set both start and end positions as 0
		this->StartPosID = 0;
		this->DestPosID = 0;
	}*/

	return 1;
}

//void ObstacleConnections::DeleteObstacle_DataOnly(const OBSTACLE_ID &ObstacleID)
//{
//	OBSTACLE_CONNECTIONS::OBSTACLE_DATA &curObstacle = this->_Obstacles[ObstacleID];
//
//	curObstacle.ConnectedObstacles.clear();
//	curObstacle.Position.X = 0;
//	curObstacle.Position.Y = 0;
//}

//OBSTACLE_ID ObstacleConnections::GetStartID() const
//{
//	return this->StartPosID;
//}

//OBSTACLE_ID ObstacleConnections::GetDestinationID() const
//{
//	return this->DestPosID;
//}

unsigned int ObstacleConnections::GetNumObstacles() const
{
	return this->_Obstacles.size();
}

int ObstacleConnections::GetObstaclePos(const OBSTACLE_ID &ObstacleID, POS_2D &Position) const
{
	if(ObstacleID >= this->_Obstacles.size())
		return -1;

	Position = this->_Obstacles[ObstacleID].Position;

	return 1;
}

unsigned int ObstacleConnections::GetNumObstacleConnections(const OBSTACLE_ID &ObstacleID) const
{
	if(ObstacleID >= this->_Obstacles.size())
		return 0;

	return this->_Obstacles[ObstacleID].ConnectedObstacles.size();
}

int ObstacleConnections::GetConnectionPos(const OBSTACLE_ID &ObstacleID, const OBSTACLE_ID &ConnectedObstacleID, POS_2D &ConnectionPos) const
{
	return -1;
}

OBSTACLE_ID ObstacleConnections::GetNextObstacleInRotDir(const OBSTACLE_ID &ObstacleID, const OBSTACLE_ID &CurConnectedObstacleID, const bool &RotationDirection) const
{
	// Find next obstacle position
	unsigned int nextObstaclePos = 0;

	const OBSTACLE_CONNECTIONS::OBSTACLE_DATA &curObstacle = this->_Obstacles[ObstacleID];

	for(unsigned int i=0; i<curObstacle.ConnectedObstacles.size(); i++)
	{
		if(curObstacle.ConnectedObstacles[i].ConnectedObstacleID == CurConnectedObstacleID)
		{
			if(RotationDirection)
			{
				nextObstaclePos = i+1;

				// Circle to start
				if(nextObstaclePos >= curObstacle.ConnectedObstacles.size())
					nextObstaclePos = 0;
			}
			else
			{
				if(i==0)
					nextObstaclePos = curObstacle.ConnectedObstacles.size()-1;
				else
					nextObstaclePos = i-1;
			}

			break;
		}
	}

	// Return ID of next obstacle
	return curObstacle.ConnectedObstacles[nextObstaclePos].ConnectedObstacleID;
}

OBSTACLE_ID ObstacleConnections::GetNextObstacleInRotDir(const OBSTACLE_ID &ObstacleID, const ROT_ANGLE_TYPE &Angle, const bool &RotationDirection) const
{
	// Set current angle to value between 0 and 2PI
	ROT_ANGLE_TYPE curAngle = Angle - floor((Angle/(M_PI*2.0)))*(M_PI*2.0);

	const OBSTACLE_CONNECTIONS::OBSTACLE_DATA &curObstacle = this->_Obstacles[ObstacleID];

	// Go through connected obstacles and find first higher angle
	for(unsigned int i=0; i<curObstacle.ConnectedObstacles.size(); i++)
	{
		// Find higher angle than current one
		if( curAngle < curObstacle.ConnectedObstacles[i].ConnectedObstacleID )
		{
			if(RotationDirection)
			{
				// Return the next angle in CCW
				return curObstacle.ConnectedObstacles[i].ConnectedObstacleID;
			}
			else
			{
				// Return the next angle in CW
				if(i==0)
					return curObstacle.ConnectedObstacles[curObstacle.ConnectedObstacles.size()-1].ConnectedObstacleID;

				return curObstacle.ConnectedObstacles[i-1].ConnectedObstacleID;
			}
		}
	}

	// If no higher angle was found, return end() for CW and 0 for CCW
	if(RotationDirection)
		return curObstacle.ConnectedObstacles[0].ConnectedObstacleID;
	else
		return curObstacle.ConnectedObstacles[curObstacle.ConnectedObstacles.size()-1].ConnectedObstacleID;
}

int ObstacleConnections::SetObstaclePos(const OBSTACLE_ID &ObstacleID, const POS_2D &Position)
{
	if(ObstacleID >= this->_Obstacles.size())
		return -1;

	this->_Obstacles[ObstacleID].Position = Position;

	return 1;
}

int ObstacleConnections::AddConnectedObstacle(const OBSTACLE_ID &ObstacleID, const OBSTACLE_ID &ConnectedObstacleID, const POS_2D &ConnectionPos)
{
	if(ObstacleID >= this->_Obstacles.size() || ConnectedObstacleID >= this->_Obstacles.size())
		return -1;

	// Connect one side
	this->AddConnectedObstacle_OneSide(ObstacleID, ConnectedObstacleID, ConnectionPos);

	// Also add it to other obstacle
	this->AddConnectedObstacle_OneSide(ConnectedObstacleID, ObstacleID, ConnectionPos);

	return 1;
}

int ObstacleConnections::RemoveConnectedObstacle(const OBSTACLE_ID &ObstacleID, const OBSTACLE_ID &ConnectedObstacleID)
{
	// Check whether these two objects are already connected
	for(unsigned int i=0; i<this->_Obstacles[ObstacleID].ConnectedObstacles.size(); i++)
	{
		if(ConnectedObstacleID == this->_Obstacles[ObstacleID].ConnectedObstacles[i].ConnectedObstacleID)
		{
			// Delete current connection
			this->_Obstacles[ObstacleID].ConnectedObstacles.erase(this->_Obstacles[ObstacleID].ConnectedObstacles.begin()+i);

			break;
		}
	}

	// Do the same for the other Obstacle
	for(unsigned int i=0; i<this->_Obstacles[ConnectedObstacleID].ConnectedObstacles.size(); i++)
	{
		if(ObstacleID == this->_Obstacles[ConnectedObstacleID].ConnectedObstacles[i].ConnectedObstacleID)
		{
			// Delete current connection
			this->_Obstacles[ConnectedObstacleID].ConnectedObstacles.erase(this->_Obstacles[ConnectedObstacleID].ConnectedObstacles.begin()+i);

			break;
		}
	}

	return 1;
}

unsigned int ObstacleConnections::AddConnectedObstacle_OneSide(const OBSTACLE_ID &ObstacleID, const OBSTACLE_ID &ConnectedObstacleID, const POS_2D &ConnectionPos)
{
	unsigned int newConnectionPos;

	OBSTACLE_CONNECTIONS::CONNECTION_DATA newConnection;
	newConnection.ConnectedObstacleID	= ConnectedObstacleID;
	newConnection.MinDistPosition		= ConnectionPos;

	// Find angle to ConnectionPos
	newConnection.Angle = atan2(ConnectionPos.Y - this->_Obstacles[ObstacleID].Position.Y, ConnectionPos.X - this->_Obstacles[ObstacleID].Position.X);

	// Check whether these two objects are already connected
	for(unsigned int i=0; i<this->_Obstacles[ObstacleID].ConnectedObstacles.size(); i++)
	{
		if(ConnectedObstacleID == this->_Obstacles[ObstacleID].ConnectedObstacles[i].ConnectedObstacleID)
		{
			// Delete current connection
			this->_Obstacles[ObstacleID].ConnectedObstacles.erase(this->_Obstacles[ObstacleID].ConnectedObstacles.begin()+i);
			break;
		}
	}

	// Add data to correct position
	bool connectionInserted = false;
	for(unsigned int i=0; i<this->_Obstacles[ObstacleID].ConnectedObstacles.size(); i++)
	{
		// Check where to add new angle
		if(newConnection.Angle <= this->_Obstacles[ObstacleID].ConnectedObstacles[i].Angle)
		{
			// Add angle at this position
			this->_Obstacles[ObstacleID].ConnectedObstacles.insert(this->_Obstacles[ObstacleID].ConnectedObstacles.begin()+i, newConnection);

			connectionInserted = true;
			newConnectionPos = i;
			break;
		}
	}

	// If this is the largest angle, add it to end
	if(!connectionInserted)
	{
		newConnectionPos = this->_Obstacles.size();
		this->_Obstacles[ObstacleID].ConnectedObstacles.push_back(newConnection);
	}

	return newConnectionPos;
}

int ObstacleConnections::CalculateMinDistancePositions(const void * const IdDistMap)
{
	// Go through all obstacles
	for(unsigned int i=0; i<this->_Obstacles.size(); i++)
	{
		// Go through all connections of this obstacle
		for(unsigned int j=0; j<this->_Obstacles[i].ConnectedObstacles.size(); j++)
		{
			// Calculate the minimum distance positions for these connections
			if(this->CalculateSingleMinDistPosition(i, IdDistMap, this->_Obstacles[i].ConnectedObstacles[j]) < 0)
				return -1;
		}
	}

	return 1;
}

int ObstacleConnections::CalculateSingleMinDistPosition(const OBSTACLE_ID &ObstacleID, const void * const IdDistMap, OBSTACLE_CONNECTIONS::CONNECTION_DATA &ConnectionData)
{
	const Map<OBSTACLE_PATH_FINDER::MAP_ID_DIST> * const pIdDistMap = static_cast<const Map<OBSTACLE_PATH_FINDER::MAP_ID_DIST>*const>(IdDistMap);
	OBSTACLE_PATH_FINDER::MAP_ID_DIST tmpData;

	// Start at middle between both elements
	POS_2D curPos = ConnectionData.MinDistPosition;
	POS_2D adjacentPos;

	// Get ID at this position
	tmpData = pIdDistMap->GetPixel(curPos);
	OBSTACLE_ID curID = tmpData.ID;

	// Get adjacent ID (go through adjacent positions and find different ID)
	OBSTACLE_ID adjacentID = curID;
	for(unsigned int i=0; i<RobotNavigation::GetNumNextMovementPositions(); i++)
	{
		RobotNavigation::GetNextMovementPosition(curPos, i, adjacentPos);

		if(pIdDistMap->GetPixel(adjacentPos, tmpData) >= 0)
		{
			if(tmpData.ID != curID)
			{
				adjacentID = curID;
				break;
			}
		}
	}

	// Store it in first position if position ID equals ObstacleID
	// This way, the minimum position of a given obstacle is always in first array element
	if(curID == ObstacleID)
	{
		if(HeightMap::FindMinHeightPos((*pIdDistMap), curPos, ConnectionData.MinDistOnElement[0]) < 0)
			return -1;
		if(HeightMap::FindMinHeightPos((*pIdDistMap), adjacentPos, ConnectionData.MinDistOnElement[1]) < 0)
			return -2;
	}
	else
	{
		if(HeightMap::FindMinHeightPos((*pIdDistMap), adjacentPos, ConnectionData.MinDistOnElement[0]) < 0)
			return -3;
		if(HeightMap::FindMinHeightPos((*pIdDistMap), curPos, ConnectionData.MinDistOnElement[1]) < 0)
			return -4;
	}

	return 1;
}
