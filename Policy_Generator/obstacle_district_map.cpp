#include "obstacle_district_map.h"
#include "robot_navigation.h"
#include <queue>

ObstacleDistrictMap::ObstacleDistrictMap()
{

}

void ObstacleDistrictMap::Reset()
{
	this->_IDMap.ResetMap(OBSTACLE_DISTRICT_MAP::EmptyID);
	this->_IDPositions.clear();
}

int ObstacleDistrictMap::CalculateAllDistrictAreas(const ObstacleMap &ObstacleData, const ObstacleConnectionManager &Connections)
{
	// Remove previous data
	this->Reset();

	// Set _IDMap to correct size
	this->_IDMap.ResetMap(ObstacleData.GetMapHeight(), ObstacleData.GetMapWidth(), OBSTACLE_DISTRICT_MAP::EmptyID);
	this->_IDPositions.resize(ObstacleData.GetNumObstacles());

	// Store all obstacle data in _IDMap and save positions in _IDpositions
	for(unsigned int i=0; i<ObstacleData.GetNumObstacles(); i++)
	{
		this->AddObstacleToMap(ObstacleData, i);
		ObstacleData.GetObstaclePosition(i, this->_IDPositions[i]);
	}

	// Calculate all districts between free spaces that are divided by Connections
	OBSTACLE_ID nextDistrictID = ObstacleData.GetNumObstacles();
	OBSTACLE_ID curPosID;

	// Add dividers between districts
	this->AddDistrictDividers(ObstacleData, Connections, OBSTACLE_DISTRICT_MAP::DistrictID);

	// Go through map and find positions with free IDs
	for(POS_2D_TYPE X = 0; X<ObstacleData.GetMapWidth(); X++)
	{
		for(POS_2D_TYPE Y = 0; Y<ObstacleData.GetMapHeight(); Y++)
		{
			ObstacleData.GetIDatPosition(POS_2D(X,Y), curPosID);

			if(curPosID == OBSTACLE_DISTRICT_MAP::EmptyID)
			{
				// Set this district to correct ID
				this->SetDistrictToID(POS_2D(X,Y), OBSTACLE_DISTRICT_MAP::EmptyID, OBSTACLE_DISTRICT_MAP::DistrictID, nextDistrictID, this->_IDMap);

				// Save start position
				this->_IDPositions.push_back(POS_2D(X,Y));

				nextDistrictID++;
			}
		}
	}
}

int ObstacleDistrictMap::AddObstacleToMap(const ObstacleMap &ObstacleData, const OBSTACLE_ID &ObstacleID)
{
	std::queue<POS_2D> posToCheck;
	OBSTACLE_ID adjacentID;

	// Get start position
	POS_2D curPos, adjacentPos;
	if(ObstacleData.GetObstaclePosition(ObstacleID, curPos) < 0)
		return -1;

	// Add this position to positions to be checked
	posToCheck.push(curPos);

	// Set obstacle ID on map
	this->_IDMap.SetPixel(curPos, ObstacleID);

	do
	{
		// Check all neighboring ones
		for(unsigned int i=0; i<RobotNavigation::GetNumNextMovementPositions(); i++)
		{
			adjacentPos = RobotNavigation::GetNextMovementPosition(curPos, i);

			if(ObstacleData.GetIDatPosition(adjacentPos, adjacentID) >= 0)
			{
				if(adjacentID == ObstacleID)
				{
					// if adjacent position also belongs to obstacle, include it and check next adjacent positions
					this->_IDMap.SetPixel(adjacentPos, ObstacleID);

					posToCheck.push(adjacentPos);
				}
			}
		}

		// Get next position to check
		curPos = posToCheck.front();
		posToCheck.pop();
	}
	while(posToCheck.size() > 0);

	return 1;
}

int ObstacleDistrictMap::AddDistrictDividers(const ObstacleMap &ObstacleData, const ObstacleConnectionManager &Connections, const OBSTACLE_ID &DividerID)
{
	CONNECTION_DATA curConnection;

	// Go through all obstacles
	for(unsigned int i=0; i<ObstacleData.GetNumObstacles(); i++)
	{
		// Go through all connections and set pixels to DistrictID
		for(unsigned int j=0; j<Connections.GetNumConnectionsToObstacle(i); j++)
		{
			// Get connection data
			if(Connections.GetConnectionData(i,j, curConnection) < 0)
				return -1;

			// Set path from MinStartPos to CenterPos and from CenterPos to MinDestPos to DividerID
			this->SetPathToID(curConnection.MinStartPos, curConnection.MinCenterPos, DividerID, this->_IDMap);
			this->SetPathToID(curConnection.MinCenterPos, curConnection.MinDestPos, DividerID, this->_IDMap);
		}
	}

	return 1;
}

int ObstacleDistrictMap::SetPathToID(const POS_2D &StartPos, const POS_2D &DestPos, const OBSTACLE_ID &ID, Map_ID &Map)
{
	return Map.SetPath(StartPos, DestPos, ID);
}

void ObstacleDistrictMap::SetDistrictToID(const POS_2D &StartPos, const OBSTACLE_ID &IDtoReplace, const OBSTACLE_ID &DividerID, const OBSTACLE_ID &NewID, Map_ID &Map)
{
	std::queue<POS_2D> posToCheck;
	OBSTACLE_ID curID;

	// Start at StartPos
	posToCheck.push(StartPos);

	do
	{
		// Get pixel
		if(Map.GetPixel(posToCheck.front(), curID) >= 0)
		{
			if(curID == IDtoReplace || curID == DividerID)
			{
				// Replace ID
				Map.SetPixel(posToCheck.front(), NewID);

				// Add surrounding pixels to the ones that need to be checked
				for(unsigned int i=0; i<RobotNavigation::GetNumNextMovementPositions(); i++)
				{
					posToCheck.push(RobotNavigation::GetNextMovementPosition(posToCheck.front(), i));
				}
			}
		}
	}
	while(posToCheck.size() > 0);
}
