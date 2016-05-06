#include "obstacle_map.h"
#include <queue>

ObstacleMap::ObstacleMap()
{
}

void ObstacleMap::ResetMap()
{
	this->_ObstacleMap.ResetMap(OBSTACLE_MAP::EmptyID);		// Set all positions to empty
	this->_ObstaclePositions.clear();						// Remove all obstacle positions
}

void ObstacleMap::ResetMap(const Map<OCCUPANCYGRID_DISCRETE_TYPE> &NewObstacleMap)
{
	// Reset map first
	this->_ObstacleMap.ResetMap(NewObstacleMap.GetHeight(), NewObstacleMap.GetHeight(), OBSTACLE_MAP::EmptyID);

	// Remove all stored obstacles
	this->_ObstaclePositions.clear();

	// Go through given obstacle map and find obstacles
	for(unsigned int i=0; i<NewObstacleMap.GetHeight(); i++)
	{
		for(unsigned int j=0; j<NewObstacleMap.GetWidth(); j++)
		{
			const POS_2D curPos(j,i);
			if(NewObstacleMap.GetPixel(curPos) == OCCUPANCYGRID_DISCRETE_FULL)			// Check for obstacle
			{
				// if full, add an obstacle here
				this->AddObstaclePos(curPos);
			}
		}
	}
}

int ObstacleMap::AddObstaclePos(const POS_2D &NewPosition)
{
	// Check surroundings to see if this position is next to a given obstacle
	POS_2D curAdjacentPos;
	unsigned int adjacentID = OBSTACLE_MAP::EmptyID;
	unsigned int tmpID;

	// Check left
	curAdjacentPos.X = NewPosition.X-1;
	curAdjacentPos.Y = NewPosition.Y;
	if(this->_ObstacleMap.GetPixel(curAdjacentPos, tmpID) >= 0)
	{
		if(tmpID != OBSTACLE_MAP::EmptyID)
		{
			adjacentID = tmpID;
		}
	}

	// Check right
	curAdjacentPos.X = NewPosition.X+1;
	curAdjacentPos.Y = NewPosition.Y;
	if(this->_ObstacleMap.GetPixel(curAdjacentPos, tmpID) >= 0)
	{
		if(tmpID != OBSTACLE_MAP::EmptyID)
		{
			if(adjacentID != OBSTACLE_MAP::EmptyID && adjacentID != tmpID)
			{
				// if another adjacent position contains a different ID, combine the two
				this->_ObstacleMap.SetPixel(NewPosition, adjacentID);

				this->CombineTwoIDs(adjacentID, tmpID);		// Replaces all positions with tmpID with adjacentID
			}
			else
				adjacentID = tmpID;
		}
	}

	// Check top
	curAdjacentPos.X = NewPosition.X;
	curAdjacentPos.Y = NewPosition.Y+1;
	if(this->_ObstacleMap.GetPixel(curAdjacentPos, tmpID) >= 0)
	{
		if(tmpID != OBSTACLE_MAP::EmptyID)
		{
			if(adjacentID != OBSTACLE_MAP::EmptyID && adjacentID != tmpID)
			{
				// if another adjacent position contains a different ID, combine the two
				this->_ObstacleMap.SetPixel(NewPosition, adjacentID);

				this->CombineTwoIDs(adjacentID, tmpID);		// Replaces all positions with tmpID with adjacentID
			}
			else
				adjacentID = tmpID;
		}
	}

	// Check bottom
	curAdjacentPos.X = NewPosition.X;
	curAdjacentPos.Y = NewPosition.Y-1;
	if(this->_ObstacleMap.GetPixel(curAdjacentPos, tmpID) >= 0)
	{
		if(tmpID != OBSTACLE_MAP::EmptyID)
		{
			if(adjacentID != OBSTACLE_MAP::EmptyID && adjacentID != tmpID)
			{
				// if another adjacent position contains a different ID, combine the two
				this->_ObstacleMap.SetPixel(NewPosition, adjacentID);

				this->CombineTwoIDs(adjacentID, tmpID);		// Replaces all positions with tmpID with adjacentID
			}
			else
				adjacentID = tmpID;
		}
	}

	if(adjacentID == OBSTACLE_MAP::EmptyID)
	{
		// Register new obstacle if not yet connected to old ones
		this->_ObstaclePositions.push_back(NewPosition);
		this->_ObstacleMap.SetPixel(NewPosition, this->_ObstaclePositions.size()-1);

		return 1;
	}
	else
	{
		// This object is connected to others, that is already handled above
		return 0;
	}
}

unsigned int ObstacleMap::GetNumObstacles() const
{
	return this->_ObstaclePositions.size();
}

void ObstacleMap::CombineTwoIDs(const unsigned int &OriginalID, const unsigned int &IDToCombine)
{
	// goes through all positions of IDToCombine and replaces those IDs with OriginalID

	// Queue for storing next positions to check
	OBSTACLE_MAP::POS_TO_CHECK_TYPE posToCheck;

	// Start at stored IDToCombine position
	posToCheck.push(this->_ObstaclePositions[IDToCombine]);

	do
	{
		this->CombineTwoIDs_Step(OriginalID, IDToCombine, posToCheck);		// Continue going through adjacent positions until all neighboring positions are checked
	}
	while(posToCheck.size() > 0);
}

void ObstacleMap::CombineTwoIDs_Step(const unsigned int &OriginalID, const unsigned int &IDToCombine, OBSTACLE_MAP::POS_TO_CHECK_TYPE &PosToCheck)
{
	unsigned int tmpID;
	POS_2D tmpPos;

	// Get current position
	const POS_2D &curPos = PosToCheck.front();
	PosToCheck.pop();		// remove element from queue

	// Check if it is already changed
	if(this->_ObstacleMap.GetPixel(curPos) == OriginalID)
	{
		return;
	}

	// Set element
	this->_ObstacleMap.SetPixel(curPos, OriginalID);

	// Check surroundings

	// Check left
	tmpPos.X = curPos.X-1;
	tmpPos.Y = curPos.Y;
	if(this->_ObstacleMap.GetPixel(tmpPos, tmpID) > 0)
	{
		if(tmpID == IDToCombine)
			PosToCheck.push(tmpPos);		// Check this position as well
	}

	// Check right
	tmpPos.X = curPos.X+1;
	tmpPos.Y = curPos.Y;
	if(this->_ObstacleMap.GetPixel(tmpPos, tmpID) > 0)
	{
		if(tmpID == IDToCombine)
			PosToCheck.push(tmpPos);		// Check this position as well
	}

	// Check top
	tmpPos.X = curPos.X;
	tmpPos.Y = curPos.Y+1;
	if(this->_ObstacleMap.GetPixel(tmpPos, tmpID) > 0)
	{
		if(tmpID == IDToCombine)
			PosToCheck.push(tmpPos);		// Check this position as well
	}

	// Check bottom
	tmpPos.X = curPos.X;
	tmpPos.Y = curPos.Y-1;
	if(this->_ObstacleMap.GetPixel(tmpPos, tmpID) > 0)
	{
		if(tmpID == IDToCombine)
			PosToCheck.push(tmpPos);		// Check this position as well
	}

	return;
}
