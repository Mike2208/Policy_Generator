#include "obstacle_map.h"
#include <queue>

ObstacleMap::ObstacleMap()
{
}

void ObstacleMap::FindAllObstacles(const OGM_MAP &MapData, const OGM_TYPE &Threshold)
{
	// Remove previous data
	this->_ObstacleMap.ResetMap(MapData.GetHeight(), MapData.GetWidth(), OBSTACLE_ID_EMPTY);
	this->_ObstaclePositions.clear();

	OBSTACLE_ID prevObstacleID = OBSTACLE_ID_EMPTY;

	// Go through map from bottom left to top right
	for(unsigned int y=0; y<MapData.GetHeight(); y++)
	{
		// when changing lines, make
		prevObstacleID = OBSTACLE_ID_EMPTY;

		for(unsigned int x=0; x<MapData.GetWidth(); x++)
		{
			// Get current obstacle value
			const OGM_TYPE curValue = MapData.GetPixel(x,y);

			// Check if this probability is high enough to be an obstacle
			if(curValue >= Threshold)
			{
				if(prevObstacleID == OBSTACLE_ID_EMPTY)
				{
					// Create new obstacle
					prevObstacleID = static_cast<OBSTACLE_ID>(this->_ObstaclePositions.size());			// Set new ID as previous obstacle here
					this->_ObstacleMap.SetPixel(x,y, static_cast<OBSTACLE_ID>(this->_ObstaclePositions.size()));
					this->_ObstaclePositions.push_back(POS_2D(x,y));

				}
				else
				{
					// Add this to previous obstacle
					this->_ObstacleMap.SetPixel(x,y, prevObstacleID);
				}

				// Check value below this one
				OBSTACLE_ID adjacentID;
				if(this->_ObstacleMap.GetPixel(x,y-1, adjacentID) >= 0)
				{
					// If lower position is valid, check value
					if(adjacentID != OBSTACLE_ID_EMPTY)
					{
						// Combine both values
						this->CombineTwoIDs(adjacentID, prevObstacleID);

						prevObstacleID = adjacentID;
					}
				}
			}
			else
			{
				// No longer in an obstacle
				prevObstacleID = OBSTACLE_ID_EMPTY;
			}
		}
	}
}

int ObstacleMap::AddObstaclePos(const POS_2D &NewPosition)
{
	// Check surroundings to see if this position is next to a given obstacle
	POS_2D curAdjacentPos;
	OBSTACLE_ID adjacentID = OBSTACLE_ID_EMPTY;
	OBSTACLE_ID tmpID;

	// Check left
	curAdjacentPos.X = NewPosition.X-1;
	curAdjacentPos.Y = NewPosition.Y;
	if(this->_ObstacleMap.GetPixel(curAdjacentPos, tmpID) >= 0)
	{
		if(tmpID != OBSTACLE_ID_EMPTY)
		{
			adjacentID = tmpID;
		}
	}

	// Check right
	curAdjacentPos.X = NewPosition.X+1;
	curAdjacentPos.Y = NewPosition.Y;
	if(this->_ObstacleMap.GetPixel(curAdjacentPos, tmpID) >= 0)
	{
		if(tmpID != OBSTACLE_ID_EMPTY)
		{
			if(adjacentID != OBSTACLE_ID_EMPTY && adjacentID != tmpID)
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
		if(tmpID != OBSTACLE_ID_EMPTY)
		{
			if(adjacentID != OBSTACLE_ID_EMPTY && adjacentID != tmpID)
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
		if(tmpID != OBSTACLE_ID_EMPTY)
		{
			if(adjacentID != OBSTACLE_ID_EMPTY && adjacentID != tmpID)
			{
				// if another adjacent position contains a different ID, combine the two
				this->_ObstacleMap.SetPixel(NewPosition, adjacentID);

				this->CombineTwoIDs(adjacentID, tmpID);		// Replaces all positions with tmpID with adjacentID
			}
			else
				adjacentID = tmpID;
		}
	}

	if(adjacentID == OBSTACLE_ID_EMPTY)
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

void ObstacleMap::CombineTwoIDs(const OBSTACLE_ID &OriginalID, const OBSTACLE_ID &IDToCombine)
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

void ObstacleMap::CombineTwoIDs_Step(const OBSTACLE_ID &OriginalID, const OBSTACLE_ID &IDToCombine, OBSTACLE_MAP::POS_TO_CHECK_TYPE &PosToCheck)
{
	OBSTACLE_ID tmpID;
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
