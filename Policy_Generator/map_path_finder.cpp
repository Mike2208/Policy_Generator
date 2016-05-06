#ifndef MAP_PATH_FINDER_CPP
#define MAP_PATH_FINDER_CPP

#include "map_path_finder.h"
#include <iostream>
#include <cmath>

// Function that compares two angles (returns true if they are closer than ROT_ANGLE_MAX_DIFFERENCE)
#define ROT_ANGLE_MAX_DIFFERENCE		0.0001
bool CompareAngles(ROT_ANGLE_TYPE Angle1, ROT_ANGLE_TYPE Angle2)
{
	if( fabs(Angle1 - Angle2) <= ROT_ANGLE_MAX_DIFFERENCE)
		return true;
	return false;
}

MapPathFinder::MapPathFinder()
{
}

int MapPathFinder::CalculateDistMap(const Map_BoolType &MainMap, const POS_2D &StartPos, Map_IntType &DistMap)
{
	const unsigned int height = MainMap.GetHeight();
	const unsigned int width = MainMap.GetWidth();

	unsigned int oldDist;
	unsigned int curDist = 0;
	unsigned int maxDist = 0;

	unsigned int curPosNum;
	std::vector<POS_2D> positionsToCheck;
	positionsToCheck.reserve(height*width);
	POS_2D tmpPos;

	// Setup DistMap to same Height/Width as MainMap
	//	Initialize all values to the maximum distance it can be from the starting point
	DistMap.ResetMap(height, width, height*width );

	// Set startPos
	DistMap.SetPixel(StartPos, 0);

	// Add StartPos to positions that need to be checked
	positionsToCheck.push_back(StartPos);
	curPosNum = 0;
	curDist = 0;

	// While there are still positions that haven't had their distances calculated, continue
	while(curPosNum < positionsToCheck.size())
	{
		// Get next position to check
		positionsToCheck[curPosNum];
		curDist = DistMap.GetPixel(positionsToCheck[curPosNum]);

		if(maxDist < curDist)
			maxDist = curDist;

		// Now set all adjacent values to curDist+1
		positionsToCheck[curPosNum].X--;
		positionsToCheck[curPosNum].Y--;
		for(int i = 0; i <= 2; i++)
		{
			for(int j = 0; j <= 2; j++)
			{
				// Check whether one of the adjacent positions is currently set as farther away
				tmpPos.X = positionsToCheck[curPosNum].X+i;
				tmpPos.Y = positionsToCheck[curPosNum].Y+j;

				if(DistMap.GetPixel(tmpPos, oldDist) < 0)
					continue;

				// Check if this position is even reachable
				if(MainMap.GetPixel(tmpPos) != OCCUPANCYGRID_DISCRETE_EMPTY)
					continue;

				if(oldDist > curDist+1)
				{
					// If yes, update this position with the smaller possible distance
					DistMap.SetPixel(tmpPos, curDist+1);

					// Add it to list of positions that need to be checked
					positionsToCheck.push_back(tmpPos);
				}
			}
		}

		// Update to check next value
		curPosNum++;
	}


	return maxDist;
}

int MapPathFinder::CalculatePath(const Map_BoolType &MainMap, const POS_2D &Start, const POS_2D &Destination, MapPath &Path)
{
	// Convert POS_2D into  POSE
	POSE startPose, endPose;
	startPose.Position = Start;
	startPose.RotationAngle = 0;
	endPose.Position = Destination;
	endPose.RotationAngle = 0;

	return MapPathFinder::CalculatePath(MainMap, startPose, endPose, Path);
}

int MapPathFinder::CalculatePath(const Map_BoolType &MainMap, const POSE &Start, const POSE &Destination, MapPath &Path)
{
	// Check that start is a valid position
	if( MainMap.GetPixel(Start.Position) != OCCUPANCYGRID_DISCRETE_EMPTY )
		return -1;

	// Get Distance Map
	Map_IntType distMap;
	MapPathFinder::CalculateDistMap(MainMap, Destination.Position, distMap);

	// Check if Destination is reachable from start
	unsigned int curDist, nextDist;
	if(distMap.GetPixel(Start.Position, curDist) < 0)
		return -2;
	if(curDist >= distMap.GetHeight()*distMap.GetWidth())
		return -3;


	// Follow negative gradient from start until destination is reached
	PATH_DATA_LOCAL newMoveOrder;
	POSE curPose = Start;
	POSE nextPose;

	newMoveOrder.PathLength		= 0;
	newMoveOrder.RotAngle		= 0;

	while(curDist > 0)
	{
		// Look at neighbors and find lowest distance to target
		nextDist = curDist;
		for(int i=-1; i<=1; i++)
		{
			for(int j=-1; j<=1; j++)
			{
				unsigned int	tmpDist;
				POS_2D			tmpPos;

				tmpPos.X = curPose.Position.X + i;
				tmpPos.Y = curPose.Position.Y + j;

				// Check that this pixel is valid
				if(distMap.GetPixel(tmpPos, tmpDist) < 0)
					continue;

				// Check if new distance is less than old distance and if so, set as next position
				if(tmpDist < nextDist)
				{
					nextDist = tmpDist;
					nextPose.Position = tmpPos;
				}
			}
		}

		// After going through neighbors, check that a lower distance was found
		//	Report error if not
		if(nextDist <= curDist)
		{
			std::cerr << "MapPathFinder::CalculatePath() ERROR: No lower distance found\n";
			return -4;
		}

		// Check whether old move order simply needs to be elongated, or if a new move order is required
		nextPose.RotationAngle = atan2((nextPose.Position.Y - curPose.Position.Y), (nextPose.Position.X - curPose.Position.X));
		if(CompareAngles(nextPose.RotationAngle, curPose.RotationAngle))
		{
			// Both angles are equal, just elongate current move order
			newMoveOrder.PathLength += sqrt((nextPose.Position.Y - curPose.Position.Y)*(nextPose.Position.Y - curPose.Position.Y) + (nextPose.Position.X-curPose.Position.X)*(nextPose.Position.X-curPose.Position.X));
		}
		else
		{
			// Add old order to Path
			Path.AddMoveOrder(newMoveOrder);

			// Create new order
			newMoveOrder.RotAngle		= nextPose.RotationAngle-curPose.RotationAngle;			// Rotate to new movement direction
			newMoveOrder.PathLength		= sqrt((nextPose.Position.Y - curPose.Position.Y)*(nextPose.Position.Y - curPose.Position.Y) + (nextPose.Position.X-curPose.Position.X)*(nextPose.Position.X-curPose.Position.X));
		}

		// Update for next round
		curPose = nextPose;
		curDist = nextDist;
	}

	// Check whether last move order needs to be added to Path
	if(newMoveOrder.PathLength != 0)
	{
		Path.AddMoveOrder(newMoveOrder);
	}

	// Check whether the robot needs to be rotated to face the correct direction
	if(curPose.RotationAngle != Destination.RotationAngle)
	{
		newMoveOrder.PathLength = 0;
		newMoveOrder.RotAngle   = Destination.RotationAngle - curPose.RotationAngle;

		Path.AddMoveOrder(newMoveOrder);
	}

	return 1;
}

#endif
