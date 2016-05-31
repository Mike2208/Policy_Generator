#ifndef MAP_HEIGHT_MAP_TEMPLATES_H
#define MAP_HEIGHT_MAP_TEMPLATES_H

#include "map_height_map.h"

// Code copied from FindMinCostPathLength
template<class T>
int HeightMap::FindMinCostPath(const Map<T> &HeightMap, const POS_2D &StartPos, const POS_2D &ZeroPos, std::vector<POS_2D> &Path)
{
	// Start at start position
	POS_2D curPos = StartPos;
//	T curCost = HeightMap.GetPixel(StartPos);		// Cost to reach current position
	T minAdjoiningVal;
	POS_2D bestAdjoiningPos;
	Map_IntType distMap;
	bool distMapCalculated = false;
	Path.clear();

	// Continue following minimum adjoining values until ZeroPos is reached
	while (curPos != ZeroPos)
	{
		// Find minimum value of all adjacent positions
		minAdjoiningVal = GetInfinteVal<T>();
		bestAdjoiningPos = curPos;
		for(unsigned int i=0; i<RobotNavigation::GetNumNextMovementPositions(); i++)
		{
			const POS_2D adjacentPos = RobotNavigation::GetNextMovementPosition(curPos, i);

			T adjacentVal;
			if(HeightMap.GetPixel(adjacentPos, adjacentVal) < 0)
				continue;

			// Update adjacent values if necessary
			if(adjacentVal < minAdjoiningVal)
			{
				minAdjoiningVal = adjacentVal;
				bestAdjoiningPos = adjacentPos;
			}
			else if(adjacentVal == minAdjoiningVal)			// if both values have same distance, select the closer one
			{
				// Check if the distMap has already been calculated ( only needs to be calculated once, and only under certain conditions)
				if(!distMapCalculated)
				{
					HeightMap::CalculateHomologyDistMap(HeightMap, ZeroPos, minAdjoiningVal, distMap);
					distMapCalculated = true;
				}

				// If adjacent position is closer, select it
				if(distMap.GetPixel(adjacentPos) < distMap.GetPixel(bestAdjoiningPos))
				{
					minAdjoiningVal = adjacentVal;
					bestAdjoiningPos = adjacentPos;
				}
			}
		}

		// Update current position and Path
		curPos = bestAdjoiningPos;
//		curCost = minAdjoiningVal;
		Path.push_back(bestAdjoiningPos);
	}

	return 1;
}

// Code copied from FindMinCostPath
template<class T>
int HeightMap::FindMinCostPathLength(const Map<T> &HeightMap, const POS_2D &StartPos, const POS_2D &ZeroPos, unsigned int &PathLength, const T *const MaxCost)
{
	// Start at start position
	POS_2D curPos = StartPos;
	T curCost = HeightMap.GetPixel(StartPos);		// Cost to reach current position
	T minAdjoiningVal;
	POS_2D bestAdjoiningPos;
	PathLength = 0;
	Map_IntType distMap;
	bool distMapCalculated = false;

	// Continue following minimum adjoining values until ZeroPos is reached
	while (curPos != ZeroPos)
	{
		// Find minimum value of all adjacent positions
		minAdjoiningVal = GetInfinteVal<T>();
		for(unsigned int i=0; i<RobotNavigation::GetNumNextMovementPositions(); i++)
		{
			const POS_2D adjacentPos = RobotNavigation::GetNextMovementPosition(curPos, i);

			T adjacentVal;
			if(HeightMap.GetPixel(adjacentPos, adjacentVal) < 0)
				continue;

			// Update adjacent values if necessary
			if(adjacentVal < minAdjoiningVal)
			{
				minAdjoiningVal = adjacentVal;
				bestAdjoiningPos = adjacentPos;
			}
			else if(adjacentVal == minAdjoiningVal)			// if both values have same distance, select the closer one
			{
				// Check if the distMap has already been calculated ( only needs to be calculated once, and only under certain conditions)
				if(!distMapCalculated)
				{
					HeightMap::CalculateHomologyDistMap(HeightMap, ZeroPos, minAdjoiningVal, distMap);
					distMapCalculated = true;
				}

				// If adjacent position is closer, select it
				if(distMap.GetPixel(adjacentPos) < distMap.GetPixel(bestAdjoiningPos))
				{
					minAdjoiningVal = adjacentVal;
					bestAdjoiningPos = adjacentPos;
				}
			}
		}

		// Update current position
		curPos = bestAdjoiningPos;

		// Update path to reach current position if necessary
		if(MaxCost != NULL)
		{
			curCost += minAdjoiningVal;
		}

		// Increment path length
		PathLength++;
	}

	// Return 0 if MaxCost exists and was exceeded
	if(MaxCost != NULL)
	{
		if(curCost >= *MaxCost)
			return 0;
	}

	return 1;
}

template<class T>
int HeightMap::CalculateHomologyDistMap(const Map<T> &OriginalMap, const POS_2D &DestPos, const T &CutoffValue, Map_IntType &HomologyMap)
{
	const unsigned int maxVal = GetInfinteVal<unsigned int>();
	const T minVal = 0;

	std::queue<POS_2D> posToCheck;

	// Set all distances to maximum and dest as zero
	HomologyMap.ResetMap(OriginalMap.GetHeight(), OriginalMap.GetWidth(), maxVal);
	HomologyMap.SetPixel(DestPos, minVal);

	// Start at destination
	posToCheck.push(DestPos);

	do
	{
		const POS_2D &curPos = posToCheck.front();
		const unsigned int curHomologyVal = HomologyMap.GetPixel(curPos);

		// Go through all adjacent positions
		for(unsigned int i=0; i<RobotNavigation::GetNumNextMovementPositions(); i++)
		{
			const POS_2D adjacentPos = RobotNavigation::GetNextMovementPosition(curPos, i);

			// Check that adjacent pos exists
			unsigned int adjacentHomologyVal;
			if(HomologyMap.GetPixel(adjacentPos, adjacentHomologyVal) < 0)
				continue;

			const T adjacentVal = OriginalMap.GetPixel(adjacentPos);
			if(adjacentVal > CutoffValue)
				continue;			// Check that adjacent position can be traversed (if not, leave value at infinity)

			// Compare values and update if quicker path is available
			if(adjacentHomologyVal > curHomologyVal+1)
			{
				HomologyMap.SetPixel(adjacentPos, curHomologyVal+1);
				posToCheck.push(adjacentPos);
			}
		}

		posToCheck.pop();
	}
	while(posToCheck.size() > 0);

	return 1;
}


#endif // MAP_HEIGHT_MAP_TEMPLATES_H
