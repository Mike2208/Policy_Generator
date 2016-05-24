#ifndef MAP_HEIGHT_MAP_TEMPLATES_H
#define MAP_HEIGHT_MAP_TEMPLATES_H

#include "map_height_map.h"

template<class T>
int HeightMap::FindMinCostPathLength(const Map<T> &HeightMap, const POS_2D &StartPos, const POS_2D &ZeroPos, T &PathLength)
{
	// Start at start position
	POS_2D curPos = StartPos;
	T curVal = HeightMap.GetPixel(curPos);
	T minAdjoiningVal;
	POS_2D bestAdjoiningPos;
	PathLength = 0;
	Map_IntType distMap;
	bool distMapCalculated = false;

	// Continue following minimum adjoining values until ZeroPos is reached
	while (curPos != ZeroPos)
	{
		// Find minimum value of all adjacent positions
		minAdjoiningVal = std::numeric_limits<T>::infinity();
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
					HeightMap::CalculateHomologyDistMap(HeightMap, ZeroPos, std::numeric_limits<T>::infinity(), distMap);
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

		// Increment path length
		PathLength++;
	}

	return 1;
}

template<class T>
int HeightMap::CalculateHomologyDistMap(const Map<T> &OriginalMap, const POS_2D &DestPos, const T &CutoffValue, Map_IntType &HomologyMap)
{
	const unsigned int maxVal = std::numeric_limits<T>::max();
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
